#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================
Windows 简易无人机地面站：TAKEOFF / LAND 指令发送器
============================================================

【脚本运行位置】
本脚本运行在 Windows 电脑上。

【脚本功能】
1. 向无人机 Jetson 端发送 UDP 控制指令。
2. 默认目标地址：
   - 无人机 IP：192.168.151.102
   - 无人机端口：7777
3. 只有两个核心按钮：
   - 发送 TAKEOFF：无人机端收到后自动 OFFBOARD、arm、起飞并执行任务
   - 发送 LAND：无人机端收到后先零速度刹停，再自动降落并上锁

【重要说明】
1. UDP 不保证一定送达，所以本地面站会连续发送多次同一指令。
2. 如果能收到无人机端 ACK，会在日志窗口显示。
3. LAND 是软件降落指令，不等于硬件急停。实飞时仍必须保留遥控器/安全员/急停手段。
"""

# ============================================================
# 一、导入依赖库
# ============================================================

import json
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText


# ============================================================
# 二、默认参数区
# ============================================================
# 这里是你平时最可能需要修改的地方。
# 如果无人机 IP 改了，就改 DEFAULT_DRONE_IP。

DEFAULT_DRONE_IP = "192.168.151.102"  # 无人机 Jetson 的 IP 地址
DEFAULT_DRONE_PORT = 7777             # 无人机端监听端口，需要和飞行脚本保持一致

TAKEOFF_REPEAT_COUNT = 3              # TAKEOFF 指令重复发送次数
LAND_REPEAT_COUNT = 10                # LAND 指令重复发送次数。LAND 更重要，所以多发几次
SEND_INTERVAL_SEC = 0.12              # 重复发送之间的时间间隔，单位 s


# ============================================================
# 三、地面站主类
# ============================================================

class SimpleGroundStation:
    """
    简易地面站窗口类。

    负责：
    1. 创建 Windows 图形界面
    2. 通过 UDP 发送 TAKEOFF / LAND 指令
    3. 接收并显示无人机端返回的 ACK 信息
    """

    # ------------------------------------------------------------
    # 3.1 初始化界面和 UDP socket
    # ------------------------------------------------------------

    def __init__(self, root):
        """
        初始化地面站窗口。
        """
        self.root = root
        self.root.title("无人机简易地面站 - TAKEOFF / LAND")
        self.root.geometry("760x460")

        # 创建 UDP socket。
        # 这个 socket 既用于发送指令，也用于接收无人机端返回的 ACK。
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.3)

        # 标记程序是否仍在运行
        self.running = True

        # 创建界面
        self.build_ui()

        # 启动 ACK 接收线程
        self.recv_thread = threading.Thread(target=self.recv_ack_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        # 关闭窗口时执行安全清理
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.log("地面站已启动。")
        self.log("默认目标：%s:%d" % (DEFAULT_DRONE_IP, DEFAULT_DRONE_PORT))

    # ------------------------------------------------------------
    # 3.2 图形界面创建区
    # ------------------------------------------------------------

    def build_ui(self):
        """
        创建 Tkinter 图形界面。

        界面包含：
        1. 无人机 IP 输入框
        2. 无人机端口输入框
        3. TAKEOFF 按钮
        4. LAND 按钮
        5. 日志显示框
        """

        # ------------------------------
        # 顶部目标地址区域
        # ------------------------------
        target_frame = tk.LabelFrame(self.root, text="无人机目标地址", padx=10, pady=10)
        target_frame.pack(fill="x", padx=12, pady=10)

        tk.Label(target_frame, text="无人机 IP：").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value=DEFAULT_DRONE_IP)
        ip_entry = tk.Entry(target_frame, textvariable=self.ip_var, width=22)
        ip_entry.grid(row=0, column=1, padx=8)

        tk.Label(target_frame, text="端口：").grid(row=0, column=2, sticky="w")
        self.port_var = tk.StringVar(value=str(DEFAULT_DRONE_PORT))
        port_entry = tk.Entry(target_frame, textvariable=self.port_var, width=10)
        port_entry.grid(row=0, column=3, padx=8)

        # ------------------------------
        # 中部按钮区域
        # ------------------------------
        button_frame = tk.Frame(self.root)
        button_frame.pack(fill="x", padx=12, pady=10)

        self.takeoff_btn = tk.Button(
            button_frame,
            text="发送 TAKEOFF：自动 Arm 起飞并开始任务",
            height=3,
            font=("Microsoft YaHei", 12, "bold"),
            command=self.on_takeoff_clicked
        )
        self.takeoff_btn.pack(side="left", expand=True, fill="x", padx=8)

        self.land_btn = tk.Button(
            button_frame,
            text="发送 LAND：刹停后自动降落上锁",
            height=3,
            font=("Microsoft YaHei", 12, "bold"),
            bg="#ff4d4d",
            fg="white",
            activebackground="#cc0000",
            activeforeground="white",
            command=self.on_land_clicked
        )
        self.land_btn.pack(side="left", expand=True, fill="x", padx=8)

        # ------------------------------
        # 底部日志区域
        # ------------------------------
        log_frame = tk.LabelFrame(self.root, text="通信日志", padx=10, pady=10)
        log_frame.pack(fill="both", expand=True, padx=12, pady=10)

        self.log_box = ScrolledText(log_frame, height=12)
        self.log_box.pack(fill="both", expand=True)

    # ------------------------------------------------------------
    # 3.3 地址读取和日志输出区
    # ------------------------------------------------------------

    def get_target_addr(self):
        """
        从界面输入框读取目标 IP 和端口。

        返回：
        (ip, port)
        """
        ip = self.ip_var.get().strip()

        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            messagebox.showerror("端口错误", "端口必须是数字，例如 7777。")
            return None

        if not ip:
            messagebox.showerror("IP 错误", "无人机 IP 不能为空。")
            return None

        return ip, port

    def log(self, text):
        """
        向日志窗口追加一行文本。

        这里使用 root.after 是为了保证从子线程写日志时不会直接操作 Tkinter 控件。
        """
        now = time.strftime("%H:%M:%S")
        line = "[%s] %s\n" % (now, text)

        def append():
            self.log_box.insert("end", line)
            self.log_box.see("end")

        self.root.after(0, append)

    # ------------------------------------------------------------
    # 3.4 指令发送区
    # ------------------------------------------------------------

    def send_command(self, cmd, repeat_count):
        """
        发送指令到无人机。

        参数：
        cmd：指令字符串，例如 TAKEOFF 或 LAND
        repeat_count：重复发送次数

        为什么要重复发送？
        UDP 不保证一定送达。连续发送几次可以提高收到指令的概率。
        """
        target = self.get_target_addr()
        if target is None:
            return

        ip, port = target

        packet = {
            "cmd": cmd,
            "source": "windows_ground_station",
            "time": time.time()
        }

        data = json.dumps(packet, ensure_ascii=False).encode("utf-8")

        self.log("准备发送 %s 到 %s:%d，重复 %d 次。" % (cmd, ip, port, repeat_count))

        def send_worker():
            for i in range(repeat_count):
                try:
                    self.sock.sendto(data, (ip, port))
                    self.log("已发送 %s，第 %d/%d 次。" % (cmd, i + 1, repeat_count))
                except Exception as e:
                    self.log("发送 %s 失败：%s" % (cmd, str(e)))

                time.sleep(SEND_INTERVAL_SEC)

        thread = threading.Thread(target=send_worker)
        thread.daemon = True
        thread.start()

    def on_takeoff_clicked(self):
        """
        TAKEOFF 按钮回调函数。

        点击后发送 TAKEOFF 指令。
        """
        ok = messagebox.askyesno(
            "确认 TAKEOFF",
            "确认发送 TAKEOFF 指令？\n\n无人机端收到后会自动 OFFBOARD、Arm、起飞，并根据飞行脚本参数执行任务。"
        )
        if ok:
            self.send_command("TAKEOFF", TAKEOFF_REPEAT_COUNT)

    def on_land_clicked(self):
        """
        LAND 按钮回调函数。

        点击后发送 LAND 指令。
        LAND 是安全相关指令，因此重复发送次数更多。
        """
        ok = messagebox.askyesno(
            "确认 LAND",
            "确认发送 LAND 指令？\n\n无人机端收到后会先零速度刹停，再自动降落并上锁。"
        )
        if ok:
            self.send_command("LAND", LAND_REPEAT_COUNT)

    # ------------------------------------------------------------
    # 3.5 ACK 接收区
    # ------------------------------------------------------------

    def recv_ack_loop(self):
        """
        接收无人机端返回的 ACK 信息。

        注意：
        收不到 ACK 不一定代表指令没发出去，
        也可能是 Windows 防火墙、网络配置或 UDP 丢包导致。
        """
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.log("接收 ACK 出错：%s" % str(e))
                continue

            text = data.decode("utf-8", errors="ignore").strip()

            # 尝试按 JSON 格式显示
            try:
                obj = json.loads(text)
                status = obj.get("status", "")
                message = obj.get("message", "")
                mode = obj.get("mode", "")
                armed = obj.get("armed", "")
                self.log(
                    "收到 ACK 来自 %s:%d：status=%s, message=%s, mode=%s, armed=%s"
                    % (addr[0], addr[1], status, message, mode, armed)
                )
            except Exception:
                self.log("收到 ACK 来自 %s:%d：%s" % (addr[0], addr[1], text))

    # ------------------------------------------------------------
    # 3.6 关闭程序区
    # ------------------------------------------------------------

    def on_close(self):
        """
        关闭窗口时执行清理。
        """
        self.running = False

        try:
            self.sock.close()
        except Exception:
            pass

        self.root.destroy()


# ============================================================
# 四、程序入口区
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleGroundStation(root)
    root.mainloop()
