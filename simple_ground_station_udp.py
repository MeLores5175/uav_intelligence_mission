# -*- coding: utf-8 -*-
"""
简易 UDP 地面站脚本

作用：
1. 从你的电脑向无人机发送 UDP 控制指令；
2. 支持 START / LAND / STOP / RESET / DISARM / STATUS / PING；
3. 等待无人机端 UDP 接收节点返回 ACK 或 STATUS。

无人机端默认：
    IP   = 127.0.0.1
    PORT = 8888
"""

import socket
import time


UAV_IP = "10.82.0.168"
UAV_PORT = 8888

RECV_TIMEOUT = 1.0


def send_cmd(sock, cmd):
    """
    发送一条 CMD 指令，并等待无人机回传。
    """
    cmd = cmd.strip().upper()

    if not cmd:
        return

    # 允许用户直接输入 START，也允许输入 CMD:START
    if not cmd.startswith("CMD:"):
        msg = "CMD:" + cmd
    else:
        msg = cmd

    addr = (UAV_IP, UAV_PORT)

    print("\n发送：", msg)

    try:
        sock.sendto(msg.encode("utf-8"), addr)
    except Exception as e:
        print("发送失败：", e)
        return

    try:
        data, from_addr = sock.recvfrom(2048)
        reply = data.decode("utf-8", errors="ignore").strip()
        print("收到回传：", reply)
        print("来自：", from_addr[0], from_addr[1])
    except socket.timeout:
        print("等待回传超时。可能是无人机没收到、IP/端口不对，或无人机端节点没启动。")
    except Exception as e:
        print("接收回传失败：", e)


def print_help():
    print("\n========== 简易 UDP 地面站 ==========")
    print("无人机地址：{}:{}".format(UAV_IP, UAV_PORT))
    print("")
    print("可输入命令：")
    print("  ping      测试通信")
    print("  status    查询无人机 FSM / MAVROS 状态")
    print("  start     发布 /uav/start=True，开始任务")
    print("  land      发布 /uav/land=True，普通 AUTO.LAND")
    print("  stop      发布 /uav/stop=True，急停 AUTO.LAND")
    print("  reset     发布 /uav/reset=True，复位 FSM")
    print("  disarm    发布 /uav/disarm=True")
    print("  loop      每 0.5 秒查询一次 STATUS")
    print("  q         退出")
    print("====================================\n")


def status_loop(sock):
    print("\n进入状态循环，每 0.5 秒查询一次 STATUS。按 Ctrl+C 退出状态循环。")

    try:
        while True:
            send_cmd(sock, "STATUS")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n已退出状态循环。")


def main():
    print_help()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(RECV_TIMEOUT)

    while True:
        try:
            user_input = input("GROUND> ").strip()
        except KeyboardInterrupt:
            print("\n退出。")
            break

        if not user_input:
            continue

        cmd = user_input.lower()

        if cmd in ["q", "quit", "exit"]:
            print("退出。")
            break

        if cmd in ["h", "help", "?"]:
            print_help()
            continue

        if cmd == "loop":
            status_loop(sock)
            continue

        if cmd in ["ping", "status", "start", "land", "stop", "reset", "disarm"]:
            send_cmd(sock, cmd)
            continue

        print("未知命令：", user_input)
        print("输入 help 查看命令。")


if __name__ == "__main__":
    main()