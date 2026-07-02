# -*- coding: utf-8 -*-
"""
简易 UDP 地面站脚本

作用：
1. 从电脑向无人机发送 UDP 控制指令；
2. 支持 START / LAND / STOP / RESET / DISARM / STATUS / PING；
3. 支持投放机构指令：
   L1 / L2 / L3
   R1 / R2 / R3
   L0 / R0

通信协议：
    地面站输入 start
    实际发送 CMD:START

    地面站输入 L1
    实际发送 CMD:L1

    地面站输入 L0
    实际依次发送：
        CMD:L1
        CMD:L2
        CMD:L3

    地面站输入 R0
    实际依次发送：
        CMD:R1
        CMD:R2
        CMD:R3

无人机端默认：
    IP   = 192.168.151.102
    PORT = 8888
"""

import socket
import time


UAV_IP = "192.168.151.102"
UAV_PORT = 8888
BOOT_PORT = UAV_PORT

RECV_TIMEOUT = 5.0

# L0 / R0 拆分发送时，每个舵机指令之间的间隔
DROP_STEP_DELAY = 0.35



def drain_socket(sock):
    """
    清空 UDP socket 中可能残留的上一条迟到回包。
    解决 L2 收到 L1 回包这种错位问题。
    """
    old_timeout = sock.gettimeout()

    try:
        sock.settimeout(0.0)

        while True:
            try:
                sock.recvfrom(4096)
            except BlockingIOError:
                break
            except socket.timeout:
                break
            except Exception:
                break
    finally:
        sock.settimeout(old_timeout)


def reply_matches(msg, reply):
    """
    判断当前回包是否属于本次发送的命令。
    如果不匹配，就认为是上一条命令的迟到回包，继续等待。
    """
    msg = msg.strip().upper()
    reply = reply.strip().upper()

    if not msg.startswith("CMD:"):
        return True

    cmd = msg[4:].strip()

    if cmd == "PING":
        return reply.startswith("ACK:PING")

    if cmd == "STATUS":
        return reply.startswith("STATUS:")

    # BOOT 回包一般是 ACK:BOOT:STARTING
    if cmd == "BOOT":
        return reply.startswith("ACK:BOOT") or reply.startswith("ERR:BOOT")

    # 普通命令期望 ACK:<CMD>:... 或 ERR:<CMD>:...
    return (
        reply.startswith("ACK:%s" % cmd) or
        reply.startswith("ERR:%s" % cmd) or
        reply.startswith("ERR:UNKNOWN_CMD:%s" % msg)
    )


def wait_reply(sock, msg):
    """
    等待本次命令对应的回包。
    若收到迟到回包，会打印并丢弃，直到收到匹配回包或总超时。
    """
    deadline = time.time() + RECV_TIMEOUT

    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            raise socket.timeout()

        old_timeout = sock.gettimeout()
        sock.settimeout(remaining)

        try:
            data, from_addr = sock.recvfrom(4096)
        finally:
            sock.settimeout(old_timeout)

        reply = data.decode("utf-8", errors="ignore").strip()

        if reply_matches(msg, reply):
            return reply, from_addr

        print("收到迟到/错位回传，已丢弃：", reply)


def print_ping_result(reply):
    """
    把接收节点返回的 PING 状态翻译成更直接的调试提示。
    """
    if reply.startswith("ACK:PING:NORMAL"):
        print("PING 结果：UDP 接收节点在线，FSM 状态机心跳正常。")
        return

    if reply.startswith("ACK:PING:RECEIVER_OK"):
        if "fsm=NO_HEARTBEAT" in reply:
            print("PING 结果：UDP 接收节点在线，但还没收到 FSM 状态机心跳。")
            return

        if "fsm=TIMEOUT" in reply:
            print("PING 结果：UDP 接收节点在线，但 FSM 状态机心跳超时。")
            return

        print("PING 结果：UDP 接收节点在线，FSM 状态未知。")
        return

    print("PING 结果：收到未知 PING 回传，请检查接收节点版本。")


def send_cmd(sock, cmd, port=None):
    """
    发送一条 CMD 指令，并等待无人机回传。
    """
    cmd = cmd.strip().upper()

    if not cmd:
        return

    # 发送新命令前先清空上一条迟到回包，避免 UDP 回包错位。
    drain_socket(sock)

    # 允许用户直接输入 START / L1 / R1
    # 也允许输入 CMD:START / CMD:L1 / CMD:R1
    if not cmd.startswith("CMD:"):
        msg = "CMD:" + cmd
    else:
        msg = cmd

    if port is None:
        port = UAV_PORT

    addr = (UAV_IP, port)

    print("\n发送：", msg)

    try:
        sock.sendto(msg.encode("utf-8"), addr)
    except Exception as e:
        print("发送失败：", e)
        return

    try:
        reply, from_addr = wait_reply(sock, msg)
        print("收到回传：", reply)
        print("来自：", from_addr[0], from_addr[1])

        if msg == "CMD:PING":
            print_ping_result(reply)

    except socket.timeout:
        print("等待回传超时。可能是无人机没收到、IP/端口不对，或无人机端节点没启动。")
    except Exception as e:
        print("接收回传失败：", e)


def send_drop_sequence(sock, prefix):
    """
    处理 L0 / R0。

    L0 不发送 CMD:L0，而是依次发送：
        CMD:L1
        CMD:L2
        CMD:L3

    R0 不发送 CMD:R0，而是依次发送：
        CMD:R1
        CMD:R2
        CMD:R3
    """
    prefix = prefix.upper()

    if prefix not in ["L", "R"]:
        print("投放序列错误：prefix 必须是 L 或 R")
        return

    print("\n{}0：开始依次发送 {}1 / {}2 / {}3".format(prefix, prefix, prefix, prefix))

    for i in range(1, 4):
        cmd = "{}{}".format(prefix, i)
        send_cmd(sock, cmd)
        time.sleep(DROP_STEP_DELAY)

    print("{}0：序列发送完成。".format(prefix))


def print_help():
    print("\n========== 简易 UDP 地面站 ==========")
    print("无人机地址：{}:{}".format(UAV_IP, UAV_PORT))
    print("回包超时：{} 秒，自动丢弃上一条迟到回包".format(RECV_TIMEOUT))
    print("BOOT 地址：{}:{}".format(UAV_IP, BOOT_PORT))
    print("")
    print("基础命令：")
    print("  boot      发送 CMD:BOOT，一键启动实飞 launch")
    print("  ping      测试 UDP 通信，并检查 FSM 状态机心跳")
    print("  status    查询无人机 FSM / MAVROS 状态")
    print("  start     发布 /uav/start=True，开始任务")
    print("  land      发布 /uav/land=True，普通 AUTO.LAND")
    print("  stop      发布 /uav/stop=True，急停 AUTO.LAND")
    print("  reset     发布 /uav/reset=True，复位 FSM")
    print("  disarm    发布 /uav/disarm=True")
    print("  loop      每 0.5 秒查询一次 STATUS")
    print("")
    print("投放机构命令：")
    print("  L1        1号舵机锁定，发送 CMD:L1")
    print("  L2        2号舵机锁定，发送 CMD:L2")
    print("  L3        3号舵机锁定，发送 CMD:L3")
    print("  R1        1号舵机释放，发送 CMD:R1")
    print("  R2        2号舵机释放，发送 CMD:R2")
    print("  R3        3号舵机释放，发送 CMD:R3")
    print("")
    print("  L0        三个舵机依次锁定，实际发送 CMD:L1 -> CMD:L2 -> CMD:L3")
    print("  R0        三个舵机依次释放，实际发送 CMD:R1 -> CMD:R2 -> CMD:R3")
    print("")
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


def is_single_drop_cmd(cmd):
    """
    判断是否为单个舵机投放机构指令：
        L1/L2/L3
        R1/R2/R3
    """
    if len(cmd) != 2:
        return False

    if cmd[0] not in ["l", "r"]:
        return False

    if cmd[1] not in ["1", "2", "3"]:
        return False

    return True


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

        if cmd == "boot":
            send_cmd(sock, "BOOT", port=BOOT_PORT)
            continue

        if cmd in ["ping", "status", "start", "land", "stop", "reset", "disarm"]:
            send_cmd(sock, cmd)
            continue

        # 单个舵机指令：L1/L2/L3/R1/R2/R3
        if is_single_drop_cmd(cmd):
            send_cmd(sock, cmd)
            continue

        # 全部动作指令：L0/R0
        # 注意：这里不发送 CMD:L0/CMD:R0，而是拆成三条单舵机指令
        if cmd == "l0":
            send_drop_sequence(sock, "L")
            continue

        if cmd == "r0":
            send_drop_sequence(sock, "R")
            continue

        print("未知命令：", user_input)
        print("输入 help 查看命令。")


if __name__ == "__main__":
    main()