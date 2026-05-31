#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UDP UAV Command Receiver

作用：
1. 在无人机端监听 UDP 指令；
2. 将电脑端发送的简单字符串协议转换成 FSM 的 ROS 控制话题；
3. 给电脑端返回 ACK / STATUS，方便后续写简易地面站脚本。

默认监听：
    192.168.151.102:8888

如果后面想监听所有网卡，可以把 bind_ip 改成 0.0.0.0，
或者运行时传参：
    _bind_ip:=0.0.0.0
"""

import socket
import rospy

from std_msgs.msg import Bool, String

try:
    from mavros_msgs.msg import State
    HAS_MAVROS_MSG = True
except Exception:
    HAS_MAVROS_MSG = False


class UdpUavCmdReceiver:
    def __init__(self):
        rospy.init_node("udp_uav_cmd_receiver", anonymous=False)

        # 你说先写成 192.168.151.102，后面可以自己改
        self.bind_ip = rospy.get_param("~bind_ip", "192.168.151.102")
        self.bind_port = int(rospy.get_param("~bind_port", 8888))

        # 可选安全口令。默认空，不校验。
        # 若设置 _auth_token:=abc123，则电脑端需要发送 CMD:START:abc123
        self.auth_token = str(rospy.get_param("~auth_token", "")).strip()

        # FSM 控制话题
        self.start_pub = rospy.Publisher("/uav/start", Bool, queue_size=10)
        self.land_pub = rospy.Publisher("/uav/land", Bool, queue_size=10)
        self.stop_pub = rospy.Publisher("/uav/stop", Bool, queue_size=10)
        self.reset_pub = rospy.Publisher("/uav/reset", Bool, queue_size=10)
        self.disarm_pub = rospy.Publisher("/uav/disarm", Bool, queue_size=10)

        # 读取 FSM 状态，便于电脑端 STATUS 查询
        self.fsm_state = "UNKNOWN"
        self.safety_state = "UNKNOWN"
        self.land_status = "UNKNOWN"

        rospy.Subscriber("/uav/fsm_state", String, self.fsm_state_cb, queue_size=10)
        rospy.Subscriber("/uav/safety_state", String, self.safety_state_cb, queue_size=10)
        rospy.Subscriber("/uav/land_status", String, self.land_status_cb, queue_size=10)

        # 可选读取 MAVROS 状态
        self.mavros_connected = "UNKNOWN"
        self.mavros_armed = "UNKNOWN"
        self.mavros_mode = "UNKNOWN"

        if HAS_MAVROS_MSG:
            rospy.Subscriber("/mavros/state", State, self.mavros_state_cb, queue_size=10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.bind_port))
        self.sock.settimeout(0.1)

        rospy.logwarn(
            "UDP UAV command receiver started. Listening on %s:%d",
            self.bind_ip,
            self.bind_port
        )

        rospy.logwarn(
            "Supported commands: CMD:PING, CMD:STATUS, CMD:START, CMD:LAND, CMD:STOP, CMD:RESET, CMD:DISARM"
        )

    def fsm_state_cb(self, msg):
        self.fsm_state = msg.data

    def safety_state_cb(self, msg):
        self.safety_state = msg.data

    def land_status_cb(self, msg):
        self.land_status = msg.data

    def mavros_state_cb(self, msg):
        self.mavros_connected = str(msg.connected)
        self.mavros_armed = str(msg.armed)
        self.mavros_mode = msg.mode

    def send_reply(self, text, addr):
        try:
            self.sock.sendto((text + "\n").encode("utf-8"), addr)
        except Exception as e:
            rospy.logwarn("Failed to send UDP reply to %s: %s", str(addr), str(e))

    def publish_bool_pulse(self, pub, topic_name):
        """
        连续发布几次 True，提高 UDP 单次命令转 ROS 话题的可靠性。
        不发布 False，因为 FSM 只关心 True 触发。
        """
        msg = Bool(data=True)

        for _ in range(3):
            pub.publish(msg)
            rospy.sleep(0.03)

        rospy.logwarn("Published %s=True", topic_name)

    def parse_command(self, raw_text):
        """
        支持格式：
            CMD:START
            CMD:START:token

        返回：
            command, error
        """
        text = raw_text.strip()

        if not text:
            return None, "EMPTY"

        parts = text.split(":")

        if len(parts) < 2:
            return None, "BAD_FORMAT"

        if parts[0].upper() != "CMD":
            return None, "BAD_PREFIX"

        cmd = parts[1].strip().upper()

        # 安全口令校验。默认 auth_token 为空，不校验。
        if self.auth_token:
            if len(parts) < 3:
                return None, "AUTH_REQUIRED"

            token = parts[2].strip()
            if token != self.auth_token:
                return None, "AUTH_FAILED"

        # 兼容一些别名，电脑端写起来更自由
        alias = {
            "TAKEOFF": "START",
            "BEGIN": "START",
            "MISSION": "START",
            "RTL": "LAND",
            "EMERGENCY": "STOP",
            "E_STOP": "STOP",
            "KILL": "STOP",
        }

        cmd = alias.get(cmd, cmd)
        return cmd, None

    def build_status_text(self):
        return (
            "STATUS:"
            "fsm=%s;"
            "safety=%s;"
            "land=%s;"
            "connected=%s;"
            "armed=%s;"
            "mode=%s"
        ) % (
            self.fsm_state,
            self.safety_state,
            self.land_status,
            self.mavros_connected,
            self.mavros_armed,
            self.mavros_mode
        )

    def handle_command(self, raw_text, addr):
        cmd, error = self.parse_command(raw_text)

        if error:
            reply = "ERR:%s:%s" % (error, raw_text.strip())
            rospy.logwarn("UDP command rejected from %s: %s", str(addr), reply)
            self.send_reply(reply, addr)
            return

        rospy.logwarn("UDP command received from %s: CMD:%s", str(addr), cmd)

        if cmd == "PING":
            self.send_reply("ACK:PING:OK", addr)
            return

        if cmd == "STATUS":
            self.send_reply(self.build_status_text(), addr)
            return

        if cmd == "START":
            self.publish_bool_pulse(self.start_pub, "/uav/start")
            self.send_reply("ACK:START:OK", addr)
            return

        if cmd == "LAND":
            self.publish_bool_pulse(self.land_pub, "/uav/land")
            self.send_reply("ACK:LAND:OK", addr)
            return

        if cmd == "STOP":
            self.publish_bool_pulse(self.stop_pub, "/uav/stop")
            self.send_reply("ACK:STOP:OK", addr)
            return

        if cmd == "RESET":
            self.publish_bool_pulse(self.reset_pub, "/uav/reset")
            self.send_reply("ACK:RESET:OK", addr)
            return

        if cmd == "DISARM":
            self.publish_bool_pulse(self.disarm_pub, "/uav/disarm")
            self.send_reply("ACK:DISARM:OK", addr)
            return

        self.send_reply("ERR:UNKNOWN_CMD:%s" % raw_text.strip(), addr)

    def spin(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                rate.sleep()
                continue
            except Exception as e:
                rospy.logwarn("UDP recv error: %s", str(e))
                rate.sleep()
                continue

            try:
                text = data.decode("utf-8", errors="ignore").strip()
            except Exception:
                self.send_reply("ERR:DECODE_FAILED", addr)
                continue

            if text:
                self.handle_command(text, addr)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = UdpUavCmdReceiver()
        node.spin()
    except rospy.ROSInterruptException:
        pass