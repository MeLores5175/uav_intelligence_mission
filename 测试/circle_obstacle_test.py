#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================
无人机圆形绕障飞行程序：地面站控制 + 前馈轨迹控制版
============================================================

【文件版本】
2026-07-02-v2-中文注释新文件名版

【运行位置】
本文件运行在无人机机载电脑上，例如 Jetson / Ubuntu / ROS 环境。

【核心功能】
1. 机载端监听 UDP 端口，默认监听 7777。
2. Windows 地面站发送 TAKEOFF 后，无人机自动进入 OFFBOARD、自动解锁、自动起飞。
3. 起飞后按照设定半径、圈数、速度执行圆形绕障。
4. Windows 地面站发送 LAND 后，飞行程序立即中断当前任务。
5. 收到 LAND 后，先发送当前位置悬停目标，让速度指令变成 0，再切 AUTO.LAND 降落。
6. 降落完成或接近地面后自动上锁。

【控制方式】
本脚本通过 /mavros/setpoint_raw/local 发布 mavros_msgs/PositionTarget。
也就是同时发送：
- position：期望位置
- velocity：速度前馈
- acceleration_or_force：加速度前馈
- yaw：期望航向角

【常改参数】
你平时主要改下面“二、常用参数修改区”里的：
- DEFAULT_RADIUS：绕圆半径
- DEFAULT_LOOPS：绕圆圈数
- DEFAULT_CIRCLE_SPEED：绕圆速度
- DEFAULT_TAKEOFF_ALT：起飞高度

【安全提醒】
1. 这不是硬件急停，只是软件降落指令。
2. 真机实飞必须保留遥控器接管、飞控失控保护和现场安全员。
3. 第一次实飞建议低高度、低速度、少圈数，并确保周围空旷。
"""

# ============================================================
# 一、导入依赖库区
# ============================================================

import math
import socket
import threading
import time

import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import euler_from_quaternion


# ============================================================
# 二、常用参数修改区
# ============================================================
# 这里是你最常修改的区域。
# 如果你不想在 rosrun 后面写很多 _radius:=... 参数，直接改这里即可。

# ------------------------------
# 2.1 地面站通信参数
# ------------------------------
DEFAULT_UDP_BIND_IP = "0.0.0.0"       # 监听所有网卡，Windows 地面站能连到 Jetson 即可
DEFAULT_UDP_PORT = 7777               # 和 Windows 地面站保持一致
DEFAULT_COMMAND_TIMEOUT = 0.2          # UDP 接收超时时间，单位 s，用于让线程能及时退出

# ------------------------------
# 2.2 起飞、绕圆、降落参数
# ------------------------------
DEFAULT_TAKEOFF_ALT = 0.6             # 起飞高度，单位 m
DEFAULT_RADIUS = 1.0                  # 绕圆半径，单位 m
DEFAULT_LOOPS = 3.0                  # 绕圆圈数，1.0 表示一圈
DEFAULT_CIRCLE_SPEED = 0.4            # 绕圆切向速度，单位 m/s，真机先从低速开始

DEFAULT_CENTER_X = 1.0               # 圆心相对 home 点的 x 偏移，单位 m
DEFAULT_CENTER_Y = 0.0                # 圆心相对 home 点的 y 偏移，单位 m

# ------------------------------
# 2.3 轨迹平滑和频率参数
# ------------------------------
DEFAULT_RATE_HZ = 50.0                # setpoint 发布频率，单位 Hz
DEFAULT_TAKEOFF_SPEED = 0.25          # 起飞阶段上升速度，单位 m/s
DEFAULT_TRANSFER_SPEED = 0.35         # 飞到圆起点、返航的转场速度，单位 m/s
DEFAULT_DESCEND_SPEED = 0.20          # 兜底下降速度，单位 m/s
DEFAULT_RAMP_TIME = 2.0               # 绕圆起步加速、结束减速时间，单位 s
DEFAULT_BRAKE_HOLD_TIME = 1.5         # 收到 LAND 后先悬停刹停的时间，单位 s

# ------------------------------
# 2.4 安全检查参数
# ------------------------------
DEFAULT_MAX_CENTRIPETAL_ACC = 3.0     # 最大允许向心加速度，单位 m/s^2
DEFAULT_ENFORCE_ACC_LIMIT = True      # True 表示参数太激进就拒绝执行任务

# ------------------------------
# 2.5 任务流程开关
# ------------------------------
DEFAULT_START_CIRCLE_AFTER_TAKEOFF = True   # TAKEOFF 后是否自动开始绕圆
DEFAULT_LAND_AFTER_MISSION = True           # 绕圆完成后是否自动降落
DEFAULT_AUTO_OFFBOARD = True                # 是否自动切 OFFBOARD
DEFAULT_AUTO_ARM = True                     # 收到 TAKEOFF 后是否自动解锁
DEFAULT_AUTO_DISARM = True                  # 降落后是否自动上锁
DEFAULT_USE_AUTO_LAND = True                # 优先使用 PX4 的 AUTO.LAND


# ============================================================
# 三、通用工具函数区
# ============================================================

def clamp(value, min_value, max_value):
    """
    数值限幅函数。

    作用：
    把 value 限制在 [min_value, max_value] 范围内，避免参数越界。
    """
    return max(min_value, min(max_value, value))


def wrap_pi(angle):
    """
    将角度限制到 [-pi, pi] 范围。

    作用：
    yaw 航向角如果一直累加，可能超过 pi 或 -pi。
    飞控一般更适合接收 [-pi, pi] 范围内的角度。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


# ============================================================
# 四、主飞行类定义区
# ============================================================

class CircleObstacleDemo:
    """
    圆形绕障飞行任务类。

    这个类负责：
    1. 接收地面站 UDP 指令。
    2. 和 MAVROS 交互，切模式、解锁、上锁。
    3. 发布 PositionTarget 前馈轨迹。
    4. 执行 TAKEOFF、绕圆、LAND 安全降落流程。
    """

    # ------------------------------------------------------------
    # 4.1 初始化区
    # ------------------------------------------------------------

    def __init__(self):
        """
        初始化 ROS 节点、参数、发布器、订阅器、服务客户端和 UDP 监听线程。
        """
        rospy.init_node("circle_obstacle_demo")

        # ------------------------------
        # 读取地面站通信参数
        # ------------------------------
        self.udp_bind_ip = rospy.get_param("~udp_bind_ip", DEFAULT_UDP_BIND_IP)
        self.udp_port = int(rospy.get_param("~udp_port", DEFAULT_UDP_PORT))
        self.command_timeout = float(rospy.get_param("~command_timeout", DEFAULT_COMMAND_TIMEOUT))

        # ------------------------------
        # 读取飞行参数
        # ------------------------------
        self.rate_hz = float(rospy.get_param("~rate_hz", DEFAULT_RATE_HZ))
        self.takeoff_alt = float(rospy.get_param("~takeoff_alt", DEFAULT_TAKEOFF_ALT))
        self.takeoff_speed = float(rospy.get_param("~takeoff_speed", DEFAULT_TAKEOFF_SPEED))
        self.transfer_speed = float(rospy.get_param("~transfer_speed", DEFAULT_TRANSFER_SPEED))
        self.descend_speed = float(rospy.get_param("~descend_speed", DEFAULT_DESCEND_SPEED))
        self.brake_hold_time = float(rospy.get_param("~brake_hold_time", DEFAULT_BRAKE_HOLD_TIME))

        self.radius = float(rospy.get_param("~radius", DEFAULT_RADIUS))
        self.loops = float(rospy.get_param("~loops", DEFAULT_LOOPS))
        self.circle_speed = float(rospy.get_param("~circle_speed", DEFAULT_CIRCLE_SPEED))
        self.center_x = float(rospy.get_param("~center_x", DEFAULT_CENTER_X))
        self.center_y = float(rospy.get_param("~center_y", DEFAULT_CENTER_Y))
        self.ramp_time = float(rospy.get_param("~ramp_time", DEFAULT_RAMP_TIME))

        self.max_centripetal_acc = float(rospy.get_param(
            "~max_centripetal_acc",
            DEFAULT_MAX_CENTRIPETAL_ACC
        ))
        self.enforce_acc_limit = bool(rospy.get_param(
            "~enforce_acc_limit",
            DEFAULT_ENFORCE_ACC_LIMIT
        ))

        # body：圆心相对起飞时机头方向；local：圆心直接按 MAVROS local 坐标偏移
        self.relative_frame = rospy.get_param("~relative_frame", "body")

        # False：逆时针；True：顺时针
        self.clockwise = bool(rospy.get_param("~clockwise", False))

        # fixed：机头保持起飞时方向；tangent：机头沿圆轨迹切线方向
        self.yaw_mode = rospy.get_param("~yaw_mode", "fixed")

        # ------------------------------
        # 读取任务流程开关
        # ------------------------------
        self.start_circle_after_takeoff = bool(rospy.get_param(
            "~start_circle_after_takeoff",
            DEFAULT_START_CIRCLE_AFTER_TAKEOFF
        ))
        self.land_after_mission = bool(rospy.get_param(
            "~land_after_mission",
            DEFAULT_LAND_AFTER_MISSION
        ))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", DEFAULT_AUTO_OFFBOARD))
        self.auto_arm = bool(rospy.get_param("~auto_arm", DEFAULT_AUTO_ARM))
        self.auto_disarm = bool(rospy.get_param("~auto_disarm", DEFAULT_AUTO_DISARM))
        self.use_auto_land = bool(rospy.get_param("~use_auto_land", DEFAULT_USE_AUTO_LAND))

        # ------------------------------
        # 飞控状态变量
        # ------------------------------
        self.state = State()         # 保存 /mavros/state
        self.pose = None             # 保存 /mavros/local_position/pose

        self.home_x = 0.0            # 起飞参考点 x
        self.home_y = 0.0            # 起飞参考点 y
        self.home_z = 0.0            # 起飞参考点 z
        self.home_yaw = 0.0          # 起飞参考航向角

        # ------------------------------
        # 地面站命令状态变量
        # ------------------------------
        self.command_lock = threading.Lock()   # 用锁保护多线程共享变量
        self.takeoff_requested = False         # 是否收到 TAKEOFF
        self.land_requested = False            # 是否收到 LAND
        self.mission_active = False            # 当前是否正在执行飞行任务
        self.last_ground_addr = None           # 最后一个地面站地址，用于回传 ACK
        self.udp_socket = None                 # UDP 套接字对象

        # ------------------------------
        # ROS 发布器
        # ------------------------------
        self.raw_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=20
        )

        # ------------------------------
        # ROS 订阅器
        # ------------------------------
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)

        # ------------------------------
        # ROS 服务客户端
        # ------------------------------
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")

        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.rate = rospy.Rate(self.rate_hz)

        # 启动 UDP 地面站监听线程
        self.start_udp_server()

    # ------------------------------------------------------------
    # 4.2 ROS 回调函数区
    # ------------------------------------------------------------

    def state_cb(self, msg):
        """
        飞控状态回调函数。

        /mavros/state 更新时自动调用。
        主要记录是否连接、是否解锁、当前模式等信息。
        """
        self.state = msg

    def pose_cb(self, msg):
        """
        本地位姿回调函数。

        /mavros/local_position/pose 更新时自动调用。
        主要记录无人机当前的位置和姿态。
        """
        self.pose = msg

    # ------------------------------------------------------------
    # 4.3 UDP 地面站通信区
    # ------------------------------------------------------------

    def start_udp_server(self):
        """
        启动 UDP 监听线程。

        注意：
        这里使用单独线程监听地面站指令，避免阻塞 ROS 主循环。
        """
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind((self.udp_bind_ip, self.udp_port))
        self.udp_socket.settimeout(self.command_timeout)

        thread = threading.Thread(target=self.udp_listen_loop)
        thread.daemon = True
        thread.start()

        rospy.loginfo("地面站 UDP 监听已启动：%s:%d", self.udp_bind_ip, self.udp_port)

    def udp_listen_loop(self):
        """
        UDP 指令监听循环。

        支持的指令：
        - TAKEOFF：请求自动解锁、起飞、开始任务
        - LAND：请求中断任务、刹停、自动降落、上锁
        """
        while not rospy.is_shutdown():
            try:
                data, addr = self.udp_socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            command = data.decode("utf-8", errors="ignore").strip().upper()

            with self.command_lock:
                self.last_ground_addr = addr

                if command == "TAKEOFF":
                    if self.mission_active:
                        self.send_udp_reply("BUSY：任务正在执行，忽略重复 TAKEOFF")
                    else:
                        self.takeoff_requested = True
                        self.land_requested = False
                        self.send_udp_reply("ACK：已收到 TAKEOFF")
                        rospy.logwarn("收到地面站 TAKEOFF 指令。")

                elif command == "LAND":
                    self.land_requested = True
                    self.send_udp_reply("ACK：已收到 LAND，准备刹停降落")
                    rospy.logwarn("收到地面站 LAND 指令。")

                else:
                    self.send_udp_reply("UNKNOWN：未知指令 %s" % command)
                    rospy.logwarn("收到未知地面站指令：%s", command)

    def send_udp_reply(self, text):
        """
        向地面站回传一条简单反馈。

        作用：
        Windows 地面站可以在日志框里看到机载端是否收到指令。
        """
        if self.last_ground_addr is None:
            return

        try:
            self.udp_socket.sendto(text.encode("utf-8"), self.last_ground_addr)
        except OSError:
            pass

    def is_land_requested(self):
        """
        读取 LAND 请求状态。

        使用锁是因为 UDP 线程和飞行主线程会同时访问这个变量。
        """
        with self.command_lock:
            return self.land_requested

    def wait_for_takeoff_command(self):
        """
        等待地面站发送 TAKEOFF 指令。

        程序启动后不会直接起飞，而是在这里等待。
        """
        rospy.loginfo("等待 Windows 地面站发送 TAKEOFF 指令...")
        rospy.loginfo("地面站目标端口：UDP %d", self.udp_port)

        while not rospy.is_shutdown():
            with self.command_lock:
                if self.takeoff_requested:
                    self.takeoff_requested = False
                    self.mission_active = True
                    return True
            self.rate.sleep()

        return False

    # ------------------------------------------------------------
    # 4.4 飞控基础工具函数区
    # ------------------------------------------------------------

    def get_yaw_from_pose(self, pose_msg):
        """
        从 PoseStamped 姿态四元数中提取 yaw 航向角。
        """
        q = pose_msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def set_mode(self, mode):
        """
        切换飞行模式。

        常用模式：
        - OFFBOARD：外部控制模式
        - AUTO.LAND：自动降落模式
        """
        try:
            resp = self.mode_srv(base_mode=0, custom_mode=mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("切换模式失败：%s", str(e))
            return False

    def arm(self, value=True):
        """
        解锁或上锁无人机。

        value=True：解锁
        value=False：上锁
        """
        try:
            resp = self.arm_srv(value)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("解锁/上锁服务调用失败：%s", str(e))
            return False

    def wait_until_ready(self):
        """
        等待 MAVROS 连接飞控，并等待本地位置数据。
        """
        rospy.loginfo("等待 MAVROS 连接飞控...")
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()

        rospy.loginfo("MAVROS 已连接，等待本地位置数据...")
        while not rospy.is_shutdown() and self.pose is None:
            self.rate.sleep()

        self.home_x = self.pose.pose.position.x
        self.home_y = self.pose.pose.position.y
        self.home_z = self.pose.pose.position.z
        self.home_yaw = self.get_yaw_from_pose(self.pose)

        rospy.loginfo(
            "Home 点已记录：x=%.3f, y=%.3f, z=%.3f, yaw=%.1f 度",
            self.home_x,
            self.home_y,
            self.home_z,
            math.degrees(self.home_yaw)
        )

    # ------------------------------------------------------------
    # 4.5 PositionTarget 目标生成与发布区
    # ------------------------------------------------------------

    def make_raw_target(self, x, y, z, vx, vy, vz, ax, ay, az, yaw):
        """
        创建 PositionTarget 控制消息。

        这里同时填入位置、速度前馈、加速度前馈和 yaw。
        """
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # 这里沿用 MAVROS setpoint_raw/local 常见写法。
        # 不要手动把 z 改成负数，本脚本按 /mavros/local_position/pose 的 z 向上为正来使用。
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # 只忽略 yaw_rate，其余位置、速度、加速度、yaw 都有效。
        msg.type_mask = PositionTarget.IGNORE_YAW_RATE

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.acceleration_or_force.x = ax
        msg.acceleration_or_force.y = ay
        msg.acceleration_or_force.z = az

        msg.yaw = wrap_pi(yaw)
        msg.yaw_rate = 0.0

        return msg

    def publish_raw_target(self, x, y, z, vx=0.0, vy=0.0, vz=0.0,
                           ax=0.0, ay=0.0, az=0.0, yaw=None):
        """
        发布 PositionTarget 控制目标。
        """
        if yaw is None:
            yaw = self.home_yaw

        msg = self.make_raw_target(x, y, z, vx, vy, vz, ax, ay, az, yaw)
        self.raw_pub.publish(msg)

    # ------------------------------------------------------------
    # 4.6 OFFBOARD 准备区
    # ------------------------------------------------------------

    def prepare_offboard_and_arm(self):
        """
        进入 OFFBOARD 并解锁。

        PX4 要求：切入 OFFBOARD 前必须先连续发送 setpoint。
        所以这里先发 2 秒当前位置保持目标，再请求 OFFBOARD 和解锁。
        """
        rospy.loginfo("进入 OFFBOARD 前，先连续发送当前位置保持 setpoint...")

        for _ in range(int(self.rate_hz * 2.0)):
            if self.is_land_requested():
                return False
            self.publish_raw_target(self.home_x, self.home_y, self.home_z)
            self.rate.sleep()

        if self.auto_offboard:
            rospy.loginfo("尝试切换到 OFFBOARD 模式...")
            self.set_mode("OFFBOARD")

        if self.auto_arm:
            rospy.loginfo("尝试自动解锁...")
            self.arm(True)

        last_mode_request = rospy.Time.now()
        last_arm_request = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.is_land_requested():
                return False

            if self.state.mode == "OFFBOARD" and self.state.armed:
                rospy.loginfo("无人机已进入 OFFBOARD，并完成解锁。")
                return True

            now = rospy.Time.now()

            if self.auto_offboard and self.state.mode != "OFFBOARD":
                if (now - last_mode_request).to_sec() > 2.0:
                    rospy.loginfo("重试切换 OFFBOARD...")
                    self.set_mode("OFFBOARD")
                    last_mode_request = now

            if self.auto_arm and not self.state.armed:
                if (now - last_arm_request).to_sec() > 2.0:
                    rospy.loginfo("重试自动解锁...")
                    self.arm(True)
                    last_arm_request = now

            self.publish_raw_target(self.home_x, self.home_y, self.home_z)
            self.rate.sleep()

        return False

    # ------------------------------------------------------------
    # 4.7 平滑轨迹工具区
    # ------------------------------------------------------------

    def smoother(self, u):
        """
        五次多项式平滑函数。

        作用：
        让起点和终点的速度、加速度都比较平滑，减少突然冲击。
        """
        u = clamp(u, 0.0, 1.0)
        s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        ds = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
        dds = 60.0 * u - 180.0 * u**2 + 120.0 * u**3
        return s, ds, dds

    def move_to(self, x, y, z, yaw=None, speed=None, hold_time=0.5):
        """
        平滑移动到指定位置。

        如果移动过程中收到 LAND，函数会立刻返回 False，主流程随后执行安全降落。
        """
        if yaw is None:
            yaw = self.home_yaw
        if speed is None:
            speed = self.transfer_speed

        sx = self.pose.pose.position.x
        sy = self.pose.pose.position.y
        sz = self.pose.pose.position.z

        dx = x - sx
        dy = y - sy
        dz = z - sz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        duration = max(dist / max(speed, 0.05), 1.5)
        steps = max(int(duration * self.rate_hz), 1)

        rospy.loginfo("平滑移动到 x=%.2f, y=%.2f, z=%.2f，预计 %.2f 秒", x, y, z, duration)

        for i in range(steps + 1):
            if rospy.is_shutdown():
                return False
            if self.is_land_requested():
                return False

            u = float(i) / float(steps)
            s, ds, dds = self.smoother(u)

            px = sx + dx * s
            py = sy + dy * s
            pz = sz + dz * s

            vx = dx * ds / duration
            vy = dy * ds / duration
            vz = dz * ds / duration

            ax = dx * dds / (duration * duration)
            ay = dy * dds / (duration * duration)
            az = dz * dds / (duration * duration)

            self.publish_raw_target(px, py, pz, vx, vy, vz, ax, ay, az, yaw)
            self.rate.sleep()

        return self.hold_position(x, y, z, yaw, hold_time)

    def hold_position(self, x, y, z, yaw=None, hold_time=1.0):
        """
        悬停保持指定时间。

        悬停时速度前馈和加速度前馈都给 0。
        """
        if yaw is None:
            yaw = self.home_yaw

        steps = max(int(hold_time * self.rate_hz), 1)

        for _ in range(steps):
            if rospy.is_shutdown():
                return False
            if self.is_land_requested():
                return False

            self.publish_raw_target(x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw)
            self.rate.sleep()

        return True

    # ------------------------------------------------------------
    # 4.8 圆形轨迹计算区
    # ------------------------------------------------------------

    def get_circle_center_abs(self):
        """
        计算圆心在 MAVROS local 坐标系下的绝对位置。
        """
        if self.relative_frame == "body":
            c = math.cos(self.home_yaw)
            s = math.sin(self.home_yaw)
            dx_local = c * self.center_x - s * self.center_y
            dy_local = s * self.center_x + c * self.center_y
        else:
            dx_local = self.center_x
            dy_local = self.center_y

        return self.home_x + dx_local, self.home_y + dy_local

    def check_circle_feasible(self):
        """
        检查绕圆参数是否过于激进。

        向心加速度公式：a = v^2 / r。
        速度越高、半径越小，所需向心加速度越大。
        """
        if self.radius <= 0.05:
            rospy.logerr("绕圆半径太小，任务终止。")
            return False

        if self.circle_speed <= 0.05:
            rospy.logerr("绕圆速度太小，任务终止。")
            return False

        a_req = self.circle_speed * self.circle_speed / self.radius
        tilt_deg = math.degrees(math.atan2(a_req, 9.81))

        rospy.loginfo(
            "绕圆参数检查：半径=%.2f m，速度=%.2f m/s，向心加速度=%.2f m/s^2，估计倾角=%.1f 度",
            self.radius,
            self.circle_speed,
            a_req,
            tilt_deg
        )

        if a_req > self.max_centripetal_acc:
            rospy.logwarn("当前向心加速度超过限制，建议减速或增大半径。")
            if self.enforce_acc_limit:
                rospy.logerr("由于 enforce_acc_limit=True，本次任务终止。")
                return False

        return True

    def circle_speed_profile(self, t, ramp_t, cruise_t, omega):
        """
        圆周角速度规划函数。

        输出当前已经转过的角度、当前角速度、当前角加速度。
        这样可以实现绕圆起步平滑加速、结束平滑减速。
        """
        if ramp_t <= 1e-6:
            return omega * t, omega, 0.0

        ramp_angle = 0.5 * omega * ramp_t

        if t < ramp_t:
            u = t / ramp_t
            s, ds, _ = self.smoother(u)
            integral_s = 2.5 * u**4 - 3.0 * u**5 + u**6
            theta_abs = omega * ramp_t * integral_s
            omega_abs = omega * s
            alpha_abs = omega * ds / ramp_t
            return theta_abs, omega_abs, alpha_abs

        if t < ramp_t + cruise_t:
            tc = t - ramp_t
            theta_abs = ramp_angle + omega * tc
            return theta_abs, omega, 0.0

        td = t - ramp_t - cruise_t
        u = clamp(td / ramp_t, 0.0, 1.0)
        s, ds, _ = self.smoother(u)
        integral_s = 2.5 * u**4 - 3.0 * u**5 + u**6
        theta_abs = ramp_angle + omega * cruise_t + omega * ramp_t * (u - integral_s)
        omega_abs = omega * (1.0 - s)
        alpha_abs = -omega * ds / ramp_t
        return theta_abs, omega_abs, alpha_abs

    def fly_circle(self):
        """
        执行圆形绕飞。

        这里会同时生成：
        1. 圆周位置
        2. 切向速度前馈
        3. 向心加速度前馈
        """
        target_z = self.home_z + self.takeoff_alt
        cx, cy = self.get_circle_center_abs()
        direction = -1.0 if self.clockwise else 1.0
        start_theta = 0.0

        start_x = cx + self.radius * math.cos(start_theta)
        start_y = cy + self.radius * math.sin(start_theta)

        rospy.loginfo(
            "圆形绕飞准备：圆心 x=%.2f, y=%.2f，半径=%.2f，圈数=%.2f，速度=%.2f",
            cx,
            cy,
            self.radius,
            self.loops,
            self.circle_speed
        )

        if not self.move_to(start_x, start_y, target_z, self.home_yaw, self.transfer_speed, 1.0):
            return False

        angle_total_abs = 2.0 * math.pi * abs(self.loops)
        omega = self.circle_speed / self.radius
        ramp_t = min(self.ramp_time, angle_total_abs / max(omega, 1e-6))
        cruise_angle = max(angle_total_abs - omega * ramp_t, 0.0)
        cruise_t = cruise_angle / max(omega, 1e-6)
        total_time = 2.0 * ramp_t + cruise_t

        rospy.loginfo(
            "开始绕圆：最大角速度=%.2f rad/s，加速=%.2f s，匀速=%.2f s，总时间=%.2f s",
            omega,
            ramp_t,
            cruise_t,
            total_time
        )

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.is_land_requested():
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= total_time:
                break

            theta_abs, omega_abs, alpha_abs = self.circle_speed_profile(elapsed, ramp_t, cruise_t, omega)
            theta = start_theta + direction * theta_abs
            theta_dot = direction * omega_abs
            theta_ddot = direction * alpha_abs

            x = cx + self.radius * math.cos(theta)
            y = cy + self.radius * math.sin(theta)
            z = target_z

            vx = -self.radius * math.sin(theta) * theta_dot
            vy = self.radius * math.cos(theta) * theta_dot
            vz = 0.0

            ax = -self.radius * math.cos(theta) * theta_dot * theta_dot - self.radius * math.sin(theta) * theta_ddot
            ay = -self.radius * math.sin(theta) * theta_dot * theta_dot + self.radius * math.cos(theta) * theta_ddot
            az = 0.0

            if self.yaw_mode == "tangent":
                yaw = theta + direction * math.pi / 2.0
            else:
                yaw = self.home_yaw

            self.publish_raw_target(x, y, z, vx, vy, vz, ax, ay, az, yaw)
            self.rate.sleep()

        end_theta = start_theta + direction * angle_total_abs
        end_x = cx + self.radius * math.cos(end_theta)
        end_y = cy + self.radius * math.sin(end_theta)

        if not self.hold_position(end_x, end_y, target_z, self.home_yaw, 1.0):
            return False

        rospy.loginfo("圆形绕飞结束。")
        return True

    # ------------------------------------------------------------
    # 4.9 降落和刹停区
    # ------------------------------------------------------------

    def brake_to_zero_then_land(self):
        """
        收到 LAND 后执行的安全降落流程。

        流程：
        1. 读取当前位置。
        2. 连续发布当前位置悬停 setpoint，速度前馈和加速度前馈都为 0。
        3. 尝试切换 AUTO.LAND。
        4. 降落接近地面后自动上锁。
        5. 如果 AUTO.LAND 不成功，则使用 OFFBOARD setpoint 兜底缓慢下降。
        """
        if self.pose is None:
            rospy.logerr("没有当前位置数据，无法执行安全降落。")
            return

        current_x = self.pose.pose.position.x
        current_y = self.pose.pose.position.y
        current_z = self.pose.pose.position.z
        current_yaw = self.get_yaw_from_pose(self.pose)

        rospy.logwarn("开始刹停：发送当前位置悬停目标，速度前馈=0，加速度前馈=0。")

        for _ in range(int(self.rate_hz * self.brake_hold_time)):
            if rospy.is_shutdown():
                return
            self.publish_raw_target(current_x, current_y, current_z, 0, 0, 0, 0, 0, 0, current_yaw)
            self.rate.sleep()

        if self.use_auto_land:
            rospy.logwarn("刹停完成，尝试切换 AUTO.LAND。")
            if self.set_mode("AUTO.LAND"):
                start_wait = rospy.Time.now()

                while not rospy.is_shutdown():
                    if not self.state.armed:
                        rospy.loginfo("AUTO.LAND 完成，无人机已上锁。")
                        return

                    rel_z = self.pose.pose.position.z - self.home_z
                    if rel_z < 0.15:
                        rospy.loginfo("无人机已经接近地面。")
                        break

                    if (rospy.Time.now() - start_wait).to_sec() > 20.0:
                        rospy.logwarn("AUTO.LAND 等待超时，准备使用兜底下降。")
                        break

                    self.rate.sleep()
            else:
                rospy.logwarn("AUTO.LAND 请求失败，准备使用兜底下降。")

        self.fallback_descend_and_disarm(current_yaw)

    def fallback_descend_and_disarm(self, yaw):
        """
        AUTO.LAND 失败时的兜底下降。

        这个函数不再检查 land_requested，因为它本身就是 LAND 流程的一部分。
        """
        rospy.logwarn("开始 OFFBOARD 兜底缓慢下降。")

        current_x = self.pose.pose.position.x
        current_y = self.pose.pose.position.y
        current_z = self.pose.pose.position.z
        final_z = self.home_z + 0.08

        descend_time = max((current_z - final_z) / max(self.descend_speed, 0.05), 1.0)
        steps = max(int(descend_time * self.rate_hz), 1)

        if self.state.mode != "OFFBOARD":
            for _ in range(int(self.rate_hz * 1.0)):
                self.publish_raw_target(current_x, current_y, current_z, 0, 0, 0, 0, 0, 0, yaw)
                self.rate.sleep()
            self.set_mode("OFFBOARD")

        for i in range(steps + 1):
            if rospy.is_shutdown():
                return
            a = float(i) / float(steps)
            z = current_z + (final_z - current_z) * a
            self.publish_raw_target(current_x, current_y, z, 0, 0, 0, 0, 0, 0, yaw)
            self.rate.sleep()

        if self.auto_disarm and self.state.armed:
            rospy.logwarn("兜底下降结束，执行上锁。")
            self.arm(False)

    def normal_land_after_mission(self):
        """
        任务正常结束后的降落流程。

        先回到 home 点上方，再执行刹停降落。
        """
        target_z = self.home_z + self.takeoff_alt
        if not self.move_to(self.home_x, self.home_y, target_z, self.home_yaw, self.transfer_speed, 1.0):
            self.brake_to_zero_then_land()
            return
        self.brake_to_zero_then_land()

    def wait_land_command_after_mission(self):
        """
        如果 land_after_mission=False，绕圆完成后悬停等待地面站 LAND。
        """
        rospy.loginfo("任务已完成，保持悬停，等待地面站 LAND 指令。")

        target_z = self.home_z + self.takeoff_alt
        while not rospy.is_shutdown():
            if self.is_land_requested():
                self.brake_to_zero_then_land()
                return
            self.publish_raw_target(self.pose.pose.position.x, self.pose.pose.position.y, target_z, 0, 0, 0, 0, 0, 0, self.home_yaw)
            self.rate.sleep()

    # ------------------------------------------------------------
    # 4.10 主任务流程区
    # ------------------------------------------------------------

    def execute_mission_once(self):
        """
        执行一次完整任务。

        TAKEOFF 后流程：
        1. 检查绕圆参数。
        2. 进入 OFFBOARD 并解锁。
        3. 起飞到目标高度。
        4. 根据开关决定是否绕圆。
        5. 根据开关决定任务后自动降落或悬停等待 LAND。
        """
        with self.command_lock:
            self.land_requested = False

        if not self.check_circle_feasible():
            return

        if not self.prepare_offboard_and_arm():
            self.brake_to_zero_then_land()
            return

        target_z = self.home_z + self.takeoff_alt
        rospy.loginfo("开始起飞到 %.2f m。", self.takeoff_alt)

        if not self.move_to(self.home_x, self.home_y, target_z, self.home_yaw, self.takeoff_speed, 2.0):
            self.brake_to_zero_then_land()
            return

        if self.start_circle_after_takeoff:
            if not self.fly_circle():
                self.brake_to_zero_then_land()
                return
        else:
            rospy.loginfo("已起飞悬停，未自动开始绕圆。")

        if self.land_after_mission:
            self.normal_land_after_mission()
        else:
            self.wait_land_command_after_mission()

    def run(self):
        """
        程序主循环。

        程序启动后：
        1. 等待 MAVROS 和本地位置。
        2. 等待地面站 TAKEOFF。
        3. 执行一次任务。
        4. 任务结束后继续等待下一次 TAKEOFF。
        """
        self.wait_until_ready()

        while not rospy.is_shutdown():
            if not self.wait_for_takeoff_command():
                break

            try:
                self.execute_mission_once()
            finally:
                with self.command_lock:
                    self.mission_active = False
                    self.land_requested = False
                rospy.loginfo("本次任务流程结束，重新等待 TAKEOFF。")


# ============================================================
# 五、程序入口区
# ============================================================

if __name__ == "__main__":
    try:
        node = CircleObstacleDemo()
        node.run()
    except rospy.ROSInterruptException:
        pass
