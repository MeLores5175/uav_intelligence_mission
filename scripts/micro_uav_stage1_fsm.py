#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
微型无人机第一阶段 FSM：安全控制增强版
功能：
1. 从 YAML 读取任务航点；
2. 按 FSM 执行起飞、二维码点、绕障点、图片靶点、特殊靶点、圆环前等待点；
3. route 点：位置控制快速通过，适合精度要求不高的普通航段；
4. scan/drop/ring 点：位置 + 速度融合控制，适合二维码、图片靶、特殊靶、穿环等复杂动作；
5. 每个航点先原地转向，再平移，禁止边平移边旋转；
6. 增加 /uav/start、/uav/stop、/uav/land、/uav/disarm、/uav/reset 安全控制话题；
7. /uav/stop 定义为“急停降落”：立即取消任务，切 AUTO.LAND，落地后自动 disarm，之后必须 reset 才能再次 start。
"""

import os
import math
from dataclasses import dataclass

import yaml
import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import euler_from_quaternion


# =========================
# 01. 数据结构定义区
# =========================

@dataclass
class Waypoint:
    """
    航点结构体。
    name: 航点名称。
    kind: 航点类型，route 或 scan。
    action: 到达并停稳后执行的动作。
    x, y, z: 相对起飞点坐标，单位 m。
    yaw_deg: 目标航向角，单位 deg。
    speed: 当前航点最大速度，单位 m/s，主要用于融合控制的速度前馈限幅。
    acc: 当前航点最大加速度，单位 m/s^2，用于限制融合控制速度变化。
    hold_time: scan 点停稳后的保持时间，单位 s。
    control_mode: 控制模式。auto 根据 kind 自动选择；position 为纯位置控制；fusion 为位置速度融合并停稳；fusion_route 为位置速度融合快速通过。
    dynamic_land: 是否使用二维码中的 left/right 动态替换该航点 y 坐标。
    """

    name: str
    kind: str
    action: str
    x: float
    y: float
    z: float
    yaw_deg: float
    speed: float
    acc: float
    hold_time: float = 0.5
    control_mode: str = "auto"
    dynamic_land: bool = False

    def yaw_rad(self):
        """将角度制 yaw 转成弧度制 yaw。"""
        return math.radians(self.yaw_deg)


# =========================
# 02. 通用数学函数区
# =========================

def norm3(vx, vy, vz):
    """计算三维向量模长。"""
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def clamp(value, min_value, max_value):
    """将 value 限制在 [min_value, max_value] 范围内。"""
    return max(min_value, min(max_value, value))


def wrap_pi(angle):
    """将角度限制到 [-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def limit_vector_norm(vx, vy, vz, max_norm):
    """限制速度向量的最大模长。"""
    length = norm3(vx, vy, vz)

    if length < 1e-6:
        return 0.0, 0.0, 0.0

    if length <= max_norm:
        return vx, vy, vz

    scale = max_norm / length
    return vx * scale, vy * scale, vz * scale


def limit_vector_change(old_v, new_v, max_delta):
    """限制速度指令变化量，用于实现加速度约束。"""
    dx = new_v[0] - old_v[0]
    dy = new_v[1] - old_v[1]
    dz = new_v[2] - old_v[2]

    delta_len = norm3(dx, dy, dz)

    if delta_len < 1e-6:
        return new_v

    if delta_len <= max_delta:
        return new_v

    scale = max_delta / delta_len

    return [
        old_v[0] + dx * scale,
        old_v[1] + dy * scale,
        old_v[2] + dz * scale,
    ]


# =========================
# 03. FSM 主类
# =========================

class MicroUAVStage1FSM:
    def __init__(self):
        """初始化节点、读取 YAML、建立 ROS 通信。"""

        rospy.init_node("micro_uav_stage1_fsm")

        # ---------- YAML 路径 ----------
        self.mission_yaml = rospy.get_param(
            "~mission_yaml",
            os.path.expanduser("~/catkin_ws/src/uav_inventory/config/stage1_mission.yaml")
        )

        self.mission_cfg = self.load_yaml(self.mission_yaml)

        # ---------- 任务参数 ----------
        self.frame_mode = self.mission_cfg["mission"].get("frame", "relative_home")
        self.wait_real_qr = self.mission_cfg["mission"].get("wait_real_qr", False)
        self.wait_real_image = self.mission_cfg["mission"].get("wait_real_image", False)

        self.control_cfg = self.mission_cfg["mission"].get("control", {})
        self.action_cfg = self.mission_cfg["mission"].get("action", {})
        self.takeoff_cfg = self.mission_cfg["mission"].get("takeoff", {})
        self.landing_cfg = self.mission_cfg["mission"].get("landing", {})

        self.waypoints = self.parse_waypoints(self.mission_cfg)

        # ---------- 控制频率 ----------
        self.rate_hz = float(self.control_cfg.get("rate_hz", 30.0))

        # ---------- yaw 控制参数 ----------
        self.yaw_eps = math.radians(float(self.control_cfg.get("yaw_eps_deg", 5.0)))
        self.yaw_break_eps = math.radians(float(self.control_cfg.get("yaw_break_eps_deg", 12.0)))

        # ---------- route 点判断参数 ----------
        self.route_pos_eps = float(self.control_cfg.get("route_pos_eps", 0.18))
        self.route_finish_vel = float(self.control_cfg.get("route_finish_vel", 0.18))
        self.route_finish_acc = float(self.control_cfg.get("route_finish_acc", 0.50))

        # ---------- scan 点判断参数 ----------
        self.scan_pos_eps = float(self.control_cfg.get("scan_pos_eps", 0.08))
        self.scan_kp = float(self.control_cfg.get("scan_kp", 0.70))
        self.scan_stable_vel = float(self.control_cfg.get("scan_stable_vel", 0.08))
        self.scan_stable_acc = float(self.control_cfg.get("scan_stable_acc", 0.25))
        self.scan_stable_time = float(self.control_cfg.get("scan_stable_time", 0.80))

        # ---------- 起飞参数 ----------
        self.takeoff_height = float(self.takeoff_cfg.get("z", 1.30))
        self.takeoff_yaw_deg = float(self.takeoff_cfg.get("yaw_deg", 0.0))
        self.takeoff_stable_time = float(self.takeoff_cfg.get("stable_time", 0.80))

        # ---------- 动作参数 ----------
        self.qr_scan_timeout = float(self.action_cfg.get("qr_scan_timeout", 3.0))
        self.image_scan_time = float(self.action_cfg.get("image_scan_time", 1.5))
        self.drop_time = float(self.action_cfg.get("drop_time", 1.2))
        self.hold_after_action = float(self.action_cfg.get("hold_after_action", 0.3))

        # ---------- 是否自动切模式、自动解锁 ----------
        self.auto_set_mode = rospy.get_param("~auto_set_mode", False)
        self.auto_arm = rospy.get_param("~auto_arm", False)

        # ---------- 飞控状态 ----------
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.extended_state_ok = False

        self.current_pose = None
        self.current_yaw = 0.0

        self.current_vel = [0.0, 0.0, 0.0]
        self.current_speed = 0.0
        self.current_acc_norm = 0.0

        self.last_vel = None
        self.last_vel_time = None

        # ---------- home 原点 ----------
        self.home_ready = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0

        # ---------- FSM 状态 ----------
        # WAIT_START：节点启动后的默认状态，必须收到 /uav/start=True 才开始执行任务。
        self.fsm_state = "WAIT_START"
        self.safety_state = "IDLE"
        self.land_status = "IDLE"
        self.last_land_status = ""

        self.nav_phase = "INIT"
        self.wp_index = 0

        self.locked_yaw = 0.0
        self.cmd_vel = [0.0, 0.0, 0.0]

        self.phase_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.action_start_time = None
        self.action_sent = False

        # ---------- 安全状态变量 ----------
        self.start_requested = False
        self.reset_required = False
        self.emergency_reason = ""
        self.land_reason = ""
        self.disarm_after_land = False

        self.last_mode_req = rospy.Time.now()
        self.last_land_req = rospy.Time.now()

        # ---------- 视觉结果缓存 ----------
        self.qr_text = ""
        self.qr_class_1 = ""
        self.qr_class_2 = ""
        self.qr_land_side = ""

        # ---------- 图片靶识别 / 投放状态 ----------
        self.current_image_class = ""
        self.target_class_1_done = False
        self.target_class_2_done = False
        self.image_drop_count = 0

        # ---------- ROS 发布 ----------
        self.raw_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=20
        )

        self.fsm_state_pub = rospy.Publisher(
            "/uav/fsm_state",
            String,
            queue_size=10
        )

        self.safety_state_pub = rospy.Publisher(
            "/uav/safety_state",
            String,
            queue_size=10,
            latch=True
        )

        self.land_status_pub = rospy.Publisher(
            "/uav/land_status",
            String,
            queue_size=10,
            latch=True
        )

        self.scan_enable_pub = rospy.Publisher(
            "/uav/scan_enable",
            Bool,
            queue_size=10
        )

        self.scan_target_pub = rospy.Publisher(
            "/uav/scan_target",
            String,
            queue_size=10
        )

        self.drop_cmd_pub = rospy.Publisher(
            "/uav/drop_cmd",
            String,
            queue_size=10
        )

        # ---------- ROS 订阅 ----------
        self.state_sub = rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        self.extended_state_sub = rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self.extended_state_cb,
            queue_size=10
        )

        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self.pose_cb,
            queue_size=10
        )

        self.vel_sub = rospy.Subscriber(
            "/mavros/local_position/velocity_local",
            TwistStamped,
            self.vel_cb,
            queue_size=10
        )

        self.qr_sub = rospy.Subscriber(
            "/uav/qr_text",
            String,
            self.qr_text_cb,
            queue_size=10
        )

        self.image_sub = rospy.Subscriber(
            "/uav/image_class",
            String,
            self.image_class_cb,
            queue_size=10
        )

        self.start_sub = rospy.Subscriber(
            "/uav/start",
            Bool,
            self.start_cb,
            queue_size=5
        )

        self.stop_sub = rospy.Subscriber(
            "/uav/stop",
            Bool,
            self.stop_cb,
            queue_size=5
        )

        self.land_sub = rospy.Subscriber(
            "/uav/land",
            Bool,
            self.land_cb,
            queue_size=5
        )

        self.disarm_sub = rospy.Subscriber(
            "/uav/disarm",
            Bool,
            self.disarm_cb,
            queue_size=5
        )

        self.reset_sub = rospy.Subscriber(
            "/uav/reset",
            Bool,
            self.reset_cb,
            queue_size=5
        )

        # ---------- MAVROS 服务 ----------
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")

        self.arming_client = rospy.ServiceProxy(
            "/mavros/cmd/arming",
            CommandBool
        )

        self.set_mode_client = rospy.ServiceProxy(
            "/mavros/set_mode",
            SetMode
        )

        self.publish_land_status("IDLE")
        self.publish_safety_state("IDLE")

        rospy.loginfo("micro_uav_stage1_fsm safety version started.")
        rospy.loginfo("Loaded %d waypoints from %s", len(self.waypoints), self.mission_yaml)

    # =========================
    # 04. YAML 读取区
    # =========================

    def load_yaml(self, yaml_path):
        """读取 YAML 任务文件。"""
        yaml_path = os.path.expanduser(yaml_path)

        if not os.path.exists(yaml_path):
            raise RuntimeError("Mission YAML not found: %s" % yaml_path)

        with open(yaml_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        if cfg is None or "mission" not in cfg:
            raise RuntimeError("Invalid mission YAML: missing mission field.")

        return cfg

    def parse_waypoints(self, cfg):
        """从 YAML 中解析航点列表。"""
        result = []
        raw_points = cfg["mission"].get("waypoints", [])

        for item in raw_points:
            wp = Waypoint(
                name=str(item.get("name", "UNKNOWN")),
                kind=str(item.get("kind", "route")),
                action=str(item.get("action", "none")),
                x=float(item.get("x", 0.0)),
                y=float(item.get("y", 0.0)),
                z=float(item.get("z", 1.30)),
                yaw_deg=float(item.get("yaw_deg", 0.0)),
                speed=float(item.get("speed", 0.4)),
                acc=float(item.get("acc", 0.3)),
                hold_time=float(item.get("hold_time", 0.5)),
                control_mode=str(item.get("control_mode", "auto")),
                dynamic_land=bool(item.get("dynamic_land", False))
            )

            if wp.kind not in ["route", "scan"]:
                raise RuntimeError("Waypoint %s has invalid kind: %s" % (wp.name, wp.kind))

            # 2026-05-18 修改：默认 route 采用位置控制，scan 采用位置 + 速度融合控制。
            if wp.control_mode == "auto":
                if wp.kind == "route":
                    wp.control_mode = "position"
                else:
                    wp.control_mode = "fusion"

            if wp.control_mode not in ["position", "fusion", "fusion_route"]:
                raise RuntimeError(
                    "Waypoint %s has invalid control_mode: %s" %
                    (wp.name, wp.control_mode)
                )

            result.append(wp)

        if len(result) == 0:
            raise RuntimeError("No waypoint found in mission YAML.")

        return result

    # =========================
    # 05. ROS 回调函数区
    # =========================

    def state_cb(self, msg):
        """更新飞控连接、模式和解锁状态，并检测正常任务中的异常退出。"""
        prev_armed = self.current_state.armed
        prev_mode = self.current_state.mode
        self.current_state = msg

        active_task = (
            self.fsm_state in ["WAIT_FCU", "TAKEOFF", "MISSION", "STAGE1_DONE"] and
            self.start_requested and
            not self.reset_required
        )

        # 飞行中突然上锁，禁止后续自动继续任务，避免再次解锁后冲向旧航点。
        if active_task and prev_armed and not msg.armed:
            self.force_wait_reset("armed changed True -> False during active task")
            return

        # 正常任务中突然退出 OFFBOARD，视作控制链路异常，直接急停降落。
        if active_task and prev_mode == "OFFBOARD" and msg.mode not in ["OFFBOARD", "AUTO.LAND"]:
            if msg.armed:
                self.request_emergency_land("mode changed OFFBOARD -> %s during active task" % msg.mode)
            else:
                self.force_wait_reset("mode changed OFFBOARD -> %s and vehicle is disarmed" % msg.mode)
            return

        # 如果外部已经切到 AUTO.LAND，就顺势进入本 FSM 的降落流程，不再回原航点。
        if active_task and msg.mode == "AUTO.LAND":
            self.request_normal_land("external AUTO.LAND detected")

    def extended_state_cb(self, msg):
        """更新 MAVROS 扩展状态，用 landed_state 判断是否落地。"""
        self.extended_state = msg
        self.extended_state_ok = True

    def pose_cb(self, msg):
        """更新当前位置和当前 yaw。"""
        self.current_pose = msg

        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.current_yaw = yaw

        if not self.home_ready:
            self.home_x = msg.pose.position.x
            self.home_y = msg.pose.position.y
            self.home_z = msg.pose.position.z
            self.home_yaw = yaw
            self.home_ready = True

            rospy.loginfo(
                "Home set: x=%.2f y=%.2f z=%.2f yaw=%.1f deg",
                self.home_x,
                self.home_y,
                self.home_z,
                math.degrees(self.home_yaw)
            )

    def vel_cb(self, msg):
        """更新当前速度，并通过速度差分估算加速度。"""
        now = msg.header.stamp

        if now.to_sec() <= 0.0:
            now = rospy.Time.now()

        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z

        self.current_vel = [vx, vy, vz]
        self.current_speed = norm3(vx, vy, vz)

        if self.last_vel is not None and self.last_vel_time is not None:
            dt = (now - self.last_vel_time).to_sec()

            if dt > 1e-3:
                ax = (vx - self.last_vel[0]) / dt
                ay = (vy - self.last_vel[1]) / dt
                az = (vz - self.last_vel[2]) / dt
                self.current_acc_norm = norm3(ax, ay, az)

        self.last_vel = [vx, vy, vz]
        self.last_vel_time = now

    def qr_text_cb(self, msg):
        """接收二维码识别结果，格式示例：man,apple,left。"""
        self.qr_text = msg.data.strip()
        parts = [p.strip() for p in self.qr_text.split(",")]

        if len(parts) == 3:
            self.qr_class_1 = parts[0]
            self.qr_class_2 = parts[1]
            self.qr_land_side = parts[2].lower()

            rospy.loginfo(
                "QR parsed: class1=%s class2=%s land=%s",
                self.qr_class_1,
                self.qr_class_2,
                self.qr_land_side
            )

    def image_class_cb(self, msg):
        """接收当前图片靶类别，视觉节点建议发布 CIFAR-100 英文类别名。"""
        self.current_image_class = msg.data.strip()

    def start_cb(self, msg):
        """收到 /uav/start=True 后，才允许进入 WAIT_FCU、起飞和任务。"""
        if not msg.data:
            return

        if self.reset_required or self.fsm_state == "WAIT_RESET":
            rospy.logerr("Start rejected: reset is required before a new mission.")
            self.publish_safety_state("START_REJECTED_WAIT_RESET")
            return

        if self.fsm_state in ["LANDING", "EMERGENCY_LAND", "DISARMING"]:
            rospy.logerr("Start rejected: landing or disarming is in progress.")
            self.publish_safety_state("START_REJECTED_LANDING")
            return

        if self.fsm_state in ["TAKEOFF", "MISSION"]:
            rospy.logwarn("Start ignored: mission is already running.")
            return

        rospy.loginfo("/uav/start received. Mission start requested.")
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = True
        self.reset_required = False
        self.emergency_reason = ""
        self.land_reason = ""
        self.disarm_after_land = False
        self.publish_land_status("IDLE")
        self.publish_safety_state("START_REQUESTED")
        self.enter_fsm_state("WAIT_FCU")

    def stop_cb(self, msg):
        """收到 /uav/stop=True 后，立即取消任务并进入急停降落流程。"""
        if not msg.data:
            return

        self.request_emergency_land("/uav/stop received")

    def land_cb(self, msg):
        """收到 /uav/land=True 后，普通降落：取消任务、AUTO.LAND、落地后上锁。"""
        if not msg.data:
            return

        self.request_normal_land("/uav/land received")

    def disarm_cb(self, msg):
        """收到 /uav/disarm=True 后，若在空中则先降落，落地后再上锁。"""
        if not msg.data:
            return

        rospy.logwarn("/uav/disarm received. Safe disarm requested.")
        self.cancel_task_outputs()
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = False
        self.disarm_after_land = True

        if self.current_state.armed and not self.is_landed():
            self.land_reason = "/uav/disarm received while airborne"
            self.publish_safety_state("DISARM_WAIT_LAND")
            self.publish_land_status("DISARM_WAIT_LAND")
            self.enter_fsm_state("DISARMING")
        else:
            self.publish_safety_state("DISARM_DIRECT")
            self.publish_land_status("DISARM_REQUESTED")
            self.enter_fsm_state("DISARMING")

    def reset_cb(self, msg):
        """收到 /uav/reset=True 后，只有无人机已上锁才允许回到 WAIT_START。"""
        if not msg.data:
            return

        if self.current_state.armed:
            rospy.logerr("Reset rejected: vehicle is still armed. Land and disarm first.")
            self.publish_safety_state("RESET_REJECTED_ARMED")
            self.publish_land_status("RESET_REJECTED_ARMED")
            return

        rospy.logwarn("/uav/reset accepted. FSM returns to WAIT_START.")
        self.cancel_task_outputs()
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = False
        self.reset_required = False
        self.emergency_reason = ""
        self.land_reason = ""
        self.disarm_after_land = False
        self.publish_safety_state("IDLE")
        self.publish_land_status("IDLE")
        self.enter_fsm_state("WAIT_START")

    # =========================
    # 06. MAVROS 指令发布区
    # =========================

    def make_target_msg(self):
        """生成 PositionTarget 基础消息。"""
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        return msg

    def publish_velocity_yaw(self, vx, vy, vz, yaw):
        """
        发布速度 + yaw 指令。
        用于 route 点移动、原地转向、刹停、降落前清零旧 setpoint。
        """
        msg = self.make_target_msg()

        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        msg.yaw = yaw

        self.raw_pub.publish(msg)

    def publish_position_velocity_yaw(self, x, y, z, vx, vy, vz, yaw):
        """
        发布位置 + 速度 + yaw 融合指令。
        用于 scan/drop 点稳定停靠。
        """
        msg = self.make_target_msg()

        msg.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.yaw = yaw

        self.raw_pub.publish(msg)

    def publish_position_yaw(self, x, y, z, yaw):
        """
        发布纯位置 + yaw 指令。
        用于普通 route 航段：精度要求不高，交给 PX4 位置控制器快速飞到目标附近。
        """
        msg = self.make_target_msg()

        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw

        self.raw_pub.publish(msg)

    def publish_neutral_setpoint(self):
        """发布零速度指令，防止急停/降落前最后一帧旧目标继续推动飞机。"""
        self.publish_velocity_yaw(0.0, 0.0, 0.0, self.current_yaw)

    # =========================
    # 07. FSM 工具函数区
    # =========================

    def current_xyz(self):
        """读取当前无人机位置。"""
        p = self.current_pose.pose.position
        return p.x, p.y, p.z

    def get_selected_land_side(self):
        """根据二维码结果选择降落方向；没有真实二维码时使用 YAML 默认值。"""
        side = self.qr_land_side.lower().strip()

        if side not in ["left", "right"]:
            side = str(self.landing_cfg.get("default_side", "left")).lower()

        if side not in ["left", "right"]:
            side = "left"

        return side

    def get_dynamic_land_xy(self, wp):
        """动态降落点坐标：LAND_APPROACH 只替换 y，LAND_FINAL 替换 x/y。"""
        side = self.get_selected_land_side()
        side_cfg = self.landing_cfg.get(side, {})

        land_x = float(side_cfg.get("x", wp.x))
        land_y = float(side_cfg.get("y", wp.y))

        if wp.name.startswith("LAND_APPROACH"):
            approach_x = float(self.landing_cfg.get("approach_x", wp.x))
            return approach_x, land_y

        return land_x, land_y

    def get_abs_wp(self, wp):
        """将 YAML 中的相对航点转换成 MAVROS local 坐标。"""
        rel_x = wp.x
        rel_y = wp.y

        # 2026-05-18 修改：降落航点可根据二维码 left/right 动态选择上下 H 点。
        if wp.dynamic_land:
            rel_x, rel_y = self.get_dynamic_land_xy(wp)

        if self.frame_mode == "relative_home":
            return (
                self.home_x + rel_x,
                self.home_y + rel_y,
                self.home_z + wp.z,
                wp.yaw_rad()
            )

        return rel_x, rel_y, wp.z, wp.yaw_rad()

    def enter_fsm_state(self, new_state):
        """切换 FSM 大状态。"""
        self.fsm_state = new_state
        self.phase_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.action_start_time = None
        self.action_sent = False
        self.cmd_vel = [0.0, 0.0, 0.0]

        rospy.loginfo("FSM -> %s", new_state)

    def enter_nav_phase(self, new_phase):
        """切换单个航点内部导航阶段。"""
        self.nav_phase = new_phase
        self.phase_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.cmd_vel = [0.0, 0.0, 0.0]

        rospy.loginfo("NAV_PHASE -> %s", new_phase)

    def next_waypoint(self):
        """进入下一个航点。"""
        if self.wp_index < len(self.waypoints):
            rospy.loginfo("Waypoint done: %s", self.waypoints[self.wp_index].name)

        self.wp_index += 1
        self.enter_nav_phase("INIT")

        if self.wp_index >= len(self.waypoints):
            self.enter_fsm_state("STAGE1_DONE")

    def clear_task_runtime(self, reset_wp):
        """清理任务运行时状态，防止旧航点、旧速度和旧动作残留。"""
        if reset_wp:
            self.wp_index = 0
            self.qr_text = ""
            self.qr_class_1 = ""
            self.qr_class_2 = ""
            self.qr_land_side = ""
            self.current_image_class = ""
            self.target_class_1_done = False
            self.target_class_2_done = False
            self.image_drop_count = 0

        self.nav_phase = "INIT"
        self.locked_yaw = self.current_yaw
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.phase_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.action_start_time = None
        self.action_sent = False

    def cancel_task_outputs(self):
        """关闭扫描输出，并清除本 FSM 的动作状态。"""
        self.scan_enable_pub.publish(Bool(data=False))
        self.scan_target_pub.publish(String(data="none"))
        self.action_sent = False
        self.action_start_time = None

    def is_yaw_aligned(self, target_yaw):
        """判断 yaw 是否对齐。"""
        yaw_err = abs(wrap_pi(target_yaw - self.current_yaw))
        return yaw_err < self.yaw_eps

    def is_speed_acc_stable(self, vel_th, acc_th):
        """判断当前速度和加速度是否满足稳定条件。"""
        return self.current_speed < vel_th and self.current_acc_norm < acc_th

    def publish_fsm_state(self):
        """发布当前 FSM 状态，便于地面站显示。"""
        if self.wp_index < len(self.waypoints):
            wp_name = self.waypoints[self.wp_index].name
        else:
            wp_name = "NONE"

        msg = String()
        msg.data = "%s | safety=%s | land=%s | phase=%s | wp=%s" % (
            self.fsm_state,
            self.safety_state,
            self.land_status,
            self.nav_phase,
            wp_name
        )

        self.fsm_state_pub.publish(msg)

    def publish_safety_state(self, status):
        """发布安全状态。"""
        self.safety_state = status
        self.safety_state_pub.publish(String(data=status))

    def publish_land_status(self, status):
        """发布降落/上锁状态，状态变化时打印日志。"""
        self.land_status = status
        self.land_status_pub.publish(String(data=status))

        if status != self.last_land_status:
            rospy.logwarn("Landing status: %s", status)
            self.last_land_status = status

    def is_landed(self):
        """判断 PX4 是否已经确认落地。"""
        return (
            self.extended_state_ok and
            self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )

    def force_wait_reset(self, reason):
        """进入 WAIT_RESET，禁止自动恢复旧任务。"""
        rospy.logerr("WAIT_RESET forced: %s", reason)
        self.cancel_task_outputs()
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = False
        self.reset_required = True
        self.emergency_reason = reason
        self.disarm_after_land = False
        self.publish_safety_state("WAIT_RESET_FORCED")
        self.publish_land_status("WAIT_RESET")
        self.enter_fsm_state("WAIT_RESET")

    def request_emergency_land(self, reason):
        """急停降落：取消任务、清除目标、请求 AUTO.LAND，落地后自动上锁。"""
        if self.fsm_state in ["EMERGENCY_LAND", "DISARMING"]:
            return

        rospy.logerr("EMERGENCY LAND requested: %s", reason)
        self.cancel_task_outputs()
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = False
        self.reset_required = True
        self.emergency_reason = reason
        self.disarm_after_land = True
        self.publish_safety_state("EMERGENCY_LAND")
        self.publish_land_status("EMERGENCY_LAND_REQUESTED")
        self.enter_fsm_state("EMERGENCY_LAND")

    def request_normal_land(self, reason):
        """普通降落：取消任务、请求 AUTO.LAND，落地后自动上锁。"""
        if self.fsm_state in ["LANDING", "EMERGENCY_LAND", "DISARMING"]:
            return

        rospy.logwarn("Normal landing requested: %s", reason)
        self.cancel_task_outputs()
        self.clear_task_runtime(reset_wp=True)
        self.start_requested = False
        self.reset_required = True
        self.land_reason = reason
        self.disarm_after_land = True
        self.publish_safety_state("LANDING")
        self.publish_land_status("LAND_REQUESTED")
        self.enter_fsm_state("LANDING")

    # =========================
    # 08. OFFBOARD / LAND / DISARM 辅助区
    # =========================

    def try_set_offboard_and_arm(self):
        """根据参数决定是否自动切 OFFBOARD 和自动解锁。"""
        if not self.start_requested or self.reset_required:
            return

        if self.fsm_state not in ["WAIT_FCU", "TAKEOFF", "MISSION"]:
            return

        now = rospy.Time.now()

        if (now - self.last_mode_req).to_sec() < 2.0:
            return

        self.last_mode_req = now

        if self.auto_set_mode and self.current_state.mode != "OFFBOARD":
            try:
                response = self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                if hasattr(response, "mode_sent") and not response.mode_sent:
                    rospy.logwarn("OFFBOARD request rejected.")
                else:
                    rospy.loginfo("Request OFFBOARD mode.")
            except rospy.ServiceException as e:
                rospy.logwarn("Set OFFBOARD failed: %s", str(e))

        if self.auto_arm and not self.current_state.armed:
            try:
                response = self.arming_client(True)
                if hasattr(response, "success") and not response.success:
                    rospy.logwarn("Arm request rejected.")
                else:
                    rospy.loginfo("Request arm.")
            except rospy.ServiceException as e:
                rospy.logwarn("Arm failed: %s", str(e))

    def try_set_auto_land(self):
        """请求 PX4 进入 AUTO.LAND 模式。"""
        if not self.current_state.armed:
            self.publish_land_status("DISARMED")
            self.enter_fsm_state("WAIT_RESET")
            return

        if self.current_state.mode == "AUTO.LAND":
            self.publish_land_status("WAIT_LANDED")
            return

        now = rospy.Time.now()

        if (now - self.last_land_req).to_sec() < 1.0:
            return

        try:
            response = self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")

            if hasattr(response, "mode_sent") and response.mode_sent:
                self.publish_land_status("AUTO_LAND_SENT")
                rospy.logwarn("AUTO.LAND mode requested.")
            else:
                self.publish_land_status("AUTO_LAND_REJECTED")
                rospy.logwarn("AUTO.LAND request rejected, will retry.")
        except rospy.ServiceException as e:
            self.publish_land_status("AUTO_LAND_FAILED")
            rospy.logwarn("Set AUTO.LAND failed: %s", str(e))

        self.last_land_req = now

    def try_disarm_after_landed(self):
        """等待 PX4 确认落地后，再调用 /mavros/cmd/arming false。"""
        if not self.current_state.armed:
            self.publish_land_status("DISARMED")
            self.reset_required = True
            self.start_requested = False
            self.disarm_after_land = False
            self.enter_fsm_state("WAIT_RESET")
            return

        if not self.is_landed():
            if not self.extended_state_ok:
                self.publish_land_status("WAIT_EXTENDED_STATE")
            else:
                self.publish_land_status("WAIT_LANDED")
            return

        now = rospy.Time.now()

        if (now - self.last_land_req).to_sec() < 1.0:
            return

        try:
            response = self.arming_client(False)

            if hasattr(response, "success") and response.success:
                self.publish_land_status("DISARMED")
                self.reset_required = True
                self.start_requested = False
                self.disarm_after_land = False
                self.enter_fsm_state("WAIT_RESET")
                rospy.logwarn("Disarm requested after PX4 confirmed landed.")
            else:
                self.publish_land_status("DISARM_DENIED")
                rospy.logwarn("Disarm denied, will retry after landed check.")
        except rospy.ServiceException as e:
            self.publish_land_status("DISARM_FAILED")
            rospy.logwarn("Disarm failed: %s", str(e))

        self.last_land_req = now

    def handle_landing(self):
        """统一处理 LANDING / EMERGENCY_LAND / DISARMING。"""
        self.cancel_task_outputs()
        self.publish_neutral_setpoint()

        if not self.current_state.armed:
            self.publish_land_status("DISARMED")
            self.reset_required = True
            self.enter_fsm_state("WAIT_RESET")
            return

        if not self.is_landed():
            self.try_set_auto_land()
            return

        self.publish_land_status("LANDED")
        self.try_disarm_after_landed()

    # =========================
    # 09. 起飞控制区
    # =========================

    def process_takeoff(self):
        """起飞到指定高度，并确认停稳。"""
        target_x = self.home_x
        target_y = self.home_y
        target_z = self.home_z + self.takeoff_height
        target_yaw = math.radians(self.takeoff_yaw_deg)

        self.publish_position_velocity_yaw(
            target_x,
            target_y,
            target_z,
            0.0,
            0.0,
            0.0,
            target_yaw
        )

        cx, cy, cz = self.current_xyz()

        pos_err = norm3(
            target_x - cx,
            target_y - cy,
            target_z - cz
        )

        stable = (
            pos_err < self.scan_pos_eps and
            self.current_speed < self.scan_stable_vel and
            self.current_acc_norm < self.scan_stable_acc
        )

        if stable:
            if self.stable_start_time is None:
                self.stable_start_time = rospy.Time.now()

            stable_time = (rospy.Time.now() - self.stable_start_time).to_sec()

            if stable_time > self.takeoff_stable_time:
                self.enter_fsm_state("MISSION")
                self.enter_nav_phase("INIT")
        else:
            self.stable_start_time = None

        rospy.loginfo_throttle(
            0.5,
            "[TAKEOFF] err=%.2f vel=%.2f acc=%.2f stable=%s",
            pos_err,
            self.current_speed,
            self.current_acc_norm,
            str(stable)
        )

    # =========================
    # 10. 航点导航控制区
    # =========================

    def process_current_waypoint(self, dt):
        """处理当前航点，包括 yaw 对齐、移动、刹停、停稳、动作。"""
        if self.wp_index >= len(self.waypoints):
            self.enter_fsm_state("STAGE1_DONE")
            return

        wp = self.waypoints[self.wp_index]
        tx, ty, tz, target_yaw = self.get_abs_wp(wp)
        cx, cy, cz = self.current_xyz()

        dx = tx - cx
        dy = ty - cy
        dz = tz - cz
        dist = norm3(dx, dy, dz)

        if self.nav_phase == "INIT":
            # 普通位置控制 / 融合快速通过航段，默认先把机头转向运动方向；
            # 扫码、投放、穿环等待等精细点，使用 YAML 指定的 yaw。
            if wp.control_mode in ["position", "fusion_route"] and dist > 0.10:
                self.locked_yaw = math.atan2(dy, dx)
            else:
                self.locked_yaw = target_yaw

            self.enter_nav_phase("YAW_ALIGN")
            return

        if self.nav_phase == "YAW_ALIGN":
            self.publish_velocity_yaw(0.0, 0.0, 0.0, self.locked_yaw)

            yaw_ready = self.is_yaw_aligned(self.locked_yaw)
            stable_ready = self.is_speed_acc_stable(
                self.route_finish_vel,
                self.route_finish_acc
            )

            if yaw_ready and stable_ready:
                self.enter_nav_phase("MOVE")

            rospy.loginfo_throttle(
                0.5,
                "[YAW_ALIGN] wp=%s yaw_err=%.1f deg vel=%.2f acc=%.2f mode=%s",
                wp.name,
                math.degrees(abs(wrap_pi(self.locked_yaw - self.current_yaw))),
                self.current_speed,
                self.current_acc_norm,
                wp.control_mode
            )

            return

        if self.nav_phase == "MOVE":
            yaw_err = abs(wrap_pi(self.locked_yaw - self.current_yaw))

            if yaw_err > self.yaw_break_eps:
                self.enter_nav_phase("YAW_ALIGN")
                return

            if wp.control_mode == "position":
                self.process_route_move(wp, tx, ty, tz, dist)
                return

            if wp.control_mode == "fusion_route":
                self.process_fusion_route_move(wp, tx, ty, tz, dist, dx, dy, dz, dt)
                return

            if wp.control_mode == "fusion":
                self.process_scan_move(wp, tx, ty, tz, dist, dx, dy, dz, dt)
                return

        if self.nav_phase == "BRAKE":
            self.publish_velocity_yaw(0.0, 0.0, 0.0, self.locked_yaw)

            stable_ready = self.is_speed_acc_stable(
                self.route_finish_vel,
                self.route_finish_acc
            )

            if stable_ready:
                self.next_waypoint()

            rospy.loginfo_throttle(
                0.5,
                "[BRAKE] wp=%s vel=%.2f acc=%.2f",
                wp.name,
                self.current_speed,
                self.current_acc_norm
            )

            return

        if self.nav_phase == "HOLD":
            self.publish_position_velocity_yaw(
                tx,
                ty,
                tz,
                0.0,
                0.0,
                0.0,
                self.locked_yaw
            )

            hold_time = (rospy.Time.now() - self.phase_start_time).to_sec()

            if hold_time > wp.hold_time:
                self.enter_nav_phase("ACTION")

            return

        if self.nav_phase == "ACTION":
            self.process_action(wp, tx, ty, tz)
            return

    def process_route_move(self, wp, tx, ty, tz, dist):
        """route 点纯位置控制移动：到达目标附近后直接切下一个点。"""
        self.publish_position_yaw(tx, ty, tz, self.locked_yaw)

        if dist < self.route_pos_eps:
            if wp.action == "none":
                self.next_waypoint()
            else:
                self.enter_nav_phase("HOLD")

            return

        rospy.loginfo_throttle(
            0.5,
            "[ROUTE_POSITION] wp=%s dist=%.2f real_v=%.2f acc=%.2f",
            wp.name,
            dist,
            self.current_speed,
            self.current_acc_norm
        )

    def process_fusion_route_move(self, wp, tx, ty, tz, pos_err, dx, dy, dz, dt):
        """复杂通过点的位置 + 速度融合控制，例如穿环前后直线通过。"""
        vx = self.scan_kp * dx
        vy = self.scan_kp * dy
        vz = self.scan_kp * dz

        vx, vy, vz = limit_vector_norm(vx, vy, vz, wp.speed)

        max_delta = wp.acc * dt

        self.cmd_vel = limit_vector_change(
            self.cmd_vel,
            [vx, vy, vz],
            max_delta
        )

        self.publish_position_velocity_yaw(
            tx,
            ty,
            tz,
            self.cmd_vel[0],
            self.cmd_vel[1],
            self.cmd_vel[2],
            self.locked_yaw
        )

        if pos_err < self.route_pos_eps:
            if wp.action == "none":
                self.next_waypoint()
            else:
                self.enter_nav_phase("HOLD")

            return

        rospy.loginfo_throttle(
            0.5,
            "[FUSION_ROUTE] wp=%s err=%.2f cmd_v=%.2f real_v=%.2f",
            wp.name,
            pos_err,
            norm3(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2]),
            self.current_speed
        )

    def process_scan_move(self, wp, tx, ty, tz, pos_err, dx, dy, dz, dt):
        """scan 点位置 + 速度融合控制，并检查停稳条件。"""
        vx = self.scan_kp * dx
        vy = self.scan_kp * dy
        vz = self.scan_kp * dz

        vx, vy, vz = limit_vector_norm(vx, vy, vz, wp.speed)

        max_delta = wp.acc * dt

        self.cmd_vel = limit_vector_change(
            self.cmd_vel,
            [vx, vy, vz],
            max_delta
        )

        self.publish_position_velocity_yaw(
            tx,
            ty,
            tz,
            self.cmd_vel[0],
            self.cmd_vel[1],
            self.cmd_vel[2],
            self.locked_yaw
        )

        stable = (
            pos_err < self.scan_pos_eps and
            self.current_speed < self.scan_stable_vel and
            self.current_acc_norm < self.scan_stable_acc
        )

        if stable:
            if self.stable_start_time is None:
                self.stable_start_time = rospy.Time.now()

            stable_time = (rospy.Time.now() - self.stable_start_time).to_sec()

            if stable_time > self.scan_stable_time:
                self.enter_nav_phase("HOLD")
        else:
            self.stable_start_time = None

        rospy.loginfo_throttle(
            0.5,
            "[SCAN_MOVE] wp=%s err=%.2f cmd_v=%.2f real_v=%.2f acc=%.2f stable=%s",
            wp.name,
            pos_err,
            norm3(self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2]),
            self.current_speed,
            self.current_acc_norm,
            str(stable)
        )

    # =========================
    # 11. 动作执行区
    # =========================

    def process_action(self, wp, tx, ty, tz):
        """
        到达 scan 点后的动作执行。
        当前版本只提供接口，不强行绑定具体视觉算法和投放机构。
        """
        self.publish_position_velocity_yaw(
            tx,
            ty,
            tz,
            0.0,
            0.0,
            0.0,
            self.locked_yaw
        )

        if self.action_start_time is None:
            self.action_start_time = rospy.Time.now()
            self.action_sent = False

            if wp.action == "image_scan_maybe_drop":
                self.current_image_class = ""

        action_elapsed = (rospy.Time.now() - self.action_start_time).to_sec()

        if wp.action == "none":
            self.next_waypoint()
            return

        if wp.action == "qr_scan":
            self.do_qr_scan_action(wp, action_elapsed)
            return

        if wp.action == "image_drop_1":
            self.do_image_drop_action(wp, "image_drop_1", action_elapsed)
            return

        if wp.action == "image_drop_2":
            self.do_image_drop_action(wp, "image_drop_2", action_elapsed)
            return

        if wp.action == "image_scan_maybe_drop":
            self.do_image_scan_maybe_drop_action(wp, action_elapsed)
            return

        if wp.action == "special_drop":
            self.do_drop_only_action(wp, "special_drop", action_elapsed)
            return

        if wp.action == "ring_wait":
            self.do_ring_wait_action(wp, action_elapsed)
            return

        if wp.action == "mission_done":
            self.do_mission_done_action(wp, action_elapsed)
            return

        rospy.logwarn("Unknown action %s at waypoint %s, skip.", wp.action, wp.name)
        self.next_waypoint()

    def do_qr_scan_action(self, wp, elapsed):
        """二维码扫描动作接口。"""
        self.scan_enable_pub.publish(Bool(data=True))
        self.scan_target_pub.publish(String(data="qr"))

        if self.wait_real_qr:
            qr_ok = self.qr_class_1 != "" and self.qr_class_2 != "" and self.qr_land_side != ""

            if qr_ok:
                rospy.loginfo("QR scan success: %s", self.qr_text)
                self.scan_enable_pub.publish(Bool(data=False))
                self.next_waypoint()
                return

            if elapsed > self.qr_scan_timeout:
                rospy.logwarn("QR scan timeout, continue with empty QR result.")
                self.scan_enable_pub.publish(Bool(data=False))
                self.next_waypoint()
                return
        else:
            if elapsed > self.qr_scan_timeout:
                rospy.loginfo("QR scan placeholder done.")
                self.scan_enable_pub.publish(Bool(data=False))
                self.next_waypoint()
                return

        rospy.loginfo_throttle(
            0.5,
            "[ACTION_QR] wp=%s elapsed=%.1f qr=%s",
            wp.name,
            elapsed,
            self.qr_text
        )

    def do_image_drop_action(self, wp, drop_name, elapsed):
        """图片靶识别 + 投放动作接口。"""
        self.scan_enable_pub.publish(Bool(data=True))
        self.scan_target_pub.publish(String(data="image_target"))

        if not self.action_sent and elapsed > self.image_scan_time:
            cmd = String()
            cmd.data = drop_name
            self.drop_cmd_pub.publish(cmd)

            self.action_sent = True
            rospy.loginfo("Drop command sent: %s", drop_name)

        if elapsed > self.image_scan_time + self.drop_time + self.hold_after_action:
            self.scan_enable_pub.publish(Bool(data=False))
            self.next_waypoint()
            return

        rospy.loginfo_throttle(
            0.5,
            "[ACTION_IMAGE_DROP] wp=%s cmd=%s elapsed=%.1f sent=%s",
            wp.name,
            drop_name,
            elapsed,
            str(self.action_sent)
        )

    def do_image_scan_maybe_drop_action(self, wp, elapsed):
        """
        图片靶扫描 + 按需投放接口。
        wait_real_image=false 时：为了调航线，默认在前两个图片靶执行两次投放。
        wait_real_image=true 时：只有 /uav/image_class 与二维码两个类别之一匹配时才投放。
        """
        self.scan_enable_pub.publish(Bool(data=True))
        self.scan_target_pub.publish(String(data="image_target"))

        drop_cmd = ""

        if elapsed > self.image_scan_time:
            if self.wait_real_image:
                img_class = self.current_image_class.strip()

                if img_class == self.qr_class_1 and not self.target_class_1_done:
                    drop_cmd = "image_drop_1"

                elif img_class == self.qr_class_2 and not self.target_class_2_done:
                    drop_cmd = "image_drop_2"
            else:
                # 调航线占位逻辑：不接视觉时，前两个图片靶各投一次，保证流程能跑通。
                if self.image_drop_count == 0:
                    drop_cmd = "image_drop_1"

                elif self.image_drop_count == 1:
                    drop_cmd = "image_drop_2"

        if drop_cmd != "" and not self.action_sent:
            self.drop_cmd_pub.publish(String(data=drop_cmd))
            self.action_sent = True
            self.image_drop_count += 1

            if drop_cmd == "image_drop_1":
                self.target_class_1_done = True

            if drop_cmd == "image_drop_2":
                self.target_class_2_done = True

            rospy.loginfo(
                "Image target matched, drop command sent: %s, image_class=%s",
                drop_cmd,
                self.current_image_class
            )

        if self.action_sent:
            if elapsed > self.image_scan_time + self.drop_time + self.hold_after_action:
                self.scan_enable_pub.publish(Bool(data=False))
                self.next_waypoint()
                return
        else:
            if elapsed > self.image_scan_time + self.hold_after_action:
                self.scan_enable_pub.publish(Bool(data=False))
                self.next_waypoint()
                return

        rospy.loginfo_throttle(
            0.5,
            "[ACTION_IMAGE_SCAN_MAYBE_DROP] wp=%s class=%s qr=(%s,%s) drop_count=%d sent=%s",
            wp.name,
            self.current_image_class,
            self.qr_class_1,
            self.qr_class_2,
            self.image_drop_count,
            str(self.action_sent)
        )

    def do_drop_only_action(self, wp, drop_name, elapsed):
        """特殊靶投放动作接口。"""
        self.scan_enable_pub.publish(Bool(data=True))
        self.scan_target_pub.publish(String(data="special_target"))

        if not self.action_sent:
            cmd = String()
            cmd.data = drop_name
            self.drop_cmd_pub.publish(cmd)

            self.action_sent = True
            rospy.loginfo("Drop command sent: %s", drop_name)

        if elapsed > self.drop_time + self.hold_after_action:
            self.scan_enable_pub.publish(Bool(data=False))
            self.next_waypoint()
            return

        rospy.loginfo_throttle(
            0.5,
            "[ACTION_SPECIAL_DROP] wp=%s elapsed=%.1f",
            wp.name,
            elapsed
        )

    def do_mission_done_action(self, wp, elapsed):
        """完整航线结束后保持在降落点上方，等待 /uav/land 或人工确认。"""
        self.scan_enable_pub.publish(Bool(data=False))

        if elapsed > wp.hold_time:
            rospy.loginfo("Mission route finished. Holding above landing point.")
            self.enter_fsm_state("STAGE1_DONE")
            return

        rospy.loginfo_throttle(
            0.5,
            "[MISSION_DONE] wp=%s elapsed=%.1f land_side=%s",
            wp.name,
            elapsed,
            self.get_selected_land_side()
        )

    def do_ring_wait_action(self, wp, elapsed):
        """
        随机圆环前等待。
        第一阶段到这里结束，不直接穿环。
        """
        self.scan_enable_pub.publish(Bool(data=False))

        if elapsed > wp.hold_time:
            rospy.loginfo("Arrived fixed ring waiting point. Stage1 done.")
            self.enter_fsm_state("STAGE1_DONE")
            return

        rospy.loginfo_throttle(
            0.5,
            "[RING_WAIT] wp=%s elapsed=%.1f",
            wp.name,
            elapsed
        )

    # =========================
    # 12. 主循环区
    # =========================

    def spin(self):
        """FSM 主循环。"""
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now
            dt = clamp(dt, 1.0 / 100.0, 1.0 / 10.0)

            self.publish_fsm_state()

            # 安全状态优先级最高：急停、普通降落、上锁流程永远先于任务控制。
            if self.fsm_state in ["EMERGENCY_LAND", "LANDING", "DISARMING"]:
                self.handle_landing()
                rate.sleep()
                continue

            if self.fsm_state == "WAIT_RESET":
                self.cancel_task_outputs()
                self.publish_neutral_setpoint()
                rospy.logwarn_throttle(2.0, "WAIT_RESET: publish /uav/reset=True after vehicle is disarmed.")
                rate.sleep()
                continue

            # 等待定位数据时不进入任务；但如果已经收到 stop/land，前面的安全分支会优先处理。
            if self.current_pose is None or not self.home_ready:
                rate.sleep()
                continue

            if self.fsm_state == "WAIT_START":
                self.cancel_task_outputs()
                self.publish_neutral_setpoint()
                rospy.loginfo_throttle(2.0, "WAIT_START: publish /uav/start=True to start mission.")

            elif self.fsm_state == "WAIT_FCU":
                self.try_set_offboard_and_arm()
                self.publish_position_velocity_yaw(
                    self.home_x,
                    self.home_y,
                    self.home_z + self.takeoff_height,
                    0.0,
                    0.0,
                    0.0,
                    math.radians(self.takeoff_yaw_deg)
                )

                if self.current_state.connected:
                    self.enter_fsm_state("TAKEOFF")

            elif self.fsm_state == "TAKEOFF":
                self.try_set_offboard_and_arm()
                self.process_takeoff()

            elif self.fsm_state == "MISSION":
                self.try_set_offboard_and_arm()
                self.process_current_waypoint(dt)

            elif self.fsm_state == "STAGE1_DONE":
                self.process_stage1_done()

            else:
                rospy.logwarn_throttle(1.0, "Unknown FSM state: %s", self.fsm_state)

            rate.sleep()

    def process_stage1_done(self):
        """第一阶段结束后保持在最后一个点，等待 /uav/land 或人工接管。"""
        if len(self.waypoints) == 0:
            self.publish_velocity_yaw(0.0, 0.0, 0.0, self.current_yaw)
            return

        last_wp = self.waypoints[-1]
        tx, ty, tz, yaw = self.get_abs_wp(last_wp)

        self.publish_position_velocity_yaw(
            tx,
            ty,
            tz,
            0.0,
            0.0,
            0.0,
            yaw
        )

        rospy.loginfo_throttle(
            2.0,
            "STAGE1_DONE: holding at %s. Publish /uav/land=True to land.",
            last_wp.name
        )


# =========================
# 13. 程序入口
# =========================

if __name__ == "__main__":
    try:
        node = MicroUAVStage1FSM()
        node.spin()
    except rospy.ROSInterruptException:
        pass
