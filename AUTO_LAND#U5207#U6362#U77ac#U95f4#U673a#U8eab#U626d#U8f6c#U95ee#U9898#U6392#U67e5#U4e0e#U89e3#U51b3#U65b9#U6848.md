# AUTO.LAND 切换瞬间机身扭转问题排查与解决方案

本文档针对当前 FSM 在降落时切换 `AUTO.LAND` 的瞬间，机身出现明显 yaw 扭转的问题，整理可能原因、排查方法和可实施解决方案。

相关代码位置：

- `scripts/micro_uav_stage1_fsm.py`
- `handle_landing()`
- `try_set_auto_land()`
- `publish_neutral_setpoint()`

## 1. 现象描述

当前降落流程大致是：

```text
/uav/land 或 /uav/stop
-> FSM 进入 LANDING / EMERGENCY_LAND
-> cancel_task_outputs()
-> publish_neutral_setpoint()
-> set_mode AUTO.LAND
-> PX4 自动降落
-> 落地后 disarm
```

其中 `publish_neutral_setpoint()` 当前发布的是：

```python
publish_velocity_yaw(0.0, 0.0, 0.0, current_yaw)
```

也就是在切模式前，FSM 还在通过 OFFBOARD setpoint 锁住当前 yaw。

但一旦切到 `AUTO.LAND`，yaw 控制权会从外部 OFFBOARD setpoint 交给 PX4 内部降落逻辑。这个交接瞬间如果 PX4 内部的目标 yaw、当前估计 yaw、磁力计 yaw、任务 yaw 或参数逻辑不一致，就可能出现机身突然扭一下。

## 2. 它是在对齐地磁场方向吗？

不一定，更准确地说：

```text
它通常不是“主动去对齐地磁场方向”，而是飞控在 AUTO.LAND 接管 yaw 控制后，
内部 yaw 目标或 yaw 估计发生了变化，导致控制器短时间输出了 yaw 转动。
```

地磁场可能参与了 yaw 估计，所以磁力计干扰、EKF yaw 估计跳变会放大这个现象。

但常见根因不止磁力计：

1. `OFFBOARD` 阶段 yaw setpoint 和 `AUTO.LAND` 阶段 PX4 内部 yaw setpoint 不一致。
2. 切模式时速度 setpoint 使用 `current_yaw`，而 PX4 接管后想保持另一个 yaw。
3. 当前 yaw 估计本身不稳定，尤其室内电磁环境下磁力计受干扰。
4. AUTO.LAND 参数或任务逻辑让 PX4 使用了“默认航向/航线航向/起飞航向”。
5. 切模式前飞机还没真正低速稳定，yaw 控制交接时姿态控制器反应明显。

## 3. 当前代码里的风险点

### 3.1 直接从任务状态切 AUTO.LAND

`request_normal_land()` 和 `request_emergency_land()` 进入降落状态后，主循环立刻调用：

```python
handle_landing()
```

而 `handle_landing()` 中会：

```python
publish_neutral_setpoint()
try_set_auto_land()
```

也就是说，当前没有一个明确的“切模式前稳定保持阶段”。

如果飞机刚到某个航点、刚结束视觉对准、刚从运动状态转入降落，姿态和速度还没完全稳住，切 `AUTO.LAND` 更容易出现瞬时扭转。

### 3.2 neutral setpoint 使用 `current_yaw`

当前：

```python
publish_neutral_setpoint()
```

使用当前实时 yaw：

```python
self.publish_velocity_yaw(0.0, 0.0, 0.0, self.current_yaw)
```

这个做法的优点是“不要继续推飞机向旧航点飞”。  
但它的缺点是：如果 `current_yaw` 在切换瞬间有小幅抖动，或者 PX4 接管后想保持的是上一个 setpoint yaw / locked_yaw，就会出现 yaw 目标不连续。

### 3.3 AUTO.LAND 后 FSM 不再真正控制 yaw

切到 `AUTO.LAND` 后，OFFBOARD setpoint 对 yaw 的控制权通常已经不再按原方式生效。  
所以即使 FSM 继续发布 `current_yaw`，PX4 降落模式内部也可能不使用这个 yaw。

## 4. 排查优先级

建议先做日志观察，不要先盲改参数。

### 4.1 记录切模式前后的 yaw

重点看：

```text
切 AUTO.LAND 前 current_yaw
切 AUTO.LAND 后 current_yaw
切 AUTO.LAND 后机身实际 yaw 是否跳变
```

可以临时在 `try_set_auto_land()` 里打印：

```python
rospy.logwarn(
    "Before AUTO.LAND: mode=%s yaw=%.1f deg speed=%.2f",
    self.current_state.mode,
    math.degrees(self.current_yaw),
    self.current_speed
)
```

如果切换前后 yaw 估计也跳了，说明可能是 EKF / 磁力计 / 定位融合问题。  
如果 yaw 估计连续，但机身实际转了，说明更像 yaw setpoint 接管问题。

### 4.2 看是否每次都朝同一个方向扭

| 现象 | 更可能原因 |
|---|---|
| 每次都向固定绝对方向扭 | 磁力计/航向参考/飞控内部 yaw 目标 |
| 每次都向任务最后一个 yaw 扭 | OFFBOARD yaw 和 AUTO.LAND yaw 交接不连续 |
| 只有速度较大时扭 | 切模式前没有稳定保持 |
| 室内某些位置更严重 | 电磁干扰或定位 yaw 估计不稳 |

### 4.3 对比手动切 AUTO.LAND

测试两种情况：

```text
1. FSM 调 /uav/land 切 AUTO.LAND
2. 人工在 QGC 或遥控器上切 AUTO.LAND
```

如果人工切也扭，问题大概率在 PX4 参数、磁力计、估计器或机体调参。  
如果只有 FSM 切时扭，问题更可能在 FSM 切换前的 setpoint 和时机。

## 5. 解决方案 A：切 AUTO.LAND 前增加稳定保持阶段

这是最推荐先做的代码侧方案。

思路：

```text
收到 land/stop 后，不要立刻 set_mode AUTO.LAND。
先进入 LAND_PREPARE 阶段，位置锁定或零速度保持 0.5~1.0 秒。
确认速度低、yaw 稳，再切 AUTO.LAND。
```

优点：

- 改动风险低。
- 能减少运动状态切模式导致的姿态突变。
- 对普通降落和急停降落都有效。

建议逻辑：

```text
LANDING_PREPARE:
  publish_position_velocity_yaw(current_x, current_y, current_z, 0,0,0, land_locked_yaw)
  等待 0.5~1.0s
  current_speed < 0.15~0.25 m/s
  yaw 误差稳定
  -> set AUTO.LAND
```

需要新增变量：

```python
self.land_prepare_start_time
self.land_hold_x
self.land_hold_y
self.land_hold_z
self.land_hold_yaw
self.land_prepare_time
self.land_prepare_vel_th
```

风险：

- 急停时如果你希望“立刻降落”，预保持时间不能太长。
- 可以普通 `/uav/land` 用 1 秒准备，急停 `/uav/stop` 用 0.2 秒或直接切。

## 6. 解决方案 B：切模式前锁定一个固定 yaw，不用实时 current_yaw

当前 neutral setpoint 用的是 `current_yaw`。可以改成：

```text
进入 LANDING 状态那一刻，把当前 yaw 存成 land_locked_yaw。
降落准备阶段和切模式前都使用 land_locked_yaw。
```

不要每帧把最新 `current_yaw` 写进 setpoint。

原因：

```text
current_yaw 是估计值，会随传感器融合轻微波动。
切模式前保持固定 yaw setpoint，能让 yaw 目标更连续。
```

建议：

```python
if self.land_locked_yaw is None:
    self.land_locked_yaw = self.current_yaw

publish_velocity_yaw(0.0, 0.0, 0.0, self.land_locked_yaw)
```

或者在普通降落前：

```python
publish_position_velocity_yaw(cx, cy, cz, 0, 0, 0, self.land_locked_yaw)
```

优点：

- 改动简单。
- 可以和方案 A 一起用。

限制：

- 如果 PX4 在 `AUTO.LAND` 后完全不用外部 yaw setpoint，仍可能扭，但一般会减轻交接瞬间的不连续。

## 7. 解决方案 C：不用 PX4 AUTO.LAND，改成 OFFBOARD 受控下降

如果 `AUTO.LAND` 的 yaw 接管就是不稳定，可以考虑不切 `AUTO.LAND`，而由 FSM 在 OFFBOARD 中自己降落。

流程：

```text
LANDING:
  保持 x/y/yaw
  z 逐渐下降
  接近地面后调用 disarm
```

控制方式：

```python
publish_position_velocity_yaw(
    land_x,
    land_y,
    target_z,
    0.0,
    0.0,
    -descent_speed,
    land_locked_yaw
)
```

优点：

- yaw 完全由 FSM 持续锁定，不会交给 AUTO.LAND 内部逻辑。
- 降落路径和速度更可控。

缺点：

- 需要可靠高度来源。
- 需要处理落地检测、地面效应、降落速度、误判落地等安全问题。
- 失去 PX4 AUTO.LAND 的一部分保护逻辑。

建议使用方式：

```text
普通任务完成后的降落可以用 OFFBOARD 受控下降。
急停仍保留 AUTO.LAND 作为安全兜底。
```

## 8. 解决方案 D：检查 PX4 降落/航向相关参数

如果人工切 `AUTO.LAND` 也会扭，代码侧可能不是主因，需要看 PX4。

建议检查方向：

1. EKF yaw 是否稳定。
2. 磁力计是否受室内电磁干扰。
3. 是否启用了不适合室内环境的磁航向融合。
4. 降落模式是否有 yaw 行为相关参数。
5. 机体 yaw rate / yaw P 参数是否过激。

可观察：

```text
QGC / MAVLink Inspector:
ATTITUDE.yaw
LOCAL_POSITION_NED
vehicle_attitude
vehicle_attitude_setpoint
vehicle_local_position
vehicle_status.nav_state
```

如果切 `AUTO.LAND` 时：

```text
vehicle_attitude_setpoint.yaw_body 突然变化
```

那就是飞控内部 yaw setpoint 改变。  
如果：

```text
yaw 估计本身跳变
```

更可能是 EKF / 磁力计 / 传感器融合问题。

## 9. 解决方案 E：切 AUTO.LAND 前先切到 Hold/Position 类模式

如果飞控支持，并且测试有效，可以尝试：

```text
OFFBOARD -> POSCTL/HOLD 稳一下 -> AUTO.LAND
```

但这个方案不一定适合当前全自主流程。

缺点：

- 模式链路更复杂。
- 中间模式是否接受、是否稳定，取决于 PX4 配置。
- 对比赛任务而言，可能增加不可控因素。

一般不作为首选。

## 10. 解决方案 F：降低 yaw 控制激烈程度

如果扭转幅度不大但动作很猛，可以检查和调低 yaw 控制相关参数。

方向：

```text
降低最大 yaw rate
降低 yaw 控制增益
检查机体惯量、桨叶、电机响应是否导致 yaw 过冲
```

这个属于飞控调参，不建议在没有日志的情况下盲调。

## 11. 推荐实施顺序

建议按下面顺序来：

1. 先确认人工切 `AUTO.LAND` 是否也扭。
2. 打印并记录切换前后的 `current_yaw/current_speed/current_state.mode`。
3. 代码先加“降落准备阶段”，切模式前锁位置和固定 yaw 0.5~1.0 秒。
4. 将 `publish_neutral_setpoint()` 在降落流程里改为使用 `land_locked_yaw`。
5. 如果仍明显扭，查看 PX4 yaw setpoint 和 yaw estimate 日志。
6. 如果确认是 `AUTO.LAND` 内部 yaw 接管不可接受，再考虑 OFFBOARD 受控下降。

## 12. 对当前项目最建议的改法

当前最稳妥的代码侧组合是：

```text
方案 A + 方案 B
```

当前代码已按该组合实现：`LANDING/EMERGENCY_LAND/DISARMING` 在真正切 `AUTO.LAND` 前，会先记录 `land_locked_yaw` 和当前位置保持点，并短暂发布位置锁定 + 0 速度 + 固定 yaw setpoint。

即：

```text
收到 /uav/land:
  保存当前 x/y/z/yaw 为 land_hold 目标
  位置锁定 + 固定 yaw 保持 0.8 秒
  速度足够低后切 AUTO.LAND

收到 /uav/stop:
  保存当前 yaw
  零速度 + 固定 yaw 保持 0.2 秒
  立即切 AUTO.LAND
```

这样既不大幅改变安全流程，又能减少 yaw 控制权交接时的突变。

## 13. 简短结论

切 `AUTO.LAND` 瞬间机身扭转，不应简单理解为“对齐地磁场方向”。它更可能是：

```text
OFFBOARD yaw setpoint -> PX4 AUTO.LAND yaw setpoint
```

交接时目标 yaw 或 yaw 估计不连续。

优先解决方式是：

```text
切 AUTO.LAND 前先稳定保持，并固定 land_locked_yaw；
如果人工切 AUTO.LAND 也扭，再查 PX4 yaw 估计、磁力计和降落模式参数；
如果 AUTO.LAND 本身不可控，再改 OFFBOARD 受控下降。
```
