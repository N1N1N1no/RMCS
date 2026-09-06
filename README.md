# RMCS

RoboMaster Control System based on ROS2 —— 基于 [Alliance-Algorithm/RMCS](https://github.com/Alliance-Algorithm/RMCS) 的个人学习 / 调试分支。

本分支用于 RoboMaster 训练周任务：在 RMCS 框架上完成**发射机构（Shooting）架构梳理**，以及**电机控制实验（接入 DR16 遥控器，单环 / 双环控制 DJI 电机）**。

## 基于 RMCS 完成的功能

### 1. 任务一：发射机构（Shooting）组件关系梳理

- 梳理了 RMCS `rmcs_core/src/controller/shooting/` 发射机构子系统的组件关系、端口数据流、热量管控与安全链；
- 任务链接（飞书）:https://fa4g5no1b1f.feishu.cn/docx/HSkld3B0xoUcDrxfYs9c3FEJnlw?blockId=doxcno0g4Bj3y7LI637iouNBThd&blockToken=CBkPwvJaDhNXfPbtxMDcKOWznee&blockType=whiteboard&doc_app_id=501&openbrd=1#doxcno0g4Bj3y7LI637iouNBThd。

### 2. 任务二 / 三：电机控制实验（motor_test）

新增基于 C 板（librmcs CBoard）+ DJI 电机的电机测试框架，组件注册在 `rmcs_ws/src/rmcs_core/plugins.xml`，完整复用了 RMCS 的 Component / 命名端口、`pid::PidController`、`ValueBroadcaster` 等机制：

| 配置文件 | 电机 | 控制方式 | 功能 |
| --- | --- | --- | --- |
| `motor_test_single.yaml` | M3508（CAN1，减速比 13:1） | 速度单环 PID | DR16 左摇杆映射目标速度 |
| `motor_test_double.yaml` | GM6020（CAN1，多圈角度） | 角度环 + 速度环双 PID | DR16 左摇杆给定目标角度 |

#### 2.1 单环速度控制：`MotorTestSingle` + `MotorTestSingleController`

- 硬件层（`motor_test_single.cpp`）：C 板接收 DBUS 遥控数据并注册 DR16，读取 M3508 实际转速 `/test/motor/velocity`；
- 控制器（`motor_test_single_controller.cpp`）：
  - **安全逻辑**：左右拨杆均非 UNKNOWN 且不同时拨下才使能；禁用 / 无遥控时目标速度为 0；
  - **左摇杆 y 轴 → 目标速度**：`target_velocity = joystick.y * max_velocity`（默认最大 50 rad/s，参数可调）；
  - **定速调试模式** `fixed_velocity_enabled`：由参数 `target_velocity` 直接给定目标速度恒速转动；
  - 速度反馈经一阶低通滤波（默认 100 Hz）输出 `measured_velocity`，并输出 `velocity_error` 供调试可视化；
- PID：速度环由 RMCS 共享组件 `pid::PidController`（`velocity_pid_controller`）实现，`kp=0.3, ki=0.005`，输出力矩限幅 ±50。由于 M3508 减速比 13:1，PID 增益整体偏大；
- 实测：电机转速可稳定跟踪目标速度（见下方波形）。

#### 2.2 双环角度控制：`MotorTestDouble` + `MotorTestDoubleController`

- 硬件层（`motor_test_double.cpp`）：C 板接入 DR16，读取 GM6020 多圈角度 `/test/motor/angle` 与转速 `/test/motor/velocity`；
- 控制器（`motor_test_double_controller.cpp`）按目标角生成方式提供三种可配置模式：
  - **遥控角度模式** `remote_angle_enabled`（当前 yaml 默认开启）：左摇杆偏离中位（死区 0.1 rad，带回差）→ 目标角 = `remote_angle_setpoint`（默认 3.0 rad）；回中 → 目标角 0；左右拨杆双下 / 无遥控 → **锁存并保持当前位置**；
  - **梯形波模式** `trapezoid_enabled`（默认关闭，调试用）：按幅值 / 上升时间 / 保持时间生成周期性梯形目标角，便于观察跟踪性能；
  - **优弧模式** `major_arc_enabled`（当前默认开启）：把设定角解算为距离当前实际角最近的多圈目标，避免跨越 0 / 2π 边界时空转一整圈，带到达容差（默认 0.05 rad）与启动稳定延时；
- 双环：外环角度 PID（`angle_pid_controller`，`kp=5.5, kd=0.15`，输出目标速度 ±100）→ 内环速度 PID（`velocity_pid_controller`，`kp=0.035, ki=0.0015`，输出力矩 ±80）；
- 角度 / 速度反馈均可选低通滤波；`target_angle`、`angle_error`、`control_velocity`、`control_torque` 等中间量全部通过端口输出。

### 3. 调试与可视化

- 两套配置均挂载 `ValueBroadcaster`，把 `/test/motor/*`（角度、转速、误差、控制量等）统一转发为可订阅话题，供 PlotJuggler / Foxglove 等工具订阅绘制波形；
- 实测波形存放在 `docs/zh-cn/images/`：

**单环速度响应（设定速度 50 rad/s）**

![单环速度响应](docs/zh-cn/images/single.png)

**双环角度跟踪**

![双环角度跟踪](docs/zh-cn/images/double.png)


## 相关文件

| 类型 | 路径 |
| --- | --- |
| 单环硬件 | `rmcs_ws/src/rmcs_core/src/hardware/motor_test_single.cpp` |
| 单环控制器 | `rmcs_ws/src/rmcs_core/src/controller/motor_test_single_controller.cpp` |
| 双环硬件 | `rmcs_ws/src/rmcs_core/src/hardware/motor_test_double.cpp` |
| 双环控制器 | `rmcs_ws/src/rmcs_core/src/controller/motor_test_double_controller.cpp` |
| 单 / 双环配置 | `rmcs_ws/src/rmcs_bringup/config/motor_test_single.yaml`、`motor_test_double.yaml` |
| 插件注册 | `rmcs_ws/src/rmcs_core/plugins.xml` |
| 任务一文档 | `docs/zh-cn/shooting_architecture.md` |
| 实测波形图 | `docs/zh-cn/images/single.png`、`docs/zh-cn/images/double.png` |
| 周任务说明 | `week2.md` |

## 调试心得 / 已知问题

- M3508 单环速度控制效果较好；GM6020 双环（角度环）效果一般，仍有调优空间；
- 调试中偶发 launch 后剧烈振荡：手扶电机底座使其稳定后松手即恢复正常且运行顺滑，怀疑与电机底座未固定（机械共振）有关，建议固定电机后复测；
- 速度环 PID 增益受减速比影响较大（13:1），更换电机 / 减速比时需重新整定。
