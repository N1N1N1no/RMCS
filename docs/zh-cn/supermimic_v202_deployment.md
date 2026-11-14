# SuperMimic V202_02 学生策略部署

车辆配置为 `wheel-leg-infantry-supermimic`，对应 `policy: supermimic`。原 Wheelbipe 老师继续使用 `wheel-leg-infantry-wheelbipe`。两者具有独立观测契约。控制器、自起、遥控器、驱动与云台均沿用官方 `RMCS/dev/wheel-leg-refactor`（`ac6bfa7e`）；学生配置以该分支的 `wheel-leg-infantry-wheelbipe-recovery.yaml` 为基础，替换策略选择、模型契约与观测/动作定义，并使用 V202_02 的 40–120° 机械域和高度命令范围。自起阶段顺序、捕获窗口和超时保持上游配置。

运行文件位于 `rmcs_ws/src/rmcs_rl/models/wheel_leg/`：

| 文件 | 用途 |
| --- | --- |
| `supermimic_v202_120_24000_50hz.onnx` | 最终 24000 轮 actor，包含训练得到的归一化参数 |
| `supermimic_v202_120_24000_50hz.contract.json` | 关节顺序、历史、命令、动作与物理接口约定 |
| `supermimic_v202_120_24000_50hz.deployment.json` | 原检查点及 ONNX 的 SHA-256、导出与精度校验回执 |

本配置使用 Runs/V202_02 中 `v202120_run_01` 的 24000 轮检查点，ONNX SHA-256 为 `3b9aa5023afa6b7e8350c76f7eede6e20b7e81ccdb4d341f2f5dc5d46ec5c887`。编码器零位仍按 110° 建模，未改硬件零位或方向。

2026-11-10 已用当前官方 C++ 自起/控制插件与 V202_120 MuJoCo 物理模型重验：52 组起姿中 45 组接管、36 组达到严格稳定标准，5 组运动/高度任务通过。110° 斜伸起姿、关节共同整圈后的观测，以及精确 120° 边界仍有已复现问题；当前结果仅确认接入与部分场景表现，尚未通过完整行为验收。

实车运行仅需 C++ 与 ONNX Runtime。训练用世界模型、critic、优化器和 `.pt` 检查点不参与推理。

## 构建与选择配置

在 RMCS 的 Jazzy 开发容器中，按照仓库约定使用 `build-rmcs`：

```bash
cd /workspaces/RMCS
build-rmcs --packages-select rmcs_core rmcs_rl rmcs_bringup --cmake-args -DBUILD_TESTING=ON
source rmcs_ws/install/setup.bash
```

首次准备全新工作区时先运行一次 `build-rmcs`，以构建 executor、描述和消息等依赖。

直接选择配置运行：

```bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-supermimic
```

该命令加载实车硬件组件，接通驱动并使能后会输出力矩。离线验证使用下文的 fixture。

使用既有实车部署容器时，在开发容器完成构建，按项目原有流程执行 `sync-remote`；在实车部署容器中执行：

```bash
set-robot wheel-leg-infantry-supermimic
service rmcs restart
service rmcs attach
```

`set-robot` 修改后续 shell 的默认车型；当前 shell 可显式传入 `robot:=...`，或重新加载其环境文件。模型和 YAML 都在 install 树内，由 `sync-remote` 一并同步。执行同步时必须在能解析 `/workspaces/RMCS` 符号链接的开发容器中。

回退到老师时重新选择 `wheel-leg-infantry-wheelbipe`，并重启 RMCS。此时应整体切换车型配置，老师和学生的观测维度不同。

## 官方子模块版本

`rmcs_rl` 使用官方仓库 `https://github.com/Alliance-Algorithm/rmcs_rl` 的 `main`（`26d29ce0`，2026-11-13 已核对远端），与父仓库的子模块引用一致。保留本地学生模型、配置回归和推理对照工具；子模块运行时源码沿用该上游提交。SuperMimic 的绕圈版本通过 `expected_metadata.leg_winding_version: paired_full_vehicle120_zero110_v1` 检查，模型、观测、频率和归一化校验继续生效。

## 实车接口

RMCS executor 为 1000 Hz，腿和轮反馈控制为 200 Hz，学生推理与观测更新为 50 Hz。ONNX 在独立工作线程运行。该分支已采用进程内推理；飞书文档中的独立 `policy_server` 启动方式适用于其他版本。

每帧 46 维观测严格按训练顺序组装：16 个非轮关节位置相对 nominal 的偏移、18 个关节速度、机体角速度 3 维、机体系重力方向 3 维、上一条已执行的原始动作 6 维。训练时的 RunningNorm 位于 ONNX 内部，外部关闭归一化和观测裁剪。

`WheelLegObservation` 内的 `Supermimic` 适配器维护最旧到最新的 10 帧，共 460 维。随后附加 40 维目标，即当前经过控制器处理的 `[vx, 0, wz, height]` 重复 10 次。实车使用保持当前命令的未来假设；高度以米输入，沿用训练的数值尺度。ONNX 输入为 `obs[1,500]`，输出为 `actions[1,6]`。因此桥配置仍为 `history_length: 1`；完整的历史已包含在生产者的 500 维数组中。生产者输出为 `/wheel_leg/rl/input/supermimic_frame`，桥接器另行发布 `/wheel_leg/rl/input/frame` 供诊断，两者分开命名以避免重复输出或自引用。

六个编码器只提供主动轴状态。`SupermimicFullState` 使用与训练相同的 V202 CAD 刚体树和六个球形闭环约束，重建其余十二个关节的位置。查表仅用于选择装配分支，随后用三维闭环求解修正；速度通过闭环 Jacobian 求解。求解不收敛、关节超出机械域或输入非有限时停止提交观测，现有动作超时保护随后生效。它估计的是刚体状态，真实间隙和结构弹性仍会造成观测偏差。

输出顺序为 `LH,RH,LK,RK,LW,RW`，RMCS 消费器内部使用 `LH,LK,RH,RK,LW,RW`，适配器负责重排。腿目标为 `nominal + 0.5 * raw_action`，随后使用同侧配对绕圈投影及机械相对角限制；轮目标为 `clip(10 * raw_action, -90, 90)` rad/s。历史保存原始动作。腿 P/D 为 60/2，轮速度 P 为 0.5，保留原硬件扭矩帽，气弹簧抵消前馈为关闭。

复位、退出使能或状态失效会清空学生历史；首帧重复填满 10 帧。复位后的原始动作反馈由原控制器清零。观测 SHA、机械模型 SHA、归一化类型、绕圈版本、输入尺寸和策略频率由模型加载器及控制器共同检查。

## 指令与初次实车验证

新配置沿用 V202 的硬件序列号、编码器零位、方向、减速比与扭矩保护。必须与待运行的同一台机器人对应；这些标定不从神经网络自动推导。

指令、自起、云台 PID 和状态估计配置均与上游自起车型一致：`vx_max: 3.0` m/s、`yaw_rate_max: 4.0` rad/s，旋转模式 `spin_yaw_rate: 10.471975511965978` rad/s；控制器还会执行策略自身的速度范围约束。高度命令范围为 0.23–0.44 m；目标或过渡高度大于 0.42 m 时，前进/转向命令限制为 ±0.5 m/s、±0.5 rad/s，默认 0.305 m。状态估计采用上游 `velocity_sign: 1.0`，四个腿电机的 API 速度到模型速度取反已在 `model_velocity()` 内处理。

训练未包含倒地自起。本配置沿用上游 `self_righting.enabled: true`、`simulation_only: false`、`hardware_mapping_verified: true` 和各阶段 2 rad/s 的参考速度设置。双下复位后拨至双中，等待状态估计和驱动就绪、自起偏航对齐，进入 `rl/state=4`。自起期间学生策略进行影子推理，执行器仍由上游自起控制器掌握；动态接管条件满足且动作新鲜后切换为 `rl/state=3`。不再配置旧版 `self_righting_overspeed_fault_enabled`、`self_righting_speed_braking_enabled`，因为当前上游控制器不读取这两个键。动作超时和 `drive_loss_debounce: 0.1` 沿用上游。

首次测试应先在卸载驱动的状态核对六轴测量值、IMU 方向、零位和模型加载日志，再用防跌支撑下的站立、小速度、转向和变高逐项检查。沿用 DR16 双下复位、双中使能、DR16 失联和策略动作过期保护。仿真通过只说明当前软件接口和仿真行为；实车的轴映射、时延与稳定性需要独立验收。

诊断可读取已有 `/wheel_leg/rl/healthy`、`valid`、`state`、`fault_latched`、`performance/inference_us`，并检查 `frame_sequence` 是否每 20 ms 更新。`healthy` 表示推理线程健康；`valid` 还要求本次使能会话中存在新鲜动作。

## 可重复的离线验证

先用 `build-rmcs ... -DBUILD_TESTING=ON` 构建。启动接口工具 `rmcs_ws/build/rmcs_core/supermimic_startup_smoke` 加载实际状态估计、遥控指令、RL 控制器、观测及桥接器，并使用合成传感器；它的固定组件列表不包含硬件插件。仅检查初始化绑定可以运行：

```bash
cd /workspaces/RMCS/rmcs_ws
source install/setup.bash
build/rmcs_core/supermimic_startup_smoke --ros-args \
  --params-file src/rmcs_bringup/config/wheel-leg-infantry-supermimic.yaml < /dev/null
```

初始化成功输出 `READY`。

本地工具 `rmcs_ws/build/rmcs_core/supermimic_policy_fixture` 只接收数字，校验闭环重建、历史顺序及目标映射；`rmcs_ws/build/rmcs_rl/supermimic_policy_parity` 使用生产 `PolicyModel` 推理，不加载车辆或硬件组件。

在 SuperMimic 训练仓库中，用含 Torch、MuJoCo、ONNX 与 ONNX Runtime 1.20 的 Python 导出：

```bash
python tools/export_supermimic_rmcs.py \
  --checkpoint /absolute/path/to/ckpt_024000.pt \
  --training_repo /absolute/path/to/isaac_sim-6-task \
  --output /absolute/path/to/export
```

导出器验证 V202 资产和关节顺序，生成 ONNX、闭环装配数据、契约与 128 组原权重推理向量。更换机械模型、观测或历史长度时需要更新对应 C++ 适配器及契约。

随后可运行完整的离线接口验证：

```bash
python tools/verify_supermimic_rmcs.py \
  --rmcs /absolute/path/to/RMCS \
  --checkpoint /absolute/path/to/ckpt_024000.pt \
  --training_repo /absolute/path/to/isaac_sim-6-task \
  --export /absolute/path/to/export \
  --output /absolute/path/to/RMCS/.codex/report/supermimic_deploy/validation \
  --cpp-runtime
```

`--cpp-runtime` 用本机已有的 `qzhhhi/rmcs-develop:latest-full` 镜像验证同一 ROS ABI 的生产 C++ 推理程序，并测试错误契约和非有限输入的拒绝。闭环测试直接调用编译后的 C++ 观测适配器，使用 ONNX Runtime 1.20，在 MuJoCo 内运行 9 组固定指令各 30 秒；跟踪误差排除前 3 秒，跌倒检测覆盖全程。结果和轨迹进入指定报告目录。

RL 原有回归在开发容器中运行：

```bash
cd /workspaces/RMCS/rmcs_ws
source install/setup.bash
colcon test --merge-install --packages-select rmcs_rl
colcon test-result --test-result-base build/rmcs_rl --verbose
```

部署模型位于 `rmcs_rl` Git 子模块中，控制器和车辆 YAML 位于父仓库。通过 Git 交付时，子模块的模型和测试改动须先形成可获取的提交，再更新父仓库的子模块引用。
