# C 车 yaw 延迟观测

这项测试在专用 `yaw_test_mode` 下运行，不会在正常自瞄模式自动启动。它用短暂的
`+力矩、零力矩、-力矩、零力矩` 序列，分别记录主机设置目标、将 CAN 帧加入发送队列、
收到电机反馈、收到云台和底盘 IMU 数据的时刻。记录的是**软件能够观察到的事件时间**；
没有示波器或电机内部时间戳，不能据此宣称微秒级物理响应延迟。

## 运行

1. 按 C 车 YAML 顶部注释切换到仅云台测试模式，部署 C 车程序。确保底盘固定、云台
   两侧和线缆安全、遥控器使能有效。正常配置的 `yaw_test_mode: false` 不会暴露测试服务。
2. 测试服务如下；默认约 4.2 秒，按遥控使能和转角、速度阈值自动停止。

   ```bash
   ros2 service call /rmcs/service/yaw_latency/start std_srvs/srv/Trigger '{}'
   ```

   需要中止时：

   ```bash
   ros2 service call /rmcs/service/yaw_latency/stop std_srvs/srv/Trigger '{}'
   ```

3. 程序日志给出 `/tmp/yaw_latency_<timestamp>.csv` 路径、事件数量和丢弃数量。
   CSV 在运动停止后由后台线程写入；复制文件前等待文件大小稳定。若有丢弃事件，
   本轮数据不能用于延迟结论。
4. 在本地分析：

   ```bash
   python3 experiments/yaw/analyze_yaw_latency.py /path/to/yaw_latency_....csv
   ```

   脚本输出 `.summary.json` 和 `.edges.csv`。每次切换的时间用前后两条反馈样本
   给出区间，并将加力矩与撤力矩分别汇总。首次电流变化要求连续两帧超过噪声门限；
   电流达到 50% 的时间还包含电流爬升过程。这些结果必须连同反馈采样间隔一起看。

   2026-10-13 的两轮实车结果见 [测试报告](latency_2026-10-13/results.md)。

## CSV 字段

| 事件 | `value1` | `value2` | `value3` | `board_tick_quarter_us` |
| --- | --- | --- | --- | --- |
| `request` | 目标力矩 N·m | 测试开始后的秒数 | — | 0 |
| `can_enqueue` | 加入主机发送队列的目标力矩 N·m | — | — | 0 |
| `motor_feedback` | 电机反馈原始电流 | 原始速度 | 原始编码器计数 | 0 |
| `gimbal_gyro` | 云台本体系 z 角速度 rad/s | x 角速度 | y 角速度 | 云台板 IMU 采样时间 |
| `chassis_gyro` | 底盘本体系 z 角速度 rad/s | x 角速度 | y 角速度 | 底盘板 IMU 采样时间 |
| `test_end` | 1 表示完整结束，0 表示中止 | — | — | 0 |

`host_ns` 是同一主机单调时钟的纳秒读数；`can_enqueue` 不是 CAN 线上实际发送时刻。
当前配置的更新频率为 1 kHz，而 yaw CAN 指令只在隔一次更新时入队，标称指令间隔约
2 ms；纳秒格式的时间戳不能提高事件本身的采样频率。
电机 CAN 反馈在当前 SDK 中没有板载采样时间戳。两块板的 `board_tick_quarter_us`
有各自的时钟起点，不能直接相减。该字段的四分之一微秒刻度也不等于 IMU 的采样间隔。

分析脚本把电流首次达到本次实测变化量的 50% 作为反馈门限；云台 IMU 与底盘 IMU
经主机到达时间近似对齐后，检测首次持续超过噪声门限的角速度变化；编码器使用
3 个计数的门限。后两者是“**首次可检测到运动**”的时间，随门限、采样率和安装
姿态变化，不是力矩作用于机械结构的瞬时物理延迟。
