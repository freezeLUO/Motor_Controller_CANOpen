# motor_controller.py 功能速查

针对 `motor_controller.py` 暴露的常用 API 制作的速查表。示例默认已执行：

```python
import canopen
from motor_controller import PPConfig, ProfilePositionController, SyncProducerHelper
```

## 初始化与网络

- `PPConfig(...)` – 节点 ID、编码器分辨率、轮廓速度、PDO 传输类型、同步周期等配置参数。
- `ProfilePositionController(network, cfg)` – 添加节点、加载 EDS、缓存控制器状态。
- `controller.initialise()` – CiA-402 安全启动流程：复位通信、进入 PRE-OP、配置轮廓与 PDO、进入 OPERATIONAL、可选配置 SYNC。
- `SyncProducerHelper(network)` – 通过 SDO 操作远端节点的 SYNC 生产者 (0x1005/0x1006)。

## 状态控制

- `controller.clear_faults()` – 写入控制字 0x80 并延时，清除故障。
- `controller.enable_operation()` / `controller.disable_operation()` – 进入或退出 Operation Enabled，自动等待 402 状态机节拍。
- `controller.quick_stop()` – 控制字 0x02 快速停机，等待状态确认。
- `controller.shutdown()` – 关停控制器，必要时停止 sync，保留网络连接。

## 模式切换

- `switch_to_profile_position_mode()` – 回到 PP 模式并预载上一次位置。
- `switch_to_profile_velocity_mode()` – 重映射速度 PDO、写 0 速度、切换到 PV。
- `switch_to_profile_torque_mode(initial_torque)` – 重映射扭矩 PDO、预载初始扭矩。
- `switch_to_cyclic_synchronous_position()` – 预载目标、进入 CSP。
- `switch_to_cyclic_synchronous_velocity()` – 配置 CSV 所需周期并重映射速度 PDO。
- `switch_to_cyclic_synchronous_torque(initial_torque=0)` – 配置 CST，预载扭矩。

## 运动下发

- `set_profile(velocity_deg_s, accel_deg_s2, decel_deg_s2)` – 通过 SDO 更新 PP 轮廓参数。
- `set_target_counts(counts, is_csp=False)` / `set_target_angle(deg, is_csp=False)` – PP/CSP 目标位置指令，包含 set-point toggle。
- `set_target_velocity_counts(counts, halt=False, is_csv=False)` / `set_target_velocity_deg_s(deg_s, ...)` – PV/CSV 速度指令。
- `set_target_torque(torque_value, halt=False, is_cst=False)` – PT/CST 目标扭矩。
- `set_csp_target_with_feedforward(position_deg, velocity_feedforward_deg_s, torque_feedforward)` – CSP 双 RPDO，位置+速度/力矩前馈。

## 数据读取

- `get_position_counts(allow_sdo_fallback=True)` / `get_position_angle(...)` – 先读 TPDO1 0x6064，失败时可回退 SDO 并缓存最后值。
- `get_velocity_counts(...)` / `get_velocity_deg_s(...)` – 读取实际速度。
- `get_actual_torque()` – SDO 0x6077 实际力矩。
- `get_motor_rated_torque(raw=False)` – SDO 0x6076 额定力矩（默认 Nm，`raw=True` 返回原始值）。
- `is_target_reached()` – 检查状态字 bit10。
- `wait_for_target(controller, target_angle_deg, timeout, threshold_deg=0.1, consecutive_samples=5)` – 用于 PP 模式等待角度收敛。

## PDO 重映射

- `_try_remap_pdos_via_sdo()` – 初始 PP 映射尝试（`initialise` 内调用）。
- `try_remap_pdos_for_velocity_mode()` – CSV/PV 模式的 PDO 重映射。
- `try_remap_pdos_for_torque_mode()` – PT/CST 模式重映射（0x6071 以 16-bit 映射）。
- `try_remap_pdos_for_csp_with_feedforward()` – CSP 增加前馈 RPDO。

## 轨迹辅助

- `plan_sine_trajectory(base_angle, cycles=1.0)` – 基于全局周期参数生成正弦轨迹。
- `plan_linear_trajectory(base_angle, angle_change_deg=-30.0)` – 线性爬坡轨迹。
- `plan_quintic_trajectory(...)` – 三段五次多项式往返轨迹。
- `run_csp_trajectory(network, controller, trajectory)` – SYNC 驱动的 CSP 轨迹执行（轮询总线）。
- `plot_results(timestamps, planned, actual)` – Matplotlib 对比曲线。

## 控制参数访问

- `ControllerParameters` – 数据类：位置/速度/电流环增益、速度前馈、积分限幅、PID 开关。
- `read_control_parameters()` – 读取 0x2380/0x2381/0x2382/0x2383/0x3000。
- `write_control_parameters(params, verify=True)` – 写入非空字段，带合法性校验和可选回读验证。

## 其他工具

- `angle_to_counts()` / `counts_to_angle()` 及速度换算函数。
- `configure_sync(period_s)` – 本地主站 SYNC 重配置。
- `SyncProducerHelper.enable_sync_producer(node_id, period_ms)` / `disable_sync_producer(node_id)` – 远端 SYNC 生产者开关。

请确保 `ZeroErr Driver_V1.5.eds` 可访问，并根据实际硬件调整 CAN 接口（pcan/socketcan 等）。高频控制优先使用 PDO；SDO 调用是顺序的，需避免过度占用总线。
