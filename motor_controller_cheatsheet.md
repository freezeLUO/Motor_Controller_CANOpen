# motor_controller.py 功能速查

针对当前版本的 `motor_controller.py` 制作的速查表。示例默认已执行：

```python
import canopen
from motor_controller import PPConfig, ProfilePositionController, SyncProducerHelper
```

## 初始化与网络

- `PPConfig(...)` – 节点 ID、编码器分辨率、轮廓速度、PDO 传输类型、同步周期等配置参数。
- `ProfilePositionController(network, cfg)` – 添加节点、加载 EDS（忽略部分默认写入失败），缓存控制器状态。
- `controller.initialise()` – CiA‑402 启动：复位通信→PRE‑OP→设置 0x6060=PP→配置轮廓参数与 PDO→OPERATIONAL→可选配置 SYNC。
- `SyncProducerHelper(network)` – 通过 SDO 操作远端节点的 SYNC 生产者 (0x1005/0x1006)。

## 状态控制

- `controller.clear_faults()` – 写入控制字 0x80 并延时，清除故障。
- `controller.enable_operation()` / `controller.disable_operation()` – 进入或退出 Operation Enabled，自动等待 402 状态机节拍。
- `controller.quick_stop()` – 控制字 0x02 快速停机，等待状态确认。
- `controller.shutdown()` – 关停控制器，必要时停止 sync，保留网络连接。

## 模式切换

- `switch_to_profile_position_mode(pdo_mapping=True)` – 切回 PP，预载当前位置到 0x607A。
- `switch_to_profile_velocity_mode(pdo_mapping=True)` – 切换到 PV，必要时重映射速度 PDO。
- `switch_to_profile_torque_mode(initial_torque, pdo_mapping=True)` – 切换到 PT，预载初始扭矩并重映射。
- `switch_to_cyclic_synchronous_position(feedforward_control=False)` – 进入 CSP；若 `feedforward_control=True`，会为速度/力矩前馈新增 RPDO2。
- `switch_to_cyclic_synchronous_velocity()` – 进入 CSV，配置/重映射速度 PDO。
- `switch_to_cyclic_synchronous_torque(initial_torque=0)` – 进入 CST，配置/预载扭矩。

模式对象 0x6060 取值：PP=0x01，PV=0x03，PT=0x04，CSP=0x08，CST=0x09，CSV=0x0A。

## 运动下发

- `set_profile(velocity_deg_s, accel_deg_s2, decel_deg_s2)` – 通过 SDO 更新 PP 轮廓参数。
- `set_target_counts(counts, is_csp=False)` / `set_target_angle(deg, is_csp=False)` – PP/CSP 目标位置（含 set‑point toggle）。
- `set_target_velocity_counts(counts, *, halt=False, is_csv=False)` / `set_target_velocity_deg_s(deg_s, ...)` – PV/CSV 速度指令；`halt=True` 置位 Halt 位。
- `set_target_velocity_deg_s_sdo(deg_s, *, halt=False)` – 仅通过 SDO 下发速度。
- `set_target_torque(torque_value, *, halt=False, is_cst=False)` – PT/CST 目标扭矩；CST 下发控制字固定 0x000F。
- `set_target_torque_sdo(torque_value, *, halt=False)` – 仅通过 SDO 下发扭矩。
- `set_csp_target_with_feedforward(position_deg, velocity_feedforward_deg_s, torque_feedforward)` – CSP 位置 + 速度/力矩前馈；内部可用 RPDO2 或回退 SDO 写 0x60B1/0x60B2。

## 数据读取

- `get_statusword()` – 状态字 0x6041（优先 TPDO1，回退 SDO）。
- `get_error_code()` – 错误字 0x603F（SDO）。
- `get_position_counts(allow_sdo_fallback=True)` / `get_position_angle(...)` – 先读 TPDO1 0x6064，失败可回退 SDO，内部保留上次缓存。
- `get_velocity_counts(...)` / `get_velocity_deg_s(...)` – 读取实际速度（TPDO1/SDO）。
- `get_position_angle_sdo()` / `get_velocity_deg_s_sdo()` – 纯 SDO 读取位置/速度。
- `get_actual_torque_sdo()` – SDO 0x6077 实际力矩。
- `get_motor_rated_torque(raw=False)` – SDO 0x6076 额定力矩（默认 Nm，`raw=True` 返回原始值）。
- `is_target_reached()` – 检查状态字 bit10（0x6041 & 0x0400）。

## PDO 重映射

- `_try_remap_pdos_via_sdo()` – 初始 PP/PDO 映射（`initialise` 内调用）：
  - TPDO1: 0x6041(Statusword, 16bit), 0x6064(Position actual, 32bit)
  - RPDO1: 0x6040(Controlword, 16bit), 0x607A(Target position, 32bit)
- `try_remap_pdos_for_velocity_mode()` – PV/CSV：RPDO1 映射 0x6040(16bit) + 0x60FF(Target velocity, 32bit)。
- `try_remap_pdos_for_torque_mode()` – PT/CST：RPDO1 映射 0x6040(16bit) + 0x6071(Target torque, 16bit)。
- `try_remap_pdos_for_csp_with_feedforward()` – CSP（带前馈）：
  - TPDO1: 0x6041(16bit) + 0x6064(32bit)
  - RPDO1: 0x6040(16bit) + 0x607A(32bit)
  - RPDO2: 0x60B1(Velocity offset, 32bit) + 0x60B2(Torque offset, 16bit)

## 控制参数访问

- `ControllerParameters` – 数据类：位置/速度/电流环增益、速度前馈、积分限幅、PID 开关。
- `read_control_parameters()` – 读取 0x2380/0x2381/0x2382/0x2383/0x3000。
- `write_control_parameters(params, verify=True)` – 写入非空字段，带合法性校验和可选回读验证。

## 其他工具

- `angle_to_counts()` / `counts_to_angle()` 及速度换算函数。
- `configure_sync(period_s)` – 本地主站 SYNC 重配置。
- `SyncProducerHelper.enable_sync_producer(node_id, period_ms)` / `disable_sync_producer(node_id)` – 远端 SYNC 生产者开关。

注意事项：
- 确保 `ZeroErr Driver_V1.5.eds` 可访问，并根据实际硬件调整 CAN 接口（pcan/socketcan 等）。
- 高频控制优先使用 PDO；SDO 顺序阻塞，避免过度占用总线。
- 通过 PDO 写目标值时使用 set‑point toggle；CST 下发控制字固定 0x000F 以避免冲突。
