# 位置模式控制库

这是一个基于 `python-canopen` 的轻量级封装，负责将 CiA-402 节点配置为 Profile Position（PP）模式，并提供若干状态机控制与运动命令的便捷方法。

## 功能特性

- 在 PP 模式下完成 PDO、SYNC、SDO 的初始化配置。
- 具备 300&nbsp;ms 间隔的故障清除与使能/失能流程。
- 支持更新位置模式的速度、加速度、减速度参数。
- 以编码器计数或角度下发目标位置。
- 读取当前位置计数/角度以及目标到达标志位。
- 可选启用 `network.sync` 作为 SYNC 生产者。
- 按照手册流程切换至周期同步位置模式（CSP）。
- 提供 `SyncProducerHelper` 用于把远程节点配置为 SYNC 生产者。

## 基本用法

```python
import canopen
from pp_lib.motor_pp_controller import PPConfig, ProfilePositionController

network = canopen.Network()
network.connect(bustype="pcan", channel="PCAN_USBBUS1", bitrate=1_000_000)

cfg = PPConfig(node_id=0x01, sync_period_s=0.01)
controller = ProfilePositionController(network, cfg)
controller.initialise()
controller.clear_faults()
controller.enable_operation()
controller.set_profile(velocity_deg_s=20.0, accel_deg_s2=50.0, decel_deg_s2=50.0)
controller.set_target_angle(90.0)

angle = controller.get_position_angle()
print(f"current angle: {angle:.2f}°")

controller.shutdown()
network.disconnect()
```

根据实际硬件调整网络参数、节点 ID 与位置模式曲线。库内部默认不启用日志，可在上层应用自行打开以便观察状态转换。

## 切换至 CSP 模式

若需要进入周期同步位置模式，可在完成基本初始化后调用：

```python
controller.switch_to_cyclic_synchronous_position(sync_period_s=0.01)
```

方法内部会在停机状态下完成模式切换、将实际位置写回目标位置、重新配置 SYNC 并重新使能驱动。可通过参数覆盖默认的同步周期。未显式传入时会复用 `PPConfig.sync_period_s` 配置。

## 由指定节点产生 SYNC

若不希望主站发送同步帧，可通过辅助类完成配置：

```python
from pp_lib.motor_pp_controller import SyncProducerHelper

sync_helper = SyncProducerHelper(network)
sync_helper.enable_sync_producer(node_id=0x02, period_ms=10)

# 需要停止时
sync_helper.disable_sync_producer(0x02)
```

调用前请确保网络中已添加该节点对象，且驱动允许通过 SDO 修改 0x1005 与 0x1006。
