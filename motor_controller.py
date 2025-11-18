"""基于 python-canopen 的电机控制辅助库。

封装 PP、PV、PT、CSP、CSV、CST 模式常用的节点初始化、状态机控制、轮廓参数设置、目标位置下发以及
位置读取等操作，方便在上层应用中直接复用，不包含监控线程或交互逻辑。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, cast

import canopen  # type: ignore
from canopen.objectdictionary import ObjectDictionaryError  # type: ignore
try:  # python-canopen >= 2.x
    from canopen.sdo.exceptions import SdoAbortedError  # type: ignore
except Exception:  # pragma: no cover - compatibility fallback
    class SdoAbortedError(Exception):  # type: ignore
        pass

log = logging.getLogger(__name__)
# logging.basicConfig(level=logging.INFO)

@dataclass(slots=True)
class PPConfig:
    """驱动初始化时使用的配置项。"""

    node_id: int
    eds_path: str = "ZeroErr Driver_V1.5.eds"
    encoder_resolution: int = 524_288
    profile_velocity_deg_s: float = 15.0
    profile_accel_deg_s2: float = 15.0
    profile_decel_deg_s2: float = 15.0
    sync_period_s: Optional[float] = None
    tpdo_transmission_type: int = 1
    rpdo_transmission_type: int = 1


@dataclass(slots=True)
class ControllerParameters:
    position_kp: int | None = None
    velocity_kp: int | None = None
    velocity_ki: int | None = None
    current_kp: int | None = None
    current_ki: int | None = None
    velocity_feedforward: int | None = None
    integral_limit_ma: int | None = None
    pid_switch: bool | None = None


class ProfilePositionController:
    """CiA-402 轮廓位置模式的高层控制器封装。"""

    def __init__(self, network: canopen.Network, config: PPConfig) -> None:
        self.network = network
        self.cfg = config
        self.node = self.network.add_node(self.cfg.node_id, self.cfg.eds_path)
        # 新版 python-canopen 会按 EDS 默认值写大量对象，部分设备会返回 0x06090030（参数越界）。
        # 本库会自行按需通过 SDO/PDO 配置所需对象，因此这里忽略 EDS 默认写入失败，继续初始化。
        try:
            self.node.load_configuration()
        except SdoAbortedError as exc:
            log.warning(
                "Node 0x%02X: skip applying EDS defaults due to SDO abort (%s)",
                self.cfg.node_id,
                exc,
            )
        self._last_enable_ts = 0.0
        self._last_disable_ts = 0.0
        self._last_position_counts: Optional[int] = None
        self._last_velocity_counts: Optional[int] = None
        self._control_toggle = True

    # ---------------- 单位换算 -----------------
    def angle_to_counts(self, angle_deg: float) -> int:
        return int(angle_deg / 360.0 * self.cfg.encoder_resolution)

    def counts_to_angle(self, counts: int) -> float:
        return counts / self.cfg.encoder_resolution * 360.0

    def deg_per_s_to_counts(self, value: float) -> int:
        return int(value / 360.0 * self.cfg.encoder_resolution)

    def counts_to_deg_per_s(self, counts: int) -> float:
        return counts / self.cfg.encoder_resolution * 360.0

    # ---------------- 状态/错误读取接口 -----------------
    def get_statusword(self) -> int:
        """读取状态字 (0x6041)。

        优先使用已映射的 TPDO1 中的 "Statusword" 变量获取最新值；如果由于
        未映射/访问异常导致失败，则回退使用 SDO 读取 0x6041。
        返回值为 16 位整型原始状态字。
        """
        try:
            # 快速路径：通过 TPDO 缓存变量（如果映射成功并且已 read()）
            tpdo1 = self.node.tpdo[1]
            try:
                var = tpdo1["Statusword"]
            except KeyError:
                # 有些固件使用其它命名或未映射；抛出让外层捕获
                raise
            # python-canopen 的 TPDO 变量通常在接收回调里更新 raw；这里读取其 raw
            value = int(var.raw)
            return value & 0xFFFF
        except Exception:
            # 回退 SDO 方式（更慢）
            try:
                return int(self.node.sdo[0x6041].raw) & 0xFFFF
            except Exception as exc:
                raise RuntimeError(
                    f"Node 0x{self.cfg.node_id:02X}: failed to read Statusword via PDO/SDO"
                ) from exc

    def get_error_code(self) -> int:
        """读取错误字 (0x603F) 仅通过 SDO。

        返回 16 位错误代码；若读取失败抛出 RuntimeError。注意：某些设备会在无故障时
        返回 0 或特定保留值，需结合厂商手册判断含义。
        """
        try:
            return int(self.node.sdo[0x603F].raw) & 0xFFFF
        except Exception as exc:
            raise RuntimeError(
                f"Node 0x{self.cfg.node_id:02X}: failed to read Error code (0x603F) via SDO"
            ) from exc

    # ---------------- 驱动初始化流程 -----------------
    def initialise(self) -> None:
        log.info("Node 0x%02X: initialising PP mode", self.cfg.node_id)
        self.node.nmt.state = "STOPPED"
        time.sleep(0.1)
        self.node.nmt.send_command(canopen.nmt.NMT_COMMANDS["RESET COMMUNICATION"])
        time.sleep(0.5)
        self.node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.1)

        self.node.sdo[0x6060].raw = 0x01
        if self.node.sdo[0x6061].raw != 0x01:
            log.warning("Node 0x%02X: PP mode not confirmed", self.cfg.node_id)

        self._configure_profile()
        self._configure_pdus()
        self.node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)

        if self.cfg.sync_period_s is not None:
            self.configure_sync(self.cfg.sync_period_s)

    def _configure_profile(self) -> None:
        velocity_cnt = self.deg_per_s_to_counts(self.cfg.profile_velocity_deg_s)
        accel_cnt = self.deg_per_s_to_counts(self.cfg.profile_accel_deg_s2)
        decel_cnt = self.deg_per_s_to_counts(self.cfg.profile_decel_deg_s2)
        self.node.sdo[0x6081].raw = velocity_cnt
        self.node.sdo[0x6083].raw = accel_cnt
        self.node.sdo[0x6084].raw = decel_cnt

    def _configure_pdus(self) -> None:
        if self._try_remap_pdos_via_sdo():
            self.node.tpdo.read()
            self.node.rpdo.read()
            log.info("Node 0x%02X: PDOs remapped via SDO", self.cfg.node_id)
            return

        self.node.tpdo.read()
        self.node.rpdo.read()

        tpdo1 = self.node.tpdo[1]
        tpdo1.clear()
        tpdo1.add_variable("Statusword")
        tpdo1.add_variable("Position actual value")
        tpdo1.trans_type = self.cfg.tpdo_transmission_type
        tpdo1.cob_id = 0x180 + self.cfg.node_id
        tpdo1.event_timer = 0
        tpdo1.enabled = True

        rpdo1 = self.node.rpdo[1]
        rpdo1.clear()
        rpdo1.add_variable("Controlword")
        rpdo1.add_variable("Target Position")
        rpdo1.trans_type = self.cfg.rpdo_transmission_type
        rpdo1.cob_id = 0x200 + self.cfg.node_id
        rpdo1.enabled = True

        self.node.tpdo.save()
        self.node.rpdo.save()

    def _try_remap_pdos_via_sdo(self) -> bool:
        """
        尝试通过 SDO 通信来重新映射节点的 PDO (Process Data Objects) 配置。

        这个函数会执行一个标准的 PDO 配置流程：
        1. 禁用 PDO，防止在配置过程中发送不完整的数据。
        2. 清除旧的 PDO 映射。
        3. 设置新的 PDO 映射（将特定的对象字典条目映射到 PDO 中）。
        4. 重新启用 PDO。

        Returns:
            bool: 如果 PDO 重新映射成功，返回 True；如果过程中出现任何错误，返回 False。
        """
        node = self.node
        node_id = self.cfg.node_id

        # 计算标准的 CANopen COB-ID (Communication Object Identifier)
        # TPDO1 (Transmit PDO 1) 的默认功能码是 0x180
        txpdo1 = 0x180 + node_id
        # RPDO1 (Receive PDO 1) 的默认功能码是 0x200
        rxpdo1 = 0x200 + node_id

        try:
            # --- 步骤 1: 禁用 PDO ---

            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            # 0x1400 是 RPDO1 的通信参数对象索引。
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000

            # --- 步骤 2: 设置传输类型 ---
            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type

            # --- 步骤 3: 清除旧的 PDO 映射 ---
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0

            # --- 步骤 4: 配置 TPDO1 的新映射 ---
            node.sdo[0x1A00][1].raw = 0x6041_0010
            # 0x6064_0020: 映射位置实际值 (Position Actual Value, 索引 0x6064, 子索引 0x00, 长度 32 bits)
            node.sdo[0x1A00][2].raw = 0x6064_0020
            # 更新子索引 0，告诉节点现在 TPDO1 中映射了 2 个对象。
            node.sdo[0x1A00][0].raw = 2

            # --- 步骤 5: 配置 RPDO1 的新映射 ---
            node.sdo[0x1600][1].raw = 0x6040_0010
            # 0x607A_0020: 映射目标位置 (Target Position, 索引 0x607A, 子索引 0x00, 长度 32 bits)
            node.sdo[0x1600][2].raw = 0x607A_0020
            # 更新子索引 0，告诉节点现在 RPDO1 中映射了 2 个对象。
            node.sdo[0x1600][0].raw = 2

            # --- 步骤 6: 重新启用 PDO ---
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
            
            return True
        except Exception as exc:  # pragma: no cover - device specific
            log.debug(
                "Node 0x%02X: PDO remap via SDO skipped (%s)",
                node_id,
                exc,
            )
            return False
        
    def try_remap_pdos_for_velocity_mode(self) -> bool:
        """
        尝试通过 SDO 通信来重新映射节点适用于速度模式的 PDO 配置，
        并在每个关键步骤后进行验证。

        Returns:
            bool: 如果 PDO 重新映射成功，返回 True；如果过程中出现任何错误，返回 False。
        """
        node = self.node
        node_id = self.cfg.node_id
        txpdo1 = 0x180 + node_id
        rxpdo1 = 0x200 + node_id

        def _verify_sdo(index: int, subindex: int, expected_value: int, name: str) -> bool:
            """一个辅助函数，用于读取并验证 SDO 值。"""
            try:
                actual_value = node.sdo[index][subindex].raw
                if actual_value != expected_value:
                    log.error(
                        f"Node 0x{node_id:02X}: Verification failed for {name} "
                        f"(0x{index:04X}:{subindex}). Expected: 0x{expected_value:X}, Got: 0x{actual_value:X}"
                    )
                    return False
                return True
            except Exception as e:
                log.error(
                    f"Node 0x{node_id:02X}: Could not read {name} (0x{index:04X}:{subindex}) for verification: {e}"
                )
                return False

        try:
            # --- 1. 禁用 PDO (设置最高位) ---
            log.debug(f"Node 0x{node_id:02X}: Disabling PDOs for remapping.")
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000
            if not (_verify_sdo(0x1800, 1, txpdo1 | 0x8000_0000, "TPDO1 COB-ID (disabled)") and
                    _verify_sdo(0x1400, 1, rxpdo1 | 0x8000_0000, "RPDO1 COB-ID (disabled)")):
                return False

            # --- 2. 设置传输类型 ---
            log.debug(f"Node 0x{node_id:02X}: Setting PDO transmission types.")
            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type
            if not (_verify_sdo(0x1800, 2, self.cfg.tpdo_transmission_type, "TPDO1 Transmission Type") and
                    _verify_sdo(0x1400, 2, self.cfg.rpdo_transmission_type, "RPDO1 Transmission Type")):
                return False

            # --- 3. 清空映射参数 ---
            log.debug(f"Node 0x{node_id:02X}: Clearing PDO mapping parameters.")
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0
            if not (_verify_sdo(0x1A00, 0, 0, "TPDO1 Number of Mapped Objects (cleared)") and
                    _verify_sdo(0x1600, 0, 0, "RPDO1 Number of Mapped Objects (cleared)")):
                return False

            # --- 4. 配置 TPDO1 映射 ---
            log.debug(f"Node 0x{node_id:02X}: Configuring TPDO1 mapping.")
            node.sdo[0x1A00][1].raw = 0x6041_0010  # Statusword
            node.sdo[0x1A00][2].raw = 0x6064_0020  # Position Actual Value
            node.sdo[0x1A00][0].raw = 2            # Activate with 2 mapped objects
            if not (_verify_sdo(0x1A00, 1, 0x6041_0010, "TPDO1 Mapping 1 (Statusword)") and
                    _verify_sdo(0x1A00, 2, 0x6064_0020, "TPDO1 Mapping 2 (Position)") and
                    _verify_sdo(0x1A00, 0, 2, "TPDO1 Number of Mapped Objects")):
                return False

            # --- 5. 配置 RPDO1 映射 ---
            log.debug(f"Node 0x{node_id:02X}: Configuring RPDO1 mapping.")
            node.sdo[0x1600][1].raw = 0x6040_0010  # Controlword
            node.sdo[0x1600][2].raw = 0x60FF_0020  # Target Velocity
            node.sdo[0x1600][0].raw = 2            # Activate with 2 mapped objects
            if not (_verify_sdo(0x1600, 1, 0x6040_0010, "RPDO1 Mapping 1 (Controlword)") and
                    _verify_sdo(0x1600, 2, 0x60FF_0020, "RPDO1 Mapping 2 (Target Velocity)") and
                    _verify_sdo(0x1600, 0, 2, "RPDO1 Number of Mapped Objects")):
                return False

            # --- 6. 启用 PDO (清除最高位) ---
            log.debug(f"Node 0x{node_id:02X}: Enabling PDOs after successful remapping.")
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
            if not (_verify_sdo(0x1800, 1, txpdo1, "TPDO1 COB-ID (enabled)") and
                    _verify_sdo(0x1400, 1, rxpdo1, "RPDO1 COB-ID (enabled)")):
                return False

            log.info(f"Node 0x{node_id:02X}: PDO remap and verification successful.")

            try:
                node.tpdo.read()
                node.rpdo.read()
                node.rpdo[1].enabled = True
            except Exception as exc:  # pragma: no cover - defensive
                log.warning(
                    "Node 0x%02X: failed to refresh local PDO cache after remap (%s)",
                    node_id,
                    exc,
                )

            return True

        except Exception as exc:
            # 捕获任何在写入或验证过程中发生的意外错误
            log.error(
                f"Node 0x{node_id:02X}: An unexpected error occurred during PDO remapping: {exc}",
                exc_info=True # exc_info=True 会打印完整的堆栈跟踪，便于调试
            )
            return False

    def try_remap_pdos_for_torque_mode(self) -> bool:
        """
        尝试通过 SDO 通信来重新映射节点适用于扭矩模式的 PDO 配置。
        Returns:
            bool: 如果 PDO 重新映射成功，返回 True；如果过程中出现任何错误，返回 False。
        """
        node = self.node
        node_id = self.cfg.node_id
        txpdo1 = 0x180 + node_id
        rxpdo1 = 0x200 + node_id

        try:
            def _verify_sdo(index: int, subindex: int, expected_value: int, name: str) -> bool:
                try:
                    raw_value = node.sdo[index][subindex].raw
                    actual_value = int(raw_value)
                    expected_uint = expected_value & 0xFFFFFFFF
                    actual_uint = actual_value & 0xFFFFFFFF
                    if actual_uint != expected_uint:
                        log.error(
                            "Node 0x%02X: Verification failed for %s (0x%04X:%d). Expected: 0x%X, Got: 0x%X (raw=%s)",
                            node_id,
                            name,
                            index,
                            subindex,
                            expected_uint,
                            actual_uint,
                            raw_value,
                        )
                        return False
                    return True
                except Exception as exc:  # pragma: no cover - defensive
                    log.error(
                        "Node 0x%02X: Could not read %s (0x%04X:%d) for verification: %s",
                        node_id,
                        name,
                        index,
                        subindex,
                        exc,
                    )
                    return False

            log.debug(f"Node 0x{node_id:02X}: Disabling PDOs for torque remap.")
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000
            if not (
                _verify_sdo(0x1800, 1, txpdo1 | 0x8000_0000, "TPDO1 COB-ID (disabled)")
                and _verify_sdo(0x1400, 1, rxpdo1 | 0x8000_0000, "RPDO1 COB-ID (disabled)")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Setting torque PDO transmission types.")
            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type
            if not (
                _verify_sdo(0x1800, 2, self.cfg.tpdo_transmission_type, "TPDO1 Transmission Type")
                and _verify_sdo(0x1400, 2, self.cfg.rpdo_transmission_type, "RPDO1 Transmission Type")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Clearing torque PDO mapping parameters.")
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0
            if not (
                _verify_sdo(0x1A00, 0, 0, "TPDO1 Number of Mapped Objects (cleared)")
                and _verify_sdo(0x1600, 0, 0, "RPDO1 Number of Mapped Objects (cleared)")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Configuring torque TPDO mapping.")
            node.sdo[0x1A00][1].raw = 0x6041_0010
            node.sdo[0x1A00][2].raw = 0x6064_0020
            node.sdo[0x1A00][0].raw = 2
            if not (
                _verify_sdo(0x1A00, 1, 0x6041_0010, "TPDO1 Mapping 1 (Statusword)")
                and _verify_sdo(0x1A00, 2, 0x6064_0020, "TPDO1 Mapping 2 (Position)")
                and _verify_sdo(0x1A00, 0, 2, "TPDO1 Number of Mapped Objects")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Configuring torque RPDO mapping.")
            node.sdo[0x1600][1].raw = 0x6040_0010
            # 0x6071 是 16-bit，部分驱动无法接受 32-bit 映射
            node.sdo[0x1600][2].raw = 0x6071_0010
            node.sdo[0x1600][0].raw = 2
            if not (
                _verify_sdo(0x1600, 1, 0x6040_0010, "RPDO1 Mapping 1 (Controlword)")
                and _verify_sdo(0x1600, 2, 0x6071_0010, "RPDO1 Mapping 2 (Target Torque)")
                and _verify_sdo(0x1600, 0, 2, "RPDO1 Number of Mapped Objects")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Enabling PDOs after torque remap.")
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
            if not (
                _verify_sdo(0x1800, 1, txpdo1, "TPDO1 COB-ID (enabled)")
                and _verify_sdo(0x1400, 1, rxpdo1, "RPDO1 COB-ID (enabled)")
            ):
                return False

            log.info(f"Node 0x{node_id:02X}: PDO remap for torque mode successful.")

            try:
                node.tpdo.read()
                node.rpdo.read()
                node.rpdo[1].enabled = True
            except Exception as exc:  # pragma: no cover - defensive
                log.warning(
                    "Node 0x%02X: failed to refresh local PDO cache after torque remap (%s)",
                    node_id,
                    exc,
                )

            return True
        except Exception as exc:
            log.error(
                "Node 0x%02X: An unexpected error occurred during torque PDO remapping: %s",
                node_id,
                exc,
                exc_info=True,
            )
            return False

    def try_remap_pdos_for_csp_with_feedforward(self) -> bool:
        """为带速度/力矩前馈的 CSP 模式重新映射 PDO，并新增一个 RPDO 用于前馈通道。"""
        node = self.node
        node_id = self.cfg.node_id
        txpdo1 = 0x180 + node_id
        rxpdo1 = 0x200 + node_id
        rxpdo2 = 0x300 + node_id

        def _verify_sdo(index: int, subindex: int, expected_value: int, name: str) -> bool:
            try:
                actual_value = node.sdo[index][subindex].raw
                if actual_value != expected_value:
                    log.error(
                        "Node 0x%02X: Verification failed for %s (0x%04X:%d). Expected: 0x%X, Got: 0x%X",
                        node_id,
                        name,
                        index,
                        subindex,
                        expected_value,
                        actual_value,
                    )
                    return False
                return True
            except Exception as exc:  # pragma: no cover - defensive
                log.error(
                    "Node 0x%02X: Could not read %s (0x%04X:%d) for verification: %s",
                    node_id,
                    name,
                    index,
                    subindex,
                    exc,
                )
                return False

        try:
            log.debug(f"Node 0x{node_id:02X}: Disabling PDOs for CSP feedforward remap.")
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000
            node.sdo[0x1401][1].raw = rxpdo2 | 0x8000_0000
            if not (
                _verify_sdo(0x1800, 1, txpdo1 | 0x8000_0000, "TPDO1 COB-ID (disabled)")
                and _verify_sdo(0x1400, 1, rxpdo1 | 0x8000_0000, "RPDO1 COB-ID (disabled)")
                and _verify_sdo(0x1401, 1, rxpdo2 | 0x8000_0000, "RPDO2 COB-ID (disabled)")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Setting CSP transmission types.")
            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type
            node.sdo[0x1401][2].raw = self.cfg.rpdo_transmission_type
            if not (
                _verify_sdo(0x1800, 2, self.cfg.tpdo_transmission_type, "TPDO1 Transmission Type")
                and _verify_sdo(0x1400, 2, self.cfg.rpdo_transmission_type, "RPDO1 Transmission Type")
                and _verify_sdo(0x1401, 2, self.cfg.rpdo_transmission_type, "RPDO2 Transmission Type")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Clearing CSP mapping parameters.")
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0
            node.sdo[0x1601][0].raw = 0
            if not (
                _verify_sdo(0x1A00, 0, 0, "TPDO1 Number of Mapped Objects (cleared)")
                and _verify_sdo(0x1600, 0, 0, "RPDO1 Number of Mapped Objects (cleared)")
                and _verify_sdo(0x1601, 0, 0, "RPDO2 Number of Mapped Objects (cleared)")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Configuring TPDO1 mapping for CSP.")
            node.sdo[0x1A00][1].raw = 0x6041_0010
            node.sdo[0x1A00][2].raw = 0x6064_0020
            node.sdo[0x1A00][0].raw = 2
            if not (
                _verify_sdo(0x1A00, 1, 0x6041_0010, "TPDO1 Mapping 1 (Statusword)")
                and _verify_sdo(0x1A00, 2, 0x6064_0020, "TPDO1 Mapping 2 (Position)")
                and _verify_sdo(0x1A00, 0, 2, "TPDO1 Number of Mapped Objects")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Configuring RPDO1 mapping for CSP.")
            node.sdo[0x1600][1].raw = 0x6040_0010
            node.sdo[0x1600][2].raw = 0x607A_0020
            node.sdo[0x1600][0].raw = 2
            if not (
                _verify_sdo(0x1600, 1, 0x6040_0010, "RPDO1 Mapping 1 (Controlword)")
                and _verify_sdo(0x1600, 2, 0x607A_0020, "RPDO1 Mapping 2 (Target Position)")
                and _verify_sdo(0x1600, 0, 2, "RPDO1 Number of Mapped Objects")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Configuring RPDO2 mapping for feedforward.")
            node.sdo[0x1601][1].raw = 0x60B1_0020
            node.sdo[0x1601][2].raw = 0x60B2_0010
            node.sdo[0x1601][0].raw = 2
            if not (
                _verify_sdo(0x1601, 1, 0x60B1_0020, "RPDO2 Mapping 1 (Velocity Feedforward)")
                and _verify_sdo(0x1601, 2, 0x60B2_0010, "RPDO2 Mapping 2 (Torque Feedforward)")
                and _verify_sdo(0x1601, 0, 2, "RPDO2 Number of Mapped Objects")
            ):
                return False

            log.debug(f"Node 0x{node_id:02X}: Enabling PDOs after CSP feedforward remap.")
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
            node.sdo[0x1401][1].raw = rxpdo2
            if not (
                _verify_sdo(0x1800, 1, txpdo1, "TPDO1 COB-ID (enabled)")
                and _verify_sdo(0x1400, 1, rxpdo1, "RPDO1 COB-ID (enabled)")
                and _verify_sdo(0x1401, 1, rxpdo2, "RPDO2 COB-ID (enabled)")
            ):
                return False

            log.info(f"Node 0x{node_id:02X}: CSP PDO remap with feedforward successful.")

            try:
                node.tpdo.read()
                node.rpdo.read()
                node.rpdo[1].enabled = True
                node.rpdo[2].enabled = True
            except Exception as exc:  # pragma: no cover - defensive
                log.warning(
                    "Node 0x%02X: failed to refresh local PDO cache after CSP remap (%s)",
                    node_id,
                    exc,
                )

            return True
        except Exception as exc:
            log.error(
                "Node 0x%02X: An unexpected error occurred during CSP PDO remapping: %s",
                node_id,
                exc,
                exc_info=True,
            )
            return False


    def configure_sync(self, period_s: float) -> None:
        self.network.sync.stop()
        self.network.sync.cob_id = 0x80
        self.network.sync.start(period=period_s)
        log.info("SYNC started at %.3f s", period_s)

    # ---------------- 402 状态机操作 -----------------
    def clear_faults(self) -> None:
        log.info("Node 0x%02X: clearing fault", self.cfg.node_id)
        self._write_controlword(0x80)
        time.sleep(0.3)

    def enable_operation(self) -> None:
        log.info("Node 0x%02X: enable operation", self.cfg.node_id)
        now = time.monotonic()
        if self._last_disable_ts and now - self._last_disable_ts < 0.3:
            time.sleep(0.3 - (now - self._last_disable_ts))

        self._write_controlword(0x06)
        self._wait_status(0x6F, 0x21)
        time.sleep(0.3)

        self._write_controlword(0x07)
        self._wait_status(0x6F, 0x23)
        time.sleep(0.3)

        self._write_controlword(0x0F)
        self._wait_status(0x6F, 0x27)
        time.sleep(0.3)
        self._last_enable_ts = time.monotonic()

    def disable_operation(self) -> None:
        log.info("Node 0x%02X: disable operation", self.cfg.node_id)
        now = time.monotonic()
        if self._last_enable_ts and now - self._last_enable_ts < 0.3:
            time.sleep(0.3 - (now - self._last_enable_ts))

        self._write_controlword(0x07)
        time.sleep(0.3)
        self._write_controlword(0x06)
        time.sleep(0.3)
        self._last_disable_ts = time.monotonic()

    def quick_stop(self) -> None:
        log.info("Node 0x%02X: quick stop", self.cfg.node_id)
        self._write_controlword(0x02)
        self._wait_status(0x6F, 0x07)

    def clear_halt(self) -> None:
        """清除 CiA-402 控制字 0x6040 的 Halt 位（bit8），仅使用 SDO。

        读取当前 Controlword，清除 bit8 后写回；若读取失败，兜底写入 0x000F（常见的
        "Enable Operation" 且未置 Halt 的控制字）。不修改其他控制位以避免破坏当前状态。
        """
        log.info("Node 0x%02X: clear HALT (bit8) via SDO", self.cfg.node_id)
        try:
            current = int(self.node.sdo[0x6040].raw)
            new_value = current & (~0x0100 & 0xFFFF)
            # 即便值未变化，仍写一次，确保设备接收到控制字
            self.node.sdo[0x6040].raw = new_value
        except Exception:
            # 回退：尝试写入一个典型的“已使能且无 HALT”的控制字
            try:
                self.node.sdo[0x6040].raw = 0x000F
            except Exception:
                log.exception("Node 0x%02X: failed to clear HALT via SDO", self.cfg.node_id)

    def recover(
        self,
        *,
        use_reset_node: bool = False,
        target_mode: str = "PP",
        pdo_mapping: bool = True,
        ensure_sync: Optional[float] = None,
    ) -> None:
        """一键恢复：通过 NMT 复位 + 模式重建 + 清故障 + 重新上电使能。

        步骤：
        1) NMT Reset（通信或整节点）→ 等待 boot-up → PRE-OPERATIONAL
        2) （可选）PDO 重新映射；预装当前实际位置到目标位置；设置目标模式（PP/CSP）
        3) 清故障 → Enable Operation；（可选）启动 SYNC；切到 OPERATIONAL

        Args:
            use_reset_node: True 则执行 Reset Node（更接近掉电重启），False 为 Reset Communication。
            target_mode: "PP" 或 "CSP"。
            pdo_mapping: 是否尝试通过 SDO 进行 PDO 重映射。
            ensure_sync: 若给出（秒），在恢复后启动 SYNC（用于同步 PDO 的场景）。
        """
        node = self.node
        nid = self.cfg.node_id
        log.info(
            "Node 0x%02X: recovery start (reset=%s, mode=%s, pdo=%s)",
            nid,
            "node" if use_reset_node else "comm",
            target_mode,
            pdo_mapping,
        )

        # 1) NMT Reset + boot-up → PRE-OPERATIONAL
        try:
            if use_reset_node:
                node.nmt.state = 'RESET'
            else:
                node.nmt.state = 'RESET COMMUNICATION'
            try:
                node.nmt.wait_for_bootup(timeout=5.0)
            except Exception:
                log.warning("Node 0x%02X: boot-up wait skipped/timeout", nid)
            node.nmt.state = "PRE-OPERATIONAL"
            time.sleep(0.2)
        except Exception:
            log.exception("Node 0x%02X: NMT reset stage failed", nid)
            raise

        # 2) 模式/映射/预装目标
        mode = (target_mode or "PP").strip().upper()
        try:
            if pdo_mapping:
                try:
                    if not self._try_remap_pdos_via_sdo():
                        log.warning("Node 0x%02X: PDO remap skipped during recovery", nid)
                except Exception:
                    log.warning("Node 0x%02X: PDO remap raised during recovery", nid)

            # 预装当前位置为目标，避免跳变
            try:
                actual = int(node.sdo[0x6064].raw)
                node.sdo[0x607A].raw = actual
            except Exception:
                log.debug("Node 0x%02X: preload target position skipped", nid)

            if mode == "CSP":
                node.sdo[0x6060].raw = 0x08
                self._wait_mode(0x08)
            else:
                node.sdo[0x6060].raw = 0x01
                self._wait_mode(0x01)
        except Exception:
            log.exception("Node 0x%02X: mode/setup stage failed", nid)
            raise

        # 3) 清故障 + 上电使能 + OPERATIONAL
        try:
            self.clear_faults()
            time.sleep(0.3)
            self.enable_operation()
            if ensure_sync is not None:
                try:
                    self.configure_sync(float(ensure_sync))
                except Exception:
                    log.warning("Node 0x%02X: ensure SYNC failed/ignored", nid)
            node.nmt.state = "OPERATIONAL"
            time.sleep(0.2)
        except Exception:
            log.exception("Node 0x%02X: enable/operational stage failed", nid)
            raise

        log.info("Node 0x%02X: recovery completed", nid)

    def _write_controlword(self, value: int) -> None:
        try:
            rpdo1 = self.node.rpdo[1]
            rpdo1["Controlword"].raw = value
            rpdo1.transmit()
        except KeyError:
            self.node.sdo[0x6040].raw = value

    def _wait_status(self, mask: int, expected: int, timeout: float = 2.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = int(self.node.sdo[0x6041].raw)
            if status & mask == expected:
                return
            time.sleep(0.05)
        raise TimeoutError(f"Node 0x{self.cfg.node_id:02X}: status wait timed out")

    def _wait_mode(self, expected: int, timeout: float = 1.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            mode = int(self.node.sdo[0x6061].raw)
            if mode == expected:
                return
            time.sleep(0.05)
        raise TimeoutError(f"Node 0x{self.cfg.node_id:02X}: mode switch timed out")

    # ---------------- 运动控制接口 -----------------
    def set_profile(self, velocity_deg_s: float, accel_deg_s2: float, decel_deg_s2: float) -> None:
        self.cfg.profile_velocity_deg_s = velocity_deg_s
        self.cfg.profile_accel_deg_s2 = accel_deg_s2
        self.cfg.profile_decel_deg_s2 = decel_deg_s2
        self._configure_profile()

    def read_control_parameters(self) -> ControllerParameters:
        node = self.node

        def _try_read_int(index: int, subindex: int) -> int | None:
            try:
                return int(node.sdo[index][subindex].raw)
            except (SdoAbortedError, ObjectDictionaryError, KeyError, ValueError) as exc:
                log.warning(
                    "Node 0x%02X: 读取 SDO %04X:%02X 失败，跳过。原因: %s",
                    self.cfg.node_id,
                    index,
                    subindex,
                    exc,
                )
                return None

        def _try_read_bool(index: int, subindex: int) -> bool | None:
            v = _try_read_int(index, subindex)
            return None if v is None else bool(v)

        params = ControllerParameters(
            position_kp=_try_read_int(0x2382, 0x01),
            velocity_kp=_try_read_int(0x2381, 0x01),
            velocity_ki=_try_read_int(0x2381, 0x02),
            current_kp=_try_read_int(0x2380, 0x01),
            current_ki=_try_read_int(0x2380, 0x02),
            velocity_feedforward=_try_read_int(0x2382, 0x03),
            integral_limit_ma=_try_read_int(0x3000, 0x00),
            pid_switch=_try_read_bool(0x2383, 0x00),
        )
        return params

    def write_control_parameters(
        self,
        params: ControllerParameters,
        *,
        verify: bool = True,
    ) -> None:
        node = self.node

        mapping: list[tuple[str, tuple[int, int], int, int, bool]] = [
            ("position_kp", (0x2382, 0x01), 0x0000, 0xFFFF, False),
            ("velocity_kp", (0x2381, 0x01), 0x0000, 0xFFFF, False),
            ("velocity_ki", (0x2381, 0x02), 0x0000, 0xFFFF, False),
            ("current_kp", (0x2380, 0x01), 0x0000, 0xFFFF, False),
            ("current_ki", (0x2380, 0x02), 0x0000, 0xFFFF, False),
            ("velocity_feedforward", (0x2382, 0x03), 0x0000, 0xFFFF, False),
            ("integral_limit_ma", (0x3000, 0x00), 0x00000000, 0x00001F40, False),
            ("pid_switch", (0x2383, 0x00), 0x0000, 0x0001, True),
        ]

        for field_name, (index, subindex), min_val, max_val, is_bool in mapping:
            value = getattr(params, field_name)
            if value is None:
                continue
            if is_bool:
                raw_value = 1 if value else 0
            else:
                raw_value = int(value)
            if raw_value < min_val or raw_value > max_val:
                raise ValueError(
                    f"{field_name} 超出合法范围: {raw_value} not in [{min_val}, {max_val}]"
                )
            node.sdo[index][subindex].raw = raw_value

        if verify:
            time.sleep(0.1)
            current = self.read_control_parameters()
            for field_name, *_ in mapping:
                expected = getattr(params, field_name)
                if expected is None:
                    continue
                actual = getattr(current, field_name)
                if expected is True or expected is False:
                    cmp_expected = bool(expected)
                    cmp_actual = bool(actual)
                else:
                    cmp_expected = expected
                    cmp_actual = actual
                if cmp_actual != cmp_expected:
                    raise RuntimeError(
                        f"写入 {field_name} 失败: 预期 {cmp_expected}, 实际 {cmp_actual}"
                    )

    def set_target_counts(self, counts: int, is_csp: bool=False) -> None:
        log.debug("Node 0x%02X: target counts %d", self.cfg.node_id, counts)
        try:
            rpdo1 = self.node.rpdo[1]
            try:
                target_var = rpdo1["Target Position"]
            except KeyError:
                target_var = rpdo1.get_variable(0x607A, 0)
            try:
                control_var = rpdo1["Controlword"]
            except KeyError:
                control_var = rpdo1.get_variable(0x6040, 0)

            if is_csp:
                target_var.raw = counts
                control_var.raw = 0x1F
                rpdo1.transmit()
            else:
                # 保证每次都有 New Set-Point 位的上升沿，避免部分驱动忽略新目标
                control_var.raw = 0x0F
                rpdo1.transmit()
                target_var.raw = counts
                control_var.raw = 0x1F
                rpdo1.transmit()
        except (KeyError, AttributeError, ValueError):
            self.node.sdo[0x607A].raw = counts
            if is_csp:
                self.node.sdo[0x6040].raw = 0x1F
            else:
                self.node.sdo[0x6040].raw = 0x0F
                self.node.sdo[0x6040].raw = 0x1F
            log.warning("使用 SDO 方式下发目标位置，性能较差，建议检查 PDO 配置")

    def set_target_angle(self, angle_deg: float, is_csp: bool=False) -> None:
        self.set_target_counts(self.angle_to_counts(angle_deg), is_csp=is_csp)

    def set_target_angle_sdo(self, angle_deg: float) -> None:
        """使用 SDO 方式下发目标位置"""
        counts = self.angle_to_counts(angle_deg)
        log.debug("Node 0x%02X: target angle %.3f deg (%d counts) via SDO", self.cfg.node_id, angle_deg, counts)
        self.node.sdo[0x607A].raw = counts
        self.node.sdo[0x6040].raw = 0x0F
        self.node.sdo[0x6040].raw = 0x1F
        
    def set_csp_target_with_feedforward(self,
        position_deg: float,
        velocity_feedforward_deg_s: float,
        torque_feedforward: int,
    ) -> None:
        """在 CSP 模式下同时下发目标位置及速度/力矩前馈。"""
        position_counts = self.angle_to_counts(position_deg)
        velocity_feedforward_counts_s = self.deg_per_s_to_counts(velocity_feedforward_deg_s)
        self.set_csp_target_counts_with_feedforward(
            position_counts,
            velocity_feedforward_counts_s,
            torque_feedforward,
        )

    def set_csp_target_counts_with_feedforward(
        self,
        position_counts: int,
        velocity_feedforward: int,
        torque_feedforward: int,
    ) -> None:
        """在 CSP 模式下同时下发目标位置及速度/力矩前馈。"""
        if torque_feedforward > 1000 or torque_feedforward < -1000:
            log.warning(
                "Node 0x%02X: feedforward torque %d outside recommended range (-1000, 1000)",
                self.cfg.node_id,
                torque_feedforward,
            )

        log.debug(
            "Node 0x%02X: CSP target=%d vel_ff=%d torque_ff=%d",
            self.cfg.node_id,
            position_counts,
            velocity_feedforward,
            torque_feedforward,
        )

        control_value = 0x001F

        try:
            rpdo_pos = self.node.rpdo[1]
            rpdo_ff = self.node.rpdo[2]

            try:  
                velocity_var = rpdo_ff["Velocity offset"]
            except KeyError:
                velocity_var = rpdo_ff.get_variable(0x60B1, 0)
            try:   
                torque_var = rpdo_ff["Torque offset"]
            except KeyError:        
                torque_var = rpdo_ff.get_variable(0x60B2, 0)

            velocity_var.raw = velocity_feedforward
            torque_var.raw = torque_feedforward
            rpdo_ff.transmit()

            try:
                position_var = rpdo_pos["Target Position"]
            except KeyError:
                position_var = rpdo_pos.get_variable(0x607A, 0)
            try:
                control_var = rpdo_pos["Controlword"]
            except KeyError:
                control_var = rpdo_pos.get_variable(0x6040, 0)

            position_var.raw = position_counts
            control_var.raw = control_value
            rpdo_pos.transmit()

            self._control_toggle = not self._control_toggle
        except (KeyError, AttributeError, ValueError) as exc:
            log.warning(
                "Node 0x%02X: CSP PDO transmit failed, falling back to SDO (%s)",
                self.cfg.node_id,
                exc,
            )
            self.node.sdo[0x607A].raw = position_counts
            self.node.sdo[0x60B1].raw = velocity_feedforward
            self.node.sdo[0x60B2].raw = torque_feedforward
            self.node.sdo[0x6040].raw = control_value
            self.node.sdo[0x6040].raw = 0x000F
            self._control_toggle = not self._control_toggle

    def set_target_velocity_counts(self, velocity_counts: int, *, halt: bool = False, is_csv: bool = False) -> None:
        """设置目标速度，单位为编码器计数/秒"""
        log.debug("Node 0x%02X: target velocity %d", self.cfg.node_id, velocity_counts)
        control_value = 0x000F | (0x0100 if halt else 0x0000)
        try:
            rpdo1 = self.node.rpdo[1]
            try:
                target_var = rpdo1["Target velocity"]
            except KeyError:
                target_var = rpdo1.get_variable(0x60FF, 0)
            try:
                control_var = rpdo1["Controlword"]
            except KeyError:
                control_var = rpdo1.get_variable(0x6040, 0)

            target_var.raw = velocity_counts
            if is_csv == False:
                control_var.raw = control_value
            else:
                control_var.raw = control_value  # CSV 模式下也带上 halt 位等控制位
            rpdo1.transmit()
        except (KeyError, AttributeError, ValueError):
            self.node.sdo[0x60FF].raw = velocity_counts
            self.node.sdo[0x6040].raw = control_value
            log.warning("使用 SDO 方式下发目标速度，性能较差，建议检查 PDO 配置")

    def set_target_velocity_deg_s(self, velocity_deg_s: float, *, halt: bool = False, is_csv: bool = False) -> None:
        """设置目标速度，单位为度/秒，halt 表示是否急停"""
        velocity_counts = self.deg_per_s_to_counts(velocity_deg_s)
        self.set_target_velocity_counts(velocity_counts, halt=halt, is_csv=is_csv)

    def set_target_velocity_deg_s_sdo(self, velocity_deg_s: float, *, halt: bool = False) -> None:
        """使用 SDO 方式下发目标速度"""
        velocity_counts = self.deg_per_s_to_counts(velocity_deg_s)
        control_value = 0x000F | (0x0100 if halt else 0x0000)
        log.debug("Node 0x%02X: target velocity %.3f deg/s (%d counts) via SDO", self.cfg.node_id, velocity_deg_s, velocity_counts)
        self.node.sdo[0x60FF].raw = velocity_counts
        self.node.sdo[0x6040].raw = control_value

    def set_target_torque(self, torque_value: int, *, halt: bool = False, is_cst: bool = False) -> None:
        if torque_value > 1000 or torque_value < -1000:
            log.warning(
                "Node 0x%02X: 目标力矩 %d 超出推荐范围 (-1000, 1000)",
                self.cfg.node_id,
                torque_value,
            )

        control_value = 0x000F | (0x0100 if halt else 0x0000)
        try:
            rpdo1 = self.node.rpdo[1]
            try:
                torque_var = rpdo1["Target torque"]
            except KeyError:
                torque_var = rpdo1.get_variable(0x6071, 0)
            try:
                control_var = rpdo1["Controlword"]
            except KeyError:
                control_var = rpdo1.get_variable(0x6040, 0)

            torque_var.raw = torque_value
            if is_cst == False:
                control_bits = control_value | (0x0010 if self._control_toggle else 0x0000)
                control_var.raw = control_bits
                self._control_toggle = not self._control_toggle
            else:
                control_var.raw = 15  # 用于 CST 模式下发控制字，避免冲突
            rpdo1.transmit()
        except (KeyError, AttributeError, ValueError):
            self.node.sdo[0x6071].raw = torque_value
            if is_cst:
                self.node.sdo[0x6040].raw = 0x000F
            else:
                toggle_value = control_value | 0x0010
                self.node.sdo[0x6040].raw = toggle_value
                self.node.sdo[0x6040].raw = control_value
            log.warning("使用 SDO 方式下发目标力矩，性能较差，建议检查 PDO 配置")

    def set_target_torque_sdo(self, torque_value: int, *, halt: bool = False) -> None:
        """使用 SDO 方式下发目标力矩"""
        if torque_value > 1000 or torque_value < -1000:
            log.warning(
                "Node 0x%02X: 目标力矩 %d 超出推荐范围 (-1000, 1000)",
                self.cfg.node_id,
                torque_value,
            )

        control_value = 0x000F | (0x0100 if halt else 0x0000)
        log.debug("Node 0x%02X: target torque %d via SDO", self.cfg.node_id, torque_value)
        self.node.sdo[0x6071].raw = torque_value
        self.node.sdo[0x6040].raw = control_value
    
    def is_target_reached(self) -> bool:
        status = int(self.node.sdo[0x6041].raw)
        return bool(status & 0x0400)

    def get_position_counts(self, *, allow_sdo_fallback: bool = True) -> int:
        pdo_error: Optional[Exception] = None
        try:
            value = int(self.node.tpdo[1]["Position actual value"].raw)
            log.debug("Node 0x%02X: read position counts %d via PDO", self.cfg.node_id, value)
            self._last_position_counts = value
            return value
        except (KeyError, AttributeError, ObjectDictionaryError, ValueError) as exc:
            pdo_error = exc

        if allow_sdo_fallback:
            try:
                value = int(self.node.sdo[0x6064].raw)
                self._last_position_counts = value
                return value
            except Exception as sdo_error:
                log.warning(
                    "Node 0x%02X: failed to read position via SDO fallback: %s",
                    self.cfg.node_id,
                    sdo_error,
                )

        if self._last_position_counts is not None:
            log.warning(
                "Node 0x%02X: PDO read failed, returning last known position",
                self.cfg.node_id,
            )
            return self._last_position_counts

        raise ValueError(
            "Failed to read position from PDO and no previous value is available."
        ) from pdo_error

    def get_position_counts_sdo(self) -> int:
        """通过 SDO 读取当前位置计数 (0x6064)。"""
        try:
            value = int(self.node.sdo[0x6064].raw)
            log.debug("Node 0x%02X: read position counts %d via SDO", self.cfg.node_id, value)
            self._last_position_counts = value
            return value
        except Exception as exc:
            raise ValueError(
                f"Node 0x{self.cfg.node_id:02X}: failed to read position via SDO"
            ) from exc

    def get_velocity_counts(self, *, allow_sdo_fallback: bool = True) -> int:
        pdo_error: Optional[Exception] = None
        try:
            value = int(self.node.tpdo[1]["Velocity actual value"].raw)
            log.debug("Node 0x%02X: read velocity counts %d via PDO", self.cfg.node_id, value)
            self._last_velocity_counts = value
            return value
        except (KeyError, AttributeError, ObjectDictionaryError, ValueError) as exc:
            pdo_error = exc

        if allow_sdo_fallback:
            try:
                value = int(self.node.sdo[0x606C].raw)
                self._last_velocity_counts = value
                return value
            except Exception as sdo_error:
                log.warning(
                    "Node 0x%02X: failed to read velocity via SDO fallback: %s",
                    self.cfg.node_id,
                    sdo_error,
                )

        if self._last_velocity_counts is not None:
            log.warning(
                "Node 0x%02X: PDO read failed, returning last known velocity",
                self.cfg.node_id,
            )
            return self._last_velocity_counts

        raise ValueError(
            "Failed to read velocity from PDO and no previous value is available."
        ) from pdo_error

    def get_velocity_counts_sdo(self) -> int:
        """通过 SDO 读取当前速度计数 (0x606C)。"""
        try:
            value = int(self.node.sdo[0x606C].raw)
            log.debug("Node 0x%02X: read velocity counts %d via SDO", self.cfg.node_id, value)
            self._last_velocity_counts = value
            return value
        except Exception as exc:
            raise ValueError(
                f"Node 0x{self.cfg.node_id:02X}: failed to read velocity via SDO"
            ) from exc

    def get_velocity_deg_s(self, *, allow_sdo_fallback: bool = True) -> float:
        velocity_counts = self.get_velocity_counts(allow_sdo_fallback=allow_sdo_fallback)
        return self.counts_to_deg_per_s(velocity_counts)

    def get_position_angle(self, *, allow_sdo_fallback: bool = True) -> float:
        counts = self.get_position_counts(allow_sdo_fallback=allow_sdo_fallback)
        return self.counts_to_angle(counts)

    def get_position_angle_sdo(self) -> float:
        """通过 SDO 读取当前位置角度。"""
        counts = self.get_position_counts_sdo()
        return self.counts_to_angle(counts)

    def get_velocity_deg_s_sdo(self) -> float:
        """通过 SDO 读取当前速度，单位为度/秒。"""
        velocity_counts = self.get_velocity_counts_sdo()
        return self.counts_to_deg_per_s(velocity_counts)

    def get_actual_torque_sdo(self) -> int:
        """通过 SDO 读取当前实际力矩 (0x6077)。"""
        try:
            return int(self.node.sdo[0x6077].raw)
        except Exception as exc:
            raise ValueError(
                f"Node 0x{self.cfg.node_id:02X}: failed to read actual torque via SDO"
            ) from exc

    def get_motor_rated_torque(self, *, raw: bool = False) -> float | int:
        """读取额定扭矩 (0x6076)，默认单位 Nm。"""
        value = int(self.node.sdo[0x6076].raw)
        if raw:
            return value
        return value / 1000.0

    def shutdown(self) -> None:
        log.info("Node 0x%02X: shutdown", self.cfg.node_id)
        try:
            self.disable_operation()
        finally:
            if self.cfg.sync_period_s is not None:
                try:
                    self.network.sync.stop()
                except Exception:  # pragma: no cover - defensive
                    log.exception("Node 0x%02X: failed to stop SYNC", self.cfg.node_id)

    # ---------------- 模式切换工具 -----------------
    def switch_to_profile_velocity_mode(self, pdo_mapping: bool=True) -> None:
        """从当前模式切换到 CiA-402 轮廓速度模式（PV）。pdo_mapping 指示是否进行 PDO 重映射。"""
        log.info("Node 0x%02X: switching to PV mode", self.cfg.node_id)

        try:
            self.disable_operation()
        except Exception:
            log.exception(
                "Node 0x%02X: failed to disable operation before PV switch",
                self.cfg.node_id,
            )
            raise
        if pdo_mapping:
            if not self.try_remap_pdos_for_velocity_mode():
                log.warning(
                    "Node 0x%02X: PV PDO remap skipped, keeping previous mapping",
                    self.cfg.node_id,
                )
                raise RuntimeError("PV 模式 PDO 重映射失败")
        else:
            log.info(
                "Node 0x%02X: PV PDO remap skipped as per caller request",
                self.cfg.node_id,
            )

        try:
            self.node.sdo[0x60FF].raw = 0
        except Exception as exc:  # pragma: no cover - 设备特定
            log.exception(
                "Node 0x%02X: failed to preload zero target velocity",
                self.cfg.node_id,
            )
            raise RuntimeError("切换 PV 前写入 0 速度失败") from exc

        try:
            self.node.sdo[0x6060].raw = 0x03
            self._wait_mode(0x03)
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed during PV mode selection",
                self.cfg.node_id,
            )
            raise RuntimeError("切换至 PV 模式失败") from exc

        try:
            self.clear_faults()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to clear faults after PV switch",
                self.cfg.node_id,
            )
            raise RuntimeError("PV 模式清故障失败") from exc

        time.sleep(0.3)

        try:
            self.enable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to re-enable operation in PV",
                self.cfg.node_id,
            )
            raise RuntimeError("PV 模式启用失败") from exc

    def switch_to_profile_torque_mode(self, initial_torque: int, pdo_mapping: bool=True) -> None:
        """切换到 CiA-402 轮廓扭矩模式（PT），并预载入初始扭矩。pdo_mapping 指示是否进行 PDO 重映射。"""
        log.info("Node 0x%02X: switching to PT mode", self.cfg.node_id)

        if initial_torque is None:
            raise ValueError("切换 PT 模式时必须提供 initial_torque")

        try:
            self.disable_operation()
        except Exception:
            log.exception(
                "Node 0x%02X: failed to disable operation before PT switch",
                self.cfg.node_id,
            )
            raise
        if pdo_mapping:
            if not self.try_remap_pdos_for_torque_mode():
                log.warning(
                    "Node 0x%02X: PT PDO remap skipped, keeping previous mapping",
                    self.cfg.node_id,
                )
                raise RuntimeError("PT 模式 PDO 重映射失败")
        else:
            log.info(
                "Node 0x%02X: PT PDO remap skipped as per caller request",
                self.cfg.node_id,
            )

        try:
            self.node.sdo[0x6071].raw = initial_torque
        except Exception as exc:  # pragma: no cover - 设备特定
            log.exception(
                "Node 0x%02X: failed to preload initial torque %d",
                self.cfg.node_id,
                initial_torque,
            )
            raise RuntimeError("配置 PT 初始扭矩失败") from exc

        try:
            self.node.sdo[0x6060].raw = 0x04
            self._wait_mode(0x04)
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed during PT mode selection",
                self.cfg.node_id,
            )
            raise RuntimeError("切换至 PT 模式失败") from exc

        try:
            self.clear_faults()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to clear faults after PT switch",
                self.cfg.node_id,
            )
            raise RuntimeError("PT 模式清故障失败") from exc

        time.sleep(0.3)

        try:
            self.enable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to re-enable operation in PT",
                self.cfg.node_id,
            )
            raise RuntimeError("PT 模式启用失败") from exc

        try:
            rpdo1 = self.node.rpdo[1]
            try:
                torque_var = rpdo1["Target torque"]
            except KeyError:
                torque_var = rpdo1.get_variable(0x6071, 0)
            try:
                control_var = rpdo1["Controlword"]
            except KeyError:
                control_var = rpdo1.get_variable(0x6040, 0)

            torque_var.raw = initial_torque
            control_var.raw = 0x000F
            rpdo1.transmit()
        except Exception as exc:
            log.warning(
                "Node 0x%02X: failed to push initial torque via PDO (%s); SDO preload still active",
                self.cfg.node_id,
                exc,
            )

    def switch_to_cyclic_synchronous_velocity(self) -> None:
        """切换到 CiA-402 周期同步速度模式（CSV）。"""
        log.info("Node 0x%02X: switching to CSV mode", self.cfg.node_id)

        # if sync_period_ms is None:
        #     if self.cfg.sync_period_s is None:
        #         raise ValueError("sync_period_ms 未指定且配置中也未提供同步周期")
        #     sync_period_ms = self.cfg.sync_period_s * 1000.0

        # try:
        #     period_value = int(sync_period_ms)
        # except (TypeError, ValueError) as exc:  # pragma: no cover - 参数校验
        #     raise ValueError(f"非法的同步周期数值: {sync_period_ms}") from exc

        # try:
        #     self.node.sdo[0x60C2][1].raw = period_value
        #     self.node.sdo[0x60C2][2].raw = -3
        # except Exception as exc:  # pragma: no cover - 设备特定
        #     log.exception(
        #         "Node 0x%02X: failed to configure CSV interpolation period",
        #         self.cfg.node_id,
        #     )
        #     raise RuntimeError("配置 CSV 插补周期失败") from exc

        time.sleep(0.2)

        try:
            self.disable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to disable operation before CSV",
                self.cfg.node_id,
            )
            raise RuntimeError("切换 CSV 前停机失败") from exc

        if not self.try_remap_pdos_for_velocity_mode():
            log.warning(
                "Node 0x%02X: CSV PDO remap skipped, keeping previous mapping",
                self.cfg.node_id,
            )
            raise RuntimeError("CSV 模式 PDO 重映射失败")
        time.sleep(0.3)
        try:
            self.node.sdo[0x60FF].raw = 0
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to preload zero target velocity for CSV",
                self.cfg.node_id,
            )
            raise RuntimeError("配置 CSV 初始速度失败") from exc

        try:
            self.node.sdo[0x6060].raw = 0x09
            self._wait_mode(0x09)
            log.info("Node 0x%02X: %d mode selected", self.cfg.node_id, self.node.sdo[0x6060].raw)
            time.sleep(0.1)
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed during CSV mode selection",
                self.cfg.node_id,
            )
            raise RuntimeError("切换至 CSV 模式失败") from exc

        try:
            self.clear_faults()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to clear faults after CSV switch",
                self.cfg.node_id,
            )
            raise RuntimeError("CSV 模式清故障失败") from exc

        time.sleep(0.3)

        try:
            self.enable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to re-enable operation in CSV",
                self.cfg.node_id,
            )
            raise RuntimeError("CSV 模式启用失败") from exc

    def switch_to_cyclic_synchronous_torque(
        self,
        initial_torque: int = 0,
    ) -> None:
        """切换到 CiA-402 周期同步扭矩模式（CST），并预载入初始扭矩。"""
        log.info("Node 0x%02X: switching to CST mode", self.cfg.node_id)

        # if sync_period_ms is None:
        #     if self.cfg.sync_period_s is None:
        #         raise ValueError("sync_period_ms 未指定且配置中也未提供同步周期")
        #     sync_period_ms = self.cfg.sync_period_s * 1000.0

        # try:
        #     period_value = int(sync_period_ms)
        # except (TypeError, ValueError) as exc:  # pragma: no cover - 参数校验
        #     raise ValueError(f"非法的同步周期数值: {sync_period_ms}") from exc

        # try:
        #     self.node.sdo[0x60C2][1].raw = period_value
        #     self.node.sdo[0x60C2][2].raw = -3
        # except Exception as exc:  # pragma: no cover - 设备特定
        #     log.exception(
        #         "Node 0x%02X: failed to configure CST interpolation period",
        #         self.cfg.node_id,
        #     )
        #     raise RuntimeError("配置 CST 插补周期失败") from exc

        time.sleep(0.2)

        try:
            self.disable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to disable operation before CST",
                self.cfg.node_id,
            )
            raise RuntimeError("切换 CST 前停机失败") from exc

        if not self.try_remap_pdos_for_torque_mode():
            log.warning(
                "Node 0x%02X: CST PDO remap skipped, keeping previous mapping",
                self.cfg.node_id,
            )
            raise RuntimeError("CST 模式 PDO 重映射失败")

        time.sleep(0.3)

        try:
            self.node.sdo[0x6071].raw = int(initial_torque)
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to preload initial torque %d for CST",
                self.cfg.node_id,
                initial_torque,
            )
            raise RuntimeError("配置 CST 初始扭矩失败") from exc

        try:
            self.node.sdo[0x6060].raw = 0x0A
            self._wait_mode(0x0A)
            log.info(
                "Node 0x%02X: %d mode selected",
                self.cfg.node_id,
                self.node.sdo[0x6060].raw,
            )
            time.sleep(0.1)
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed during CST mode selection",
                self.cfg.node_id,
            )
            raise RuntimeError("切换至 CST 模式失败") from exc

        try:
            self.clear_faults()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to clear faults after CST switch",
                self.cfg.node_id,
            )
            raise RuntimeError("CST 模式清故障失败") from exc

        time.sleep(0.3)

        try:
            self.enable_operation()
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to re-enable operation in CST",
                self.cfg.node_id,
            )
            raise RuntimeError("CST 模式启用失败") from exc

        try:
            self.set_target_torque(int(initial_torque))
        except Exception as exc:
            log.warning(
                "Node 0x%02X: failed to push initial torque via PDO after CST switch (%s)",
                self.cfg.node_id,
                exc,
            )

    def switch_to_cyclic_synchronous_position(self, feedforward_control: bool = False) -> None:
        # 切换到 CiA-402 周期同步位置模式（CSP）。
        log.info("Node 0x%02X: switching to CSP mode", self.cfg.node_id)

        # if sync_period_ms is None:
        #     if self.cfg.sync_period_s is None:
        #         raise ValueError("sync_period_ms 未指定且配置中也未提供同步周期")
        #     sync_period_ms = self.cfg.sync_period_s * 1000.0

        # try:
        #     period_value = int(sync_period_ms)
        # except (TypeError, ValueError) as exc:  # pragma: no cover - 参数校验
        #     raise ValueError(f"非法的同步周期数值: {sync_period_ms}") from exc

        # log.info("设置同步周期为 %.3f ms", sync_period_ms)

        # try:
        #     self.node.sdo[0x60C2][1].raw = period_value
        #     self.node.sdo[0x60C2][2].raw = -3
        # except Exception as exc:  # pragma: no cover - 设备特定
        #     log.exception("Node 0x%02X: failed to configure interpolation period", self.cfg.node_id)
        #     raise RuntimeError("配置插补周期失败") from exc

        time.sleep(0.2)

        try:
            self.disable_operation()
        except Exception as exc:
            log.exception("Node 0x%02X: failed to disable operation before CSP", self.cfg.node_id)
            raise RuntimeError("切换 CSP 前停机失败") from exc
        # 根据是否启用前馈控制选择不同的 PDO 重映射
        if feedforward_control == True:
            if not self.try_remap_pdos_for_csp_with_feedforward():
                log.warning(
                    "Node 0x%02X: CSP (with feedforward) PDO remap skipped, keeping previous mapping",
                    self.cfg.node_id,
                )
                raise RuntimeError("CSP (带前馈) 模式 PDO 重映射失败")
        else:
            if not self._try_remap_pdos_via_sdo():
                log.warning(
                    "Node 0x%02X: CSP PDO remap skipped, keeping previous mapping",
                    self.cfg.node_id,
                )
                raise RuntimeError("CSP 模式 PDO 重映射失败")
            time.sleep(0.3)

        try:
            actual_counts = int(self.node.sdo[0x6064].raw)
            self.node.sdo[0x607A].raw = actual_counts
            self.node.sdo[0x6060].raw = 0x08
            self._wait_mode(0x08)
        except Exception as exc:
            log.exception("Node 0x%02X: failed during CSP mode selection", self.cfg.node_id)
            raise RuntimeError("切换至 CSP 模式失败") from exc

        try:
            self.clear_faults()
        except Exception as exc:
            log.exception("Node 0x%02X: failed to clear faults after CSP switch", self.cfg.node_id)
            raise RuntimeError("CSP 模式清故障失败") from exc

        time.sleep(0.3)

        try:
            self.enable_operation()
        except Exception as exc:
            log.exception("Node 0x%02X: failed to re-enable operation in CSP", self.cfg.node_id)
            raise RuntimeError("CSP 模式启用失败") from exc

    def switch_to_profile_position_mode(self, pdo_mapping: bool=True) -> None:
        """切回 CiA-402 轮廓位置模式（PP）。pdo_mapping 指示是否进行 PDO 重映射。"""
        log.info("Node 0x%02X: switching back to PP mode", self.cfg.node_id)

        try:
            self.disable_operation()
        except Exception:
            log.exception("Node 0x%02X: failed to disable operation before PP switch", self.cfg.node_id)
            raise
        if pdo_mapping:
            if not self._try_remap_pdos_via_sdo():
                log.warning(
                    "Node 0x%02X: PP PDO remap skipped, keeping previous mapping",
                    self.cfg.node_id,
                )
                raise RuntimeError("PP 模式 PDO 重映射失败")
        else:
            log.info(
                "Node 0x%02X: PP PDO remap skipped as per caller request",
                self.cfg.node_id,
            )
        try:
            actual_counts = int(self.node.sdo[0x6064].raw)
            self.node.sdo[0x607A].raw = actual_counts
        except Exception:
            log.exception("Node 0x%02X: failed to preload target position before PP switch", self.cfg.node_id)
            raise

        self.node.sdo[0x6060].raw = 0x01
        self._wait_mode(0x01)

        self.clear_faults()
        time.sleep(0.3)
        self.enable_operation()


class SyncProducerHelper:
    """通过 SDO 配置远程节点作为 SYNC 生产者。"""

    def __init__(self, network: canopen.Network) -> None:
        self.network = network

    def _get_node(self, node_id: int) -> canopen.RemoteNode:
        try:
            node = self.network[node_id]
        except KeyError as exc:
            raise ValueError(f"未找到节点 0x{node_id:02X}") from exc
        return cast(canopen.RemoteNode, node)

    def enable_sync_producer(self, node_id: int, period_ms: int) -> None:
        node = self._get_node(node_id)
        period_us = int(period_ms) * 1000
        cobid_value = 0x40000080
        log.info(
            "Node 0x%02X: enable SYNC producer cobid=0x%08X period=%dus",
            node_id,
            cobid_value,
            period_us,
        )
        node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.1)
        node.sdo[0x1005].raw = cobid_value
        node.sdo[0x1006].raw = period_us
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)

    def disable_sync_producer(self, node_id: int) -> None:
        node = self._get_node(node_id)
        log.info("Node 0x%02X: disable SYNC producer", node_id)
        node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.1)
        node.sdo[0x1005].raw = 0x80
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)
