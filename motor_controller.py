"""基于 python-canopen 的轮廓位置模式（PP）辅助库。

封装 PP 模式常用的节点初始化、状态机控制、轮廓参数设置、目标位置下发以及
位置读取等操作，方便在上层应用中直接复用，不包含监控线程或交互逻辑。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, cast

import canopen  # type: ignore
from canopen.objectdictionary import ObjectDictionaryError  # type: ignore

log = logging.getLogger(__name__)


@dataclass(slots=True)
class PPConfig:
    """驱动初始化时使用的配置项。"""

    node_id: int
    eds_path: str = "ZeroErr Driver_V1.5.eds"
    encoder_resolution: int = 524_288
    profile_velocity_deg_s: float = 30.0
    profile_accel_deg_s2: float = 30.0
    profile_decel_deg_s2: float = 30.0
    sync_period_s: Optional[float] = None
    tpdo_transmission_type: int = 1
    rpdo_transmission_type: int = 1


class ProfilePositionController:
    """CiA-402 轮廓位置模式的高层控制器封装。"""

    def __init__(self, network: canopen.Network, config: PPConfig) -> None:
        self.network = network
        self.cfg = config
        self.node = self.network.add_node(self.cfg.node_id, self.cfg.eds_path)
        self.node.load_configuration()
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

    # def _try_remap_pdos_via_sdo(self) -> bool:
    #     node = self.node
    #     node_id = self.cfg.node_id
    #     txpdo1 = 0x180 + node_id
    #     rxpdo1 = 0x200 + node_id

    #     try:
    #         node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
    #         node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000

    #         node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
    #         node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type

    #         node.sdo[0x1A00][0].raw = 0
    #         node.sdo[0x1600][0].raw = 0

    #         node.sdo[0x1A00][1].raw = 0x6041_0010
    #         node.sdo[0x1A00][2].raw = 0x6064_0020
    #         node.sdo[0x1A00][0].raw = 2

    #         node.sdo[0x1600][1].raw = 0x6040_0010
    #         node.sdo[0x1600][2].raw = 0x607A_0020
    #         node.sdo[0x1600][0].raw = 2

    #         node.sdo[0x1800][1].raw = txpdo1
    #         node.sdo[0x1400][1].raw = rxpdo1
    #         return True
    #     except Exception as exc:  # pragma: no cover - device specific
    #         log.debug(
    #             "Node 0x%02X: PDO remap via SDO skipped (%s)",
    #             node_id,
    #             exc,
    #         )
    #         return False
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
            # 通过设置 COB-ID 的最高位 (0x80000000) 来禁用 PDO。
            # 这可以防止在配置更改期间，节点发送或接收不完整或无效的 PDO 数据。
            # 0x1800 是 TPDO1 的通信参数对象索引。
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            # 0x1400 是 RPDO1 的通信参数对象索引。
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000

            # --- 步骤 2: 设置传输类型 ---
            # 配置 PDO 的传输方式（例如：同步、异步、事件驱动等）。
            # 子索引 2 存储了传输类型。
            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type

            # --- 步骤 3: 清除旧的 PDO 映射 ---
            # 在设置新映射之前，必须先清空现有的映射。
            # 0x1A00 是 TPDO1 的映射参数对象索引。
            # 0x1600 是 RPDO1 的映射参数对象索引。
            # 子索引 0 存储了映射的对象数量。将其设为 0 即可清空所有映射。
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0

            # --- 步骤 4: 配置 TPDO1 的新映射 ---
            # 将需要从节点发送出去的数据对象映射到 TPDO1。
            # 映射格式为 0xIIIISSLL (索引, 子索引, 长度)。
            # 0x6041_0010: 映射状态字 (Status Word, 索引 0x6041, 子索引 0x00, 长度 16 bits)
            node.sdo[0x1A00][1].raw = 0x6041_0010
            # 0x6064_0020: 映射位置实际值 (Position Actual Value, 索引 0x6064, 子索引 0x00, 长度 32 bits)
            node.sdo[0x1A00][2].raw = 0x6064_0020
            # 更新子索引 0，告诉节点现在 TPDO1 中映射了 2 个对象。
            node.sdo[0x1A00][0].raw = 2

            # --- 步骤 5: 配置 RPDO1 的新映射 ---
            # 将需要发送给节点的控制对象映射到 RPDO1。
            # 0x6040_0010: 映射控制字 (Control Word, 索引 0x6040, 子索引 0x00, 长度 16 bits)
            node.sdo[0x1600][1].raw = 0x6040_0010
            # 0x607A_0020: 映射目标位置 (Target Position, 索引 0x607A, 子索引 0x00, 长度 32 bits)
            node.sdo[0x1600][2].raw = 0x607A_0020
            # 更新子索引 0，告诉节点现在 RPDO1 中映射了 2 个对象。
            node.sdo[0x1600][0].raw = 2

            # --- 步骤 6: 重新启用 PDO ---
            # 配置完成后，清除 COB-ID 的最高位，重新启用 PDO。
            # 节点现在将根据新的映射和传输类型来处理 PDO。
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
            
            return True
        except Exception as exc:  # pragma: no cover - device specific
            # 如果在上述任何 SDO 写入过程中发生错误（例如节点不响应、对象不存在等），
            # 捕获异常并记录一条调试信息。
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
            return True

        except Exception as exc:
            # 捕获任何在写入或验证过程中发生的意外错误
            log.error(
                f"Node 0x{node_id:02X}: An unexpected error occurred during PDO remapping: {exc}",
                exc_info=True # exc_info=True 会打印完整的堆栈跟踪，便于调试
            )
            return False

    # def try_remap_pdos_for_velocity_mode(self) -> bool:
    #     """
    #     尝试通过 SDO 通信来重新映射节点适用于速度模式的 PDO 配置。
    #     Returns:
    #         bool: 如果 PDO 重新映射成功，返回 True；如果过程中出现任何错误，返回 False。
    #     """
    #     node = self.node
    #     node_id = self.cfg.node_id
    #     txpdo1 = 0x180 + node_id
    #     rxpdo1 = 0x200 + node_id

    #     try:
    #         node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
    #         node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000

    #         node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
    #         node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type

    #         node.sdo[0x1A00][0].raw = 0
    #         node.sdo[0x1600][0].raw = 0

    #         node.sdo[0x1A00][1].raw = 0x6041_0010
    #         node.sdo[0x1A00][2].raw = 0x6064_0020
    #         node.sdo[0x1A00][0].raw = 2
    #         node.sdo[0x1600][1].raw = 0x6040_0010
    #         # 0x607A_0020: 映射目标速度
    #         node.sdo[0x1600][2].raw = 0x60FF_0020  # 速度模式下映射目标速度
    #         node.sdo[0x1600][0].raw = 2

    #         node.sdo[0x1800][1].raw = txpdo1
    #         node.sdo[0x1400][1].raw = rxpdo1
            
    #         return True
    #     except Exception as exc:  # pragma: no cover - device specific
    #         log.debug(
    #             "Node 0x%02X: PDO remap via SDO skipped (%s)",
    #             node_id,
    #             exc,
    #         )
    #         return False

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
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000

            node.sdo[0x1800][2].raw = self.cfg.tpdo_transmission_type
            node.sdo[0x1400][2].raw = self.cfg.rpdo_transmission_type

            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0

            node.sdo[0x1A00][1].raw = 0x6041_0010
            node.sdo[0x1A00][2].raw = 0x6064_0020
            node.sdo[0x1A00][0].raw = 2
            node.sdo[0x1600][1].raw = 0x6040_0010
            # 0x607A_0020: 映射目标力矩
            node.sdo[0x1600][2].raw = 0x6071_0020  # 扭矩模式下映射目标力矩
            node.sdo[0x1600][0].raw = 2

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

    def set_target_counts(self, counts: int) -> None:
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

            target_var.raw = counts
            # control_bits = 0x0F | (0x10 if self._control_toggle else 0x00)
            # control_var.raw = control_bits
            control_var.raw = 0x1F
            rpdo1.transmit()
            self._control_toggle = not self._control_toggle
        except (KeyError, AttributeError, ValueError):
            self.node.sdo[0x607A].raw = counts
            self.node.sdo[0x6040].raw = 0x3F
            self.node.sdo[0x6040].raw = 0x0F
            print("使用 SDO 方式下发目标位置，性能较差，建议检查 PDO 配置")

    def set_target_angle(self, angle_deg: float) -> None:
        self.set_target_counts(self.angle_to_counts(angle_deg))

    def set_target_velocity_counts(self, velocity_counts: int, *, halt: bool = False, is_csv: bool = False) -> None:
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
                control_var.raw = 15  # 用于 CSV 模式下发控制字，避免冲突
            rpdo1.transmit()
        except (KeyError, AttributeError, ValueError):
            self.node.sdo[0x60FF].raw = velocity_counts
            self.node.sdo[0x6040].raw = control_value
            print("使用 SDO 方式下发目标速度，性能较差，建议检查 PDO 配置")

    def set_target_velocity_deg_s(self, velocity_deg_s: float, *, halt: bool = False, is_csv: bool = False) -> None:
        velocity_counts = self.deg_per_s_to_counts(velocity_deg_s)
        self.set_target_velocity_counts(velocity_counts, halt=halt, is_csv=is_csv)

    def is_target_reached(self) -> bool:
        status = int(self.node.sdo[0x6041].raw)
        return bool(status & 0x0400)

    def get_position_counts(self, *, allow_sdo_fallback: bool = True) -> int:
        pdo_error: Optional[Exception] = None
        try:
            value = int(self.node.tpdo[1]["Position actual value"].raw)
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

    def get_velocity_counts(self, *, allow_sdo_fallback: bool = True) -> int:
        pdo_error: Optional[Exception] = None
        try:
            value = int(self.node.tpdo[1]["Velocity actual value"].raw)
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

    def get_velocity_deg_s(self, *, allow_sdo_fallback: bool = True) -> float:
        velocity_counts = self.get_velocity_counts(allow_sdo_fallback=allow_sdo_fallback)
        return self.counts_to_deg_per_s(velocity_counts)

    def get_position_angle(self, *, allow_sdo_fallback: bool = True) -> float:
        counts = self.get_position_counts(allow_sdo_fallback=allow_sdo_fallback)
        return self.counts_to_angle(counts)

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
    def switch_to_profile_velocity_mode(self) -> None:
        """从当前模式切换到 CiA-402 轮廓速度模式（PV）。"""
        log.info("Node 0x%02X: switching to PV mode", self.cfg.node_id)

        try:
            self.disable_operation()
        except Exception:
            log.exception(
                "Node 0x%02X: failed to disable operation before PV switch",
                self.cfg.node_id,
            )
            raise

        if not self.try_remap_pdos_for_velocity_mode():
            log.warning(
                "Node 0x%02X: PV PDO remap skipped, keeping previous mapping",
                self.cfg.node_id,
            )
            raise RuntimeError("PV 模式 PDO 重映射失败")

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

    def switch_to_cyclic_synchronous_velocity(
        self, sync_period_ms: Optional[float] = 20
    ) -> None:
        """切换到 CiA-402 周期同步速度模式（CSV）。"""
        log.info("Node 0x%02X: switching to CSV mode", self.cfg.node_id)

        if sync_period_ms is None:
            if self.cfg.sync_period_s is None:
                raise ValueError("sync_period_ms 未指定且配置中也未提供同步周期")
            sync_period_ms = self.cfg.sync_period_s * 1000.0

        try:
            period_value = int(sync_period_ms)
        except (TypeError, ValueError) as exc:  # pragma: no cover - 参数校验
            raise ValueError(f"非法的同步周期数值: {sync_period_ms}") from exc

        try:
            self.node.sdo[0x60C2][1].raw = period_value
            self.node.sdo[0x60C2][2].raw = -3
        except Exception as exc:  # pragma: no cover - 设备特定
            log.exception(
                "Node 0x%02X: failed to configure CSV interpolation period",
                self.cfg.node_id,
            )
            raise RuntimeError("配置 CSV 插补周期失败") from exc

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

    def switch_to_cyclic_synchronous_position(self, sync_period_ms: Optional[float] = 20) -> None:
        log.info("Node 0x%02X: switching to CSP mode", self.cfg.node_id)

        if sync_period_ms is None:
            if self.cfg.sync_period_s is None:
                raise ValueError("sync_period_ms 未指定且配置中也未提供同步周期")
            sync_period_ms = self.cfg.sync_period_s * 1000.0

        try:
            period_value = int(sync_period_ms)
        except (TypeError, ValueError) as exc:  # pragma: no cover - 参数校验
            raise ValueError(f"非法的同步周期数值: {sync_period_ms}") from exc

        log.info("设置同步周期为 %.3f ms", sync_period_ms)

        try:
            self.node.sdo[0x60C2][1].raw = period_value
            self.node.sdo[0x60C2][2].raw = -3
        except Exception as exc:  # pragma: no cover - 设备特定
            log.exception("Node 0x%02X: failed to configure interpolation period", self.cfg.node_id)
            raise RuntimeError("配置插补周期失败") from exc

        time.sleep(0.2)

        try:
            self.disable_operation()
        except Exception as exc:
            log.exception("Node 0x%02X: failed to disable operation before CSP", self.cfg.node_id)
            raise RuntimeError("切换 CSP 前停机失败") from exc

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

    def switch_to_profile_position_mode(self) -> None:
        """从 CSP 切回 CiA-402 轮廓位置模式（PP）。"""
        log.info("Node 0x%02X: switching back to PP mode", self.cfg.node_id)

        try:
            self.disable_operation()
        except Exception:
            log.exception("Node 0x%02X: failed to disable operation before PP switch", self.cfg.node_id)
            raise

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
