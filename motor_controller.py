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
    profile_velocity_deg_s: float = 10.0
    profile_accel_deg_s2: float = 10.0
    profile_decel_deg_s2: float = 10.0
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
        self._control_toggle = True

    # ---------------- 单位换算 -----------------
    def angle_to_counts(self, angle_deg: float) -> int:
        return int(angle_deg / 360.0 * self.cfg.encoder_resolution)

    def counts_to_angle(self, counts: int) -> float:
        return counts / self.cfg.encoder_resolution * 360.0

    def deg_per_s_to_counts(self, value: float) -> int:
        return int(value / 360.0 * self.cfg.encoder_resolution)

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
            node.sdo[0x1600][2].raw = 0x607A_0020
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
    def switch_to_cyclic_synchronous_position(self, sync_period_ms: Optional[float] = 20) -> None:
        log.info("Node 0x%02X: switching to CSP mode", self.cfg.node_id)
        log.info("设置同步周期为 %.3f ms", sync_period_ms)
        self.node.sdo[0x60C2][1].raw = sync_period_ms   # 20 × 10^-3 = 0.02 s
        self.node.sdo[0x60C2][2].raw = -3
        time.sleep(0.2)
        self.disable_operation()
        actual_counts = int(self.node.sdo[0x6064].raw)
        self.node.sdo[0x607A].raw = actual_counts
        self.node.sdo[0x6060].raw = 0x08
        self._wait_mode(0x08)
        self.clear_faults()
        time.sleep(0.3)
        # if sync_period_s is not None:
        #     self.cfg.sync_period_s = sync_period_s
        #     self.configure_sync(sync_period_s)
        # elif self.cfg.sync_period_s is not None:
        #     self.configure_sync(self.cfg.sync_period_s)

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
