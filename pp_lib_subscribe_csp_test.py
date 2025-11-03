"""基于 canopen.Network.subscribe 监听 SYNC 帧的 CSP 轨迹测试脚本。

运行环境与 `pp_lib_full_test.py` 相同，只是将手动轮询总线的逻辑改为
使用 `Network.subscribe` 注册回调，由 canopen 的 Notifier 线程在收到
SYNC 帧后触发目标位置下发。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import List, Optional, Tuple

import can
import canopen  # type: ignore

from motor_controller import (  # type: ignore
    PPConfig,
    ProfilePositionController,
    SyncProducerHelper,
)
from pp_lib_full_test import (  # type: ignore
    BusConfig,
    SYNC_FREQUENCY_HZ,
    SYNC_PERIOD_MS,
    TARGET_REACHED_TIMEOUT_S,
    plan_quintic_trajectory,
    plot_results,
    wait_for_target,
)

log = logging.getLogger(__name__)


def run_csp_trajectory_via_subscribe(
    network: canopen.Network,
    controller: "ProfilePositionController",
    trajectory: List[float],
) -> Tuple[List[float], List[float], List[float]]:
    total_samples = len(trajectory)
    if total_samples == 0:
        raise ValueError("trajectory 不能为空")

    planned: List[float] = []
    actual: List[float] = []
    timestamps: List[float] = []

    done = threading.Event()
    callback_lock = threading.Lock()
    start_time = time.perf_counter()
    sample_index = 0
    callback_error: Optional[BaseException] = None

    def handle_sync(msg: can.Message) -> None:
        nonlocal sample_index, callback_error
        if done.is_set():
            return
        if msg.arbitration_id != 0x80:
            return

        with callback_lock:
            if done.is_set():
                return
            try:
                if sample_index >= total_samples:
                    done.set()
                    return

                target_angle = trajectory[sample_index]
                sample_index += 1

                controller.set_target_angle(target_angle)
                planned.append(target_angle)

                value = controller.get_position_angle(allow_sdo_fallback=False)
                actual.append(value)

                timestamps.append(time.perf_counter() - start_time)
                print(
                    f"样本 {sample_index}/{total_samples}: 目标 {target_angle:.2f}°, 实际 {value:.2f}°"
                )

                if sample_index >= total_samples:
                    done.set()
            except Exception as exc:  # pragma: no cover - 调试用
                callback_error = exc
                log.exception("处理 SYNC 回调时出错")
                done.set()

    subscription = network.subscribe(handle_sync, 0x80)

    try:
        sync_period = 1.0 / SYNC_FREQUENCY_HZ
        expected_runtime = total_samples * sync_period + 40.0
        if not done.wait(expected_runtime):
            raise TimeoutError("轨迹发送超时，可能未收到 SYNC 帧")
        if callback_error is not None:
            raise RuntimeError("SYNC 回调执行失败") from callback_error
    finally:
        if subscription is not None:
            try:
                network.unsubscribe(subscription)  # type: ignore[attr-defined]
            except AttributeError:
                try:
                    subscription.stop()  # type: ignore[attr-defined]
                except AttributeError:
                    try:
                        subscription.clear()  # type: ignore[attr-defined]
                    except AttributeError:
                        pass

    if len(planned) != total_samples:
        raise RuntimeError(
            f"轨迹执行未完成，采样数量不足: 预期 {total_samples}, 实际 {len(planned)}"
        )

    return timestamps, planned, actual


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    bus_cfg = BusConfig()

    network = canopen.Network()
    bus_kwargs = {
        "bustype": bus_cfg.bustype,
        "channel": bus_cfg.channel,
        "bitrate": bus_cfg.bitrate,
    }
    if bus_kwargs["bustype"] == "vcan":
        log.warning('检测到 bustype="vcan"，自动改用 "socketcan"')
        bus_kwargs["bustype"] = "socketcan"

    try:
        network.connect(**bus_kwargs)
    except can.CanInterfaceNotImplementedError as exc:
        log.error("CAN 接口不可用：%s", exc)
        return

    controller = ProfilePositionController(
        network,
        PPConfig(node_id=0x04, sync_period_s=None),
    )
    sync_helper = SyncProducerHelper(network)
    sync_enabled = False

    try:
        controller.initialise()
        sync_helper.enable_sync_producer(controller.cfg.node_id, SYNC_PERIOD_MS)
        sync_enabled = True

        network.sync.stop()
        network.sync.cob_id = 0x80

        controller.clear_faults()
        controller.enable_operation()

        controller.set_target_angle(0.0)
        time.sleep(3.0)
        wait_for_target(controller, 0.0, TARGET_REACHED_TIMEOUT_S)
        time.sleep(1.0)

        controller.switch_to_cyclic_synchronous_position(sync_period_ms=SYNC_PERIOD_MS)
        time.sleep(1.0)

        trajectory = plan_quintic_trajectory()
        timestamps, planned, actual = run_csp_trajectory_via_subscribe(
            network,
            controller,
            trajectory,
        )
        plot_results(timestamps, planned, actual)

    finally:
        if sync_enabled:
            try:
                sync_helper.disable_sync_producer(controller.cfg.node_id)
            except Exception:  # pragma: no cover - 防御
                log.exception("禁用 SYNC 生产者失败")
        try:
            controller.shutdown()
        except Exception:  # pragma: no cover - 防御
            log.exception("关停控制器失败")
        network.disconnect()


if __name__ == "__main__":
    main()
