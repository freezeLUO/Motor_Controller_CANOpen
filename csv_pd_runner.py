"""CSV 模式下利用 PD 控制器进行轨迹跟踪的辅助函数。"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import List, Tuple

import can
import canopen  # type: ignore

from motor_controller import ProfilePositionController

log = logging.getLogger(__name__)


@dataclass(slots=True)
class PDGains:
    """PD 控制器增益配置。"""

    kp: float = 6.0
    kd: float = 0.3
    velocity_limit_deg_s: float = 180.0


def _clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return value
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def run_csv_pd_trajectory(
    network: canopen.Network,
    controller: ProfilePositionController,
    trajectory: List[float],
    sample_period_s: float,
    gains: PDGains | None = None,
) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
    """在 CSV 模式下执行带 PD 控制的轨迹跟踪。

    Args:
        network: CANopen 网络实例。
        controller: 已切换至 CSV 模式的控制器实例。
        trajectory: 目标角度序列，单位为度。
        sample_period_s: 每个 SYNC 周期的时长（秒）。
        gains: PD 控制器增益，缺省使用默认值。

    Returns:
        (timestamps, planned_angles, actual_angles, commanded_velocities, actual_velocities)
    """

    if gains is None:
        gains = PDGains()

    total_samples = len(trajectory)
    if total_samples == 0:
        raise ValueError("轨迹为空，无法执行 CSV 测试")

    planned: List[float] = []
    actual: List[float] = []
    commanded_velocities: List[float] = []
    actual_velocities: List[float] = []
    timestamps: List[float] = []

    done = threading.Event()
    stop_sync_worker = threading.Event()
    sample_index = 0
    prev_error = 0.0
    start_time = time.perf_counter()

    def handle_sync(_: float) -> None:
        nonlocal sample_index, prev_error
        if done.is_set():
            return
        try:
            if sample_index >= total_samples:
                controller.set_target_velocity_deg_s(0.0, halt=True)
                done.set()
                return

            target_angle = trajectory[sample_index]
            actual_angle = controller.get_position_angle()  
            print(f"当前角度: {actual_angle} deg")
            actual_velocity = controller.get_velocity_deg_s()
            print(f"当前速度: {actual_velocity} deg/s")
            error = target_angle - actual_angle
            derivative = (error - prev_error) / sample_period_s if sample_period_s > 0 else 0.0
            command_velocity = gains.kp * error + gains.kd * derivative
            command_velocity = _clamp(command_velocity, gains.velocity_limit_deg_s)
            print(f"命令速度: {command_velocity} deg/s")
            controller.set_target_velocity_deg_s(command_velocity, is_csv=True)

            timestamps.append(time.perf_counter() - start_time)
            planned.append(target_angle)
            actual.append(actual_angle)
            commanded_velocities.append(command_velocity)
            actual_velocities.append(actual_velocity)

            prev_error = error
            sample_index += 1

            if sample_index >= total_samples:
                controller.set_target_velocity_deg_s(0.0, halt=True)
                done.set()
        except Exception:
            log.exception("处理 CSV SYNC 回调时出错")
            controller.set_target_velocity_deg_s(0.0, halt=True)
            done.set()

    def sync_worker() -> None:
        bus = network.bus
        if bus is None:
            log.error("未找到底层 CAN 总线，无法监听 SYNC")
            done.set()
            return
        while not done.is_set() and not stop_sync_worker.is_set():
            try:
                msg = bus.recv(timeout=0.1)
            except can.CanError:
                log.exception("读取 SYNC 帧时发生 CAN 错误")
                controller.set_target_velocity_deg_s(0.0, halt=True)
                done.set()
                return
            if msg is None or msg.arbitration_id != 0x80:
                continue
            handle_sync(msg.timestamp)

    worker_thread = threading.Thread(target=sync_worker, name="csv-sync-listener", daemon=True)
    worker_thread.start()

    try:
        expected_runtime = total_samples * sample_period_s + 20.0
        if not done.wait(expected_runtime):
            raise TimeoutError("CSV 轨迹执行超时，可能未收到 SYNC 帧")
    finally:
        stop_sync_worker.set()
        worker_thread.join(timeout=1.0)
        controller.set_target_velocity_deg_s(0.0, halt=True)

    if len(planned) != total_samples:
        raise RuntimeError(
            f"CSV 轨迹执行未完成: 预期 {total_samples}, 实际 {len(planned)}"
        )

    return timestamps, planned, actual, commanded_velocities, actual_velocities
