"""对 pp_lib 提供的主要功能进行端到端测试和演示。

步骤概览：
1. 连接总线并实例化 `ProfilePositionController`。
2. 使用 `SyncProducerHelper` 将被控电机配置为 50 Hz 的 SYNC 生产者。
3. 在 PP 模式下执行 0°→90°→0° 的往返动作。
4. 切换到 CSP 模式。
5. 规划一段周期性轨迹。
6. 订阅 SYNC 帧，在每个周期下发目标位置并读取 PDO 中的实际位置。
7. 绘制规划轨迹与实际轨迹的对比曲线。

注意：运行本脚本需要真实的 CANopen 设备以及 `matplotlib` 依赖。
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import can
import canopen  # type: ignore
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK SC"   # 名称要与字体内部名字一致
plt.rcParams["axes.unicode_minus"] = False 
from motor_controller import (  # type: ignore
    PPConfig,
    ProfilePositionController,
    SyncProducerHelper,
)
from csv_pd_runner import PDGains, run_csv_pd_trajectory

log = logging.getLogger(__name__)


@dataclass
class BusConfig:
    bustype: str = "pcan"
    channel: str = "PCAN_USBBUS1"
    bitrate: int = 1_000_000


SYNC_FREQUENCY_HZ = 100.0
SYNC_PERIOD_MS = int(1000 / SYNC_FREQUENCY_HZ)
SAMPLE_PERIOD_S = 1.0 / SYNC_FREQUENCY_HZ
TRAJECTORY_DURATION_S = 10
TRAJECTORY_AMPLITUDE_DEG = 30.0
TRAJECTORY_FREQUENCY_HZ = 0.5
TARGET_REACHED_TIMEOUT_S = 20.0


def wait_for_target(
    controller: ProfilePositionController,
    target_angle_deg: float,
    timeout: float,
    *,
    threshold_deg: float = 0.1,
    consecutive_samples: int = 5,
) -> None:
    deadline = time.monotonic() + timeout
    consecutive_within_threshold = 0
    error = float("inf")

    while time.monotonic() < deadline:
        actual_angle = controller.get_position_angle(allow_sdo_fallback=False)
        error = abs(actual_angle - target_angle_deg)
        print("actual angle:", actual_angle)

        if error <= threshold_deg:
            consecutive_within_threshold += 1
            if consecutive_within_threshold >= consecutive_samples:
                return
        else:
            consecutive_within_threshold = 0

        time.sleep(0.1)

    raise TimeoutError(
        f"等待目标位置超时：最终位置误差 {error:.3f}° (阈值 {threshold_deg}°)"
    )


def plan_sine_trajectory(base_angle: float, *, cycles: float = 1.0) -> List[float]:
    if TRAJECTORY_FREQUENCY_HZ <= 0:
        raise ValueError("TRAJECTORY_FREQUENCY_HZ 必须大于 0")
    if cycles <= 0:
        raise ValueError("正弦周期数必须大于 0")

    total_duration = cycles / TRAJECTORY_FREQUENCY_HZ
    steps = max(1, int(round(total_duration / SAMPLE_PERIOD_S)))

    trajectory: List[float] = []
    for i in range(steps):
        time_s = i * SAMPLE_PERIOD_S
        phase = 2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ * time_s
        trajectory.append(base_angle + TRAJECTORY_AMPLITUDE_DEG * math.sin(phase))

    return trajectory

def plan_linear_trajectory(base_angle: float, angle_change_deg: float = -30.0) -> List[float]:
    """
    生成一个线性变化的轨迹。

    :param base_angle: 起始角度 (度)。
    :param angle_change_deg: 在整个轨迹持续期间需要增加的总角度 (度)，默认为30.0。
    :return: 一个包含轨迹角度列表的列表。
    """
    # 1. 计算总步数 (这部分逻辑与原函数完全相同)
    steps = int(TRAJECTORY_DURATION_S / SAMPLE_PERIOD_S)
    
    # 2. 计算每一步的角度增量
    #    总变化量 / 步数 = 每一步的变化量
    angle_increment = angle_change_deg / steps
    
    trajectory = []
    for i in range(steps):
        # 3. 计算当前步骤的角度
        #    起始角度 + (当前步数 * 每步增量)
        current_angle = base_angle + i * angle_increment
        trajectory.append(current_angle)
        
    return trajectory


def _generate_quintic_segment(start_deg: float, end_deg: float, duration_s: float) -> List[float]:
    if duration_s <= 0:
        raise ValueError("段持续时间必须大于 0")

    delta = end_deg - start_deg
    t = duration_s
    a0 = start_deg
    a3 = 10.0 * delta / (t ** 3)
    a4 = -15.0 * delta / (t ** 4)
    a5 = 6.0 * delta / (t ** 5)

    segment: List[float] = []
    steps = max(1, int(math.ceil(duration_s / SAMPLE_PERIOD_S)))
    for i in range(steps):
        current_t = min(i * SAMPLE_PERIOD_S, duration_s)
        segment.append(
            a0
            + a3 * current_t**3
            + a4 * current_t**4
            + a5 * current_t**5
        )

    final_value = a0 + a3 * t**3 + a4 * t**4 + a5 * t**5
    if not segment or not math.isclose(segment[-1], final_value, abs_tol=1e-6):
        segment.append(final_value)

    return segment


def plan_quintic_trajectory(
    *,
    start_deg: float = 0.0,
    positive_deg: float = 90.0,
    negative_deg: float = -90.0,
    segment_durations_s: float | Sequence[float] | None = None,
) -> List[float]:
    if segment_durations_s is None:
        default_duration = TRAJECTORY_DURATION_S / 3.0
        durations = [default_duration] * 3
    elif isinstance(segment_durations_s, (int, float)):
        duration_value = float(segment_durations_s)
        durations = [duration_value] * 3
    else:
        durations = list(segment_durations_s)
        if len(durations) != 3:
            raise ValueError("segment_durations_s 长度必须为 3")

    if any(duration <= 0 for duration in durations):
        raise ValueError("所有段的持续时间必须大于 0")

    waypoints = [start_deg, positive_deg, negative_deg, start_deg]

    trajectory: List[float] = []
    for idx in range(3):
        segment = _generate_quintic_segment(
            waypoints[idx],
            waypoints[idx + 1],
            durations[idx],
        )
        if idx > 0 and segment:
            segment = segment[1:]
        trajectory.extend(segment)

    return trajectory


def run_csp_trajectory(
    network: canopen.Network,
    controller: "ProfilePositionController",  # 使用字符串避免循环导入
    trajectory: List[float],
) -> Tuple[List[float], List[float], List[float]]:
    total_samples = len(trajectory)
    planned: List[float] = []
    actual: List[float] = []
    timestamps: List[float] = []
    done = threading.Event()
    stop_sync_worker = threading.Event()
    start_time = time.perf_counter()
    sample_index = 0

    def handle_sync(timestamp: float) -> None:
        """在接收到 SYNC 帧时发送下一个目标点。"""
        nonlocal sample_index
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
            Value = controller.get_position_angle(allow_sdo_fallback=False)
            actual.append(Value)
            print(f"样本 {sample_index}/{total_samples}: 目标 {target_angle:.2f}°, 实际 {Value:.2f}°")
            timestamps.append(time.perf_counter() - start_time)

            if sample_index >= total_samples:
                done.set()
        except Exception:
            print("处理 SYNC 回调时出错")
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
                done.set()
                return
            if msg is None:
                continue
            if msg.arbitration_id != 0x80:
                continue
            handle_sync(msg.timestamp)

    worker_thread = threading.Thread(target=sync_worker, name="sync-listener", daemon=True)
    worker_thread.start()

    try:
        sync_period = 1.0 / SYNC_FREQUENCY_HZ
        expected_runtime = total_samples * sync_period + 40
        if not done.wait(expected_runtime):
            raise TimeoutError("轨迹发送超时，可能未收到 SYNC 帧")
    finally:
        stop_sync_worker.set()
        worker_thread.join(timeout=1.0)

    if len(planned) != total_samples:
        raise RuntimeError(f"轨迹执行未完成，采样数量不足: 预期 {total_samples}, 实际 {len(planned)}")

    return timestamps, planned, actual

def plot_results(timestamps: List[float], planned: List[float], actual: List[float]) -> None:
    if not timestamps:
        raise ValueError("没有可绘制的数据")

    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, planned, label="Planned Angle")
    plt.plot(timestamps, actual, label="Actual Angle", linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.title("CSP Trajectory Tracking Comparison")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


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
        time.sleep(1.0)
        controller.switch_to_profile_position_mode()
        # 测试PP模式
        controller.set_target_angle_sdo(20.0)
        wait_for_target(controller, 20.0, TARGET_REACHED_TIMEOUT_S)
        time.sleep(1.0)
        controller.set_target_angle(0.0,is_csp=False)
        time.sleep(3.0)
        wait_for_target(controller, 0.0, TARGET_REACHED_TIMEOUT_S)
        time.sleep(1.0)
        #---
        # # 测试 CSV 模式（PD 控制轨迹跟踪）
        # controller.switch_to_cyclic_synchronous_velocity()
        # time.sleep(1.0)
        # base_angle = controller.get_position_angle()
        # trajectory = plan_quintic_trajectory()
        # pd_gains = PDGains(kp=3.0, kd=0.1, velocity_limit_deg_s=50.0)
        # (csv_timestamps, csv_planned, csv_actual, csv_commanded_velocities, csv_actual_velocities) = run_csv_pd_trajectory(
        #     network,
        #     controller,
        #     trajectory,
        #     SAMPLE_PERIOD_S,
        #     gains=pd_gains,
        # )
        # plot_results(csv_timestamps, csv_planned, csv_actual)
        # controller.switch_to_profile_position_mode()
        # time.sleep(1.0)

        # 测试CSP模式
        # controller.switch_to_cyclic_synchronous_position()
        # time.sleep(1.0)
        # trajectory = plan_quintic_trajectory()
        # timestamps, planned, actual = run_csp_trajectory(network, controller, trajectory)
        # plot_results(timestamps, planned, actual)
        # 测试PV模式
        # controller.switch_to_profile_velocity_mode()
        # time.sleep(1.0)
        # controller.set_target_velocity_deg_s(20.0)
        # time.sleep(2.0) 
        # vel = controller.get_velocity_deg_s()
        # print(f"当前速度: {vel} deg/s")
        # pos = controller.get_position_angle(allow_sdo_fallback=False)
        # print(f"当前角度: {pos} deg")
        # time.sleep(1.0)
        # controller.set_target_velocity_deg_s(0.0)
        # time.sleep(2.0)
        # vel = controller.get_velocity_deg_s()
        # print(f"当前速度: {vel} deg/s")
        # pos = controller.get_position_angle(allow_sdo_fallback=False)
        # print(f"当前角度: {pos} deg")
        # time.sleep(1.0)
        # # 测试 PT 模式
        # controller.switch_to_profile_torque_mode(initial_torque=0)
        # time.sleep(1.0)
        # controller.set_target_torque(100, is_cst=True)
        # time.sleep(1.0)
        # tor = controller.get_actual_torque()
        # print(f"当前力矩: {tor}")
        # pos = controller.get_position_angle(allow_sdo_fallback=False)
        # print(f"当前角度: {pos} deg")
        # time.sleep(1.0)
        # controller.set_target_torque(-50, is_cst=True)
        # time.sleep(1.0)
        # tor = controller.get_actual_torque()
        # print(f"当前力矩: {tor}")
        # pos = controller.get_position_angle(allow_sdo_fallback=False)
        # print(f"当前角度: {pos} deg")


    finally:
        if sync_enabled:
            try:
                sync_helper.disable_sync_producer(controller.cfg.node_id)
            except Exception:
                log.exception("禁用 SYNC 生产者失败")
        try:
            controller.shutdown()
        except Exception:
            log.exception("关停控制器失败")
        network.disconnect()


if __name__ == "__main__":
    main()
