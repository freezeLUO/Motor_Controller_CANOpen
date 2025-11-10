from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple, Sequence

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


SYNC_FREQUENCY_HZ = 100.0                         # 同步频率
SYNC_PERIOD_MS = int(1000 / SYNC_FREQUENCY_HZ)    # 同步周期（毫秒）
SAMPLE_PERIOD_S = 1.0 / SYNC_FREQUENCY_HZ         # 采样周期（秒）
TRAJECTORY_DURATION_S = 12                        # 轨迹持续时间（秒）
TRAJECTORY_AMPLITUDE_DEG = 30.0                   # 轨迹幅度（度）
TRAJECTORY_FREQUENCY_HZ = 0.5                     # 轨迹频率（Hz）
TARGET_REACHED_TIMEOUT_S = 20.0                   # 目标到达超时时间（秒）
USE_CSP_VELOCITY_FEEDFORWARD = True              # 是否启用 CSP 速度前馈运行
USE_CSV_VELOCITY_FEEDFORWARD = False              # 是否启用 CSV 模式 PD+前馈跟踪
USE_CSV_PSEUDO_DERIVATIVE = False                 # CSV 是否启用基于位置差分的伪D

# 等待目标位置到达
def wait_for_target(
    controller: ProfilePositionController,
    target_angle_deg: float,
    timeout: float,
    *,
    threshold_deg: float = 0.1,
    consecutive_samples: int = 5,
) -> None:
    # 等待直到控制器达到目标位置，或超时
    deadline = time.monotonic() + timeout
    consecutive_within_threshold = 0
    error = float("inf")
    while time.monotonic() < deadline:
        actual_angle = controller.get_position_angle(allow_sdo_fallback=False)
        error = abs(actual_angle - target_angle_deg)
        print("Actual angle:", actual_angle)
        if error <= threshold_deg:
            consecutive_within_threshold += 1
            # 如果连续多次满足条件，则认为到达目标
            if consecutive_within_threshold >= consecutive_samples:
                return
        else:
            consecutive_within_threshold = 0
        time.sleep(0.1)
    raise TimeoutError(f"等待目标位置超时：最终位置误差 {error:.3f}° (阈值 {threshold_deg}°)")

def _generate_quintic_segment(start_deg: float, end_deg: float, duration_s: float) -> List[List[float]]:
    '''生成从 start_deg 到 end_deg 的五次多项式轨迹段，持续时间为 duration_s 秒

    返回的每个采样点包含 [角度, 角速度, 角加速度]。
    '''

    if duration_s <= 0:
        raise ValueError("段持续时间必须大于 0")

    delta = end_deg - start_deg
    t = duration_s
    a0 = start_deg
    a3 = 10.0 * delta / (t ** 3)
    a4 = -15.0 * delta / (t ** 4)
    a5 = 6.0 * delta / (t ** 5)

    segment: List[List[float]] = []
    steps = max(1, int(math.ceil(duration_s / SAMPLE_PERIOD_S)))
    for i in range(steps):
        current_t = min(i * SAMPLE_PERIOD_S, duration_s)
        position = (
            a0
            + a3 * current_t**3
            + a4 * current_t**4
            + a5 * current_t**5
        )
        velocity = (
            3.0 * a3 * current_t**2
            + 4.0 * a4 * current_t**3
            + 5.0 * a5 * current_t**4
        )
        acceleration = (
            6.0 * a3 * current_t
            + 12.0 * a4 * current_t**2
            + 20.0 * a5 * current_t**3
        )
        segment.append([position, velocity, acceleration])

    final_position = a0 + a3 * t**3 + a4 * t**4 + a5 * t**5
    final_velocity = 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4
    final_acceleration = 6.0 * a3 * t + 12.0 * a4 * t**2 + 20.0 * a5 * t**3
    if (
        not segment
        or not math.isclose(segment[-1][0], final_position, abs_tol=1e-6)
    ):
        segment.append([final_position, final_velocity, final_acceleration])
    else:
        segment[-1] = [final_position, final_velocity, final_acceleration]

    return segment

def plan_quintic_trajectory(
    *,
    start_deg: float = 0.0,
    positive_deg: float = 90.0,
    negative_deg: float = -90.0,
    segment_durations_s: float | Sequence[float] | None = None,
) -> List[List[float]]:
    '''规划一个五次多项式轨迹，包含三个段：从 start_deg 到 positive_deg，再到 negative_deg，最后回到 start_deg。

    返回值是 [[角度, 角速度, 角加速度], ...] 的列表。
    '''

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

    trajectory: List[List[float]] = []
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


def _clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return value
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


@dataclass(slots=True)
class CSVFeedforwardGains:
    kp: float = 6.0
    kd: float = 0.3
    velocity_limit_deg_s: float = 80.0


class CSVFeedforwardController:
    def __init__(self, gains: CSVFeedforwardGains) -> None:
        self.gains = gains

    def compute_command(
        self,
        *,
        target_position_deg: float,
        target_velocity_deg_s: float,
        actual_position_deg: float,
        actual_velocity_deg_s: float,
    ) -> float:
        error = target_position_deg - actual_position_deg
        velocity_error = target_velocity_deg_s - actual_velocity_deg_s
        command_velocity = (
            target_velocity_deg_s
            + self.gains.kp * error
            + self.gains.kd * velocity_error
        )
        return _clamp(command_velocity, self.gains.velocity_limit_deg_s)


def run_csv_trajectory_via_subscribe(
    network: canopen.Network,
    controller: "ProfilePositionController",
    trajectory: Sequence[Sequence[float]],
    sample_period_s: float,
    gains: CSVFeedforwardGains | None = None,
    *,
    use_pseudo_derivative: bool = False,
) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
    """Execute CSV trajectory tracking with P(+optional pseudo-D) + velocity feedforward via SYNC subscription.

    Notes:
    - Does NOT read actual velocity from the device. If `use_pseudo_derivative` is True, an estimated velocity is
      computed from position differences: (pos[k] - pos[k-1]) / sample_period_s; otherwise the derivative term is
      effectively disabled (kd treated as 0), and returned "actual_velocities" will be zeros.
    """

    if gains is None:
        gains = CSVFeedforwardGains()
    if sample_period_s <= 0:
        raise ValueError("sample_period_s 必须大于 0")

    total_samples = len(trajectory)
    if total_samples == 0:
        raise ValueError("trajectory 不能为空")

    planned: List[float] = [float(sample[0]) for sample in trajectory]
    commanded_velocities: List[float] = []
    actual_angles: List[float] = []
    actual_velocities: List[float] = []
    timestamps: List[float] = []

    # Do not rely on actual velocity; if pseudo-D disabled, force kd=0 to avoid using (target_velocity - 0)
    eff_gains = CSVFeedforwardGains(
        kp=gains.kp,
        kd=(gains.kd if use_pseudo_derivative else 0.0),
        velocity_limit_deg_s=gains.velocity_limit_deg_s,
    )
    controller_logic = CSVFeedforwardController(eff_gains)

    done = threading.Event()
    callback_lock = threading.Lock()
    start_time = time.perf_counter()
    sample_index = 0
    callback_error: Optional[BaseException] = None
    sync_counter = [0]
    last_report = [0.0]

    def handle_sync(can_id: int, data: bytes, timestamp: float) -> None:
        nonlocal sample_index, callback_error
        if done.is_set():
            return
        if can_id != 0x80:
            return

        with callback_lock:
            if done.is_set():
                return
            try:
                sync_counter[0] += 1
                if sample_index >= total_samples:
                    controller.set_target_velocity_deg_s(0.0, halt=True, is_csv=True)
                    done.set()
                    return

                try:
                    sample = trajectory[sample_index]
                    target_angle = float(sample[0])
                    target_velocity = float(sample[1])
                except (IndexError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"trajectory[{sample_index}] 缺少有效的 [角度, 速度, ...] 信息"
                    ) from exc

                try:
                    actual_angle = controller.get_position_angle()
                except Exception as exc:
                    raise RuntimeError("读取实际角度失败") from exc

                # Estimate actual velocity if enabled; otherwise keep zero (KD disabled by eff_gains)
                if use_pseudo_derivative and actual_angles:
                    prev_angle = actual_angles[-1]
                    actual_velocity = (actual_angle - prev_angle) / sample_period_s
                else:
                    actual_velocity = 0.0

                command_velocity = controller_logic.compute_command(
                    target_position_deg=target_angle,
                    target_velocity_deg_s=target_velocity,
                    actual_position_deg=actual_angle,
                    actual_velocity_deg_s=actual_velocity,
                )

                controller.set_target_velocity_deg_s(command_velocity, is_csv=True)

                elapsed = time.perf_counter() - start_time

                timestamps.append(elapsed)
                actual_angles.append(actual_angle)
                actual_velocities.append(actual_velocity)
                commanded_velocities.append(command_velocity)

                if use_pseudo_derivative:
                    log.debug(
                        f"CSV 样本 {sample_index + 1}/{total_samples}: 目标角 {target_angle:.2f}°,"
                        f" 目标速 {target_velocity:.2f}°/s, 实际角 {actual_angle:.2f}°,"
                        f" 估速 {actual_velocity:.2f}°/s, 命令速 {command_velocity:.2f}°/s"
                    )
                else:
                    log.debug(
                        f"CSV 样本 {sample_index + 1}/{total_samples}: 目标角 {target_angle:.2f}°,"
                        f" 目标速 {target_velocity:.2f}°/s, 实际角 {actual_angle:.2f}°,"
                        f" KD关, 命令速 {command_velocity:.2f}°/s"
                    )

                sample_index += 1

                if elapsed - last_report[0] >= 1.0:
                    print(
                        f"[CSV监控] 运行 {elapsed:.2f}s: 收到 SYNC {sync_counter[0]} 次, 发送速度指令 {sample_index} 次",
                    )
                    last_report[0] = elapsed

                if sample_index >= total_samples:
                    controller.set_target_velocity_deg_s(0.0, halt=True, is_csv=True)
                    done.set()
            except Exception as exc:  # pragma: no cover - 调试用
                callback_error = exc
                log.exception("处理 CSV SYNC 回调时出错")
                controller.set_target_velocity_deg_s(0.0, halt=True, is_csv=True)
                done.set()

    subscription = network.subscribe(0x80, handle_sync)

    try:
        expected_runtime = total_samples * sample_period_s + 40.0
        if not done.wait(expected_runtime):
            raise TimeoutError("CSV 轨迹执行超时，可能未收到 SYNC 帧")
        if callback_error is not None:
            raise RuntimeError("CSV SYNC 回调执行失败") from callback_error
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
        controller.set_target_velocity_deg_s(0.0, halt=True, is_csv=True)

    if sample_index != total_samples:
        raise RuntimeError(
            f"CSV 轨迹执行未完成: 预期 {total_samples}, 实际 {sample_index}"
        )

    return timestamps, planned, actual_angles, commanded_velocities, actual_velocities


def _run_csp_trajectory_via_subscribe(
    network: canopen.Network,
    controller: "ProfilePositionController",
    trajectory: Sequence[Sequence[float]],
    *,
    use_velocity_feedforward: bool,
) -> Tuple[List[float], List[float], List[float]]:
    """Shared CSP runner; when use_velocity_feedforward=True, send velocity feedforward."""

    total_samples = len(trajectory)
    if total_samples == 0:
        raise ValueError("trajectory 不能为空")

    planned: List[float] = [float(sample[0]) for sample in trajectory]
    actual: List[float] = []
    timestamps: List[float] = []

    done = threading.Event()
    callback_lock = threading.Lock()
    start_time = time.perf_counter()
    sample_index = 0
    callback_error: Optional[BaseException] = None
    sync_counter = [0]
    last_report = [0.0]

    def handle_sync(can_id: int, data: bytes, timestamp: float) -> None:
        nonlocal sample_index, callback_error
        if done.is_set():
            return
        if can_id != 0x80:
            return

        with callback_lock:
            if done.is_set():
                return
            try:
                sync_counter[0] += 1
                if sample_index >= total_samples:
                    done.set()
                    return

                try:
                    sample = trajectory[sample_index]
                    target_angle = float(sample[0])
                except (IndexError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"trajectory[{sample_index}] 不是合法的 [角度, 速度, 加速度] 样本"
                    ) from exc

                velocity_ff = 0.0
                if use_velocity_feedforward:
                    try:
                        velocity_ff = float(sample[1])
                    except (IndexError, TypeError, ValueError) as exc:
                        raise ValueError(
                            f"trajectory[{sample_index}] 缺少速度前馈分量"
                        ) from exc

                sample_index += 1

                if use_velocity_feedforward:
                    controller.set_csp_target_with_feedforward(
                        target_angle,
                        velocity_ff,
                        torque_feedforward=0,
                    )
                else:
                    controller.set_target_angle(target_angle, is_csp=True)

                value = controller.get_position_angle(allow_sdo_fallback=False)
                actual.append(value)

                elapsed = time.perf_counter() - start_time
                timestamps.append(elapsed)
                log_line = (
                    f"样本 {sample_index}/{total_samples}: 目标 {target_angle:.2f}°, 实际 {value:.2f}°"
                )
                if use_velocity_feedforward:
                    log_line += f", 速度前馈 {velocity_ff:.2f}°/s"
                log.debug(log_line)

                if elapsed - last_report[0] >= 1.0:
                    log.info(
                        f"[监控] 运行 {elapsed:.2f}s: 收到 SYNC {sync_counter[0]} 次, 发送目标 {sample_index} 次",
                    )
                    statusword = controller.get_statusword()
                    log.info(f"当前状态字: 0x{statusword:04X}")
                    error_code = controller.get_error_code()
                    log.info(f"当前错误码: 0x{error_code:04X}")
                    last_report[0] = elapsed

                if sample_index >= total_samples:
                    done.set()
            except Exception as exc:  # pragma: no cover - 调试用
                callback_error = exc
                log.exception("处理 SYNC 回调时出错")
                done.set()

    subscription = network.subscribe(0x80, handle_sync)

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

    if sample_index != total_samples:
        raise RuntimeError(
            f"轨迹执行未完成，采样数量不足: 预期 {total_samples}, 实际 {sample_index}"
        )

    return timestamps, planned, actual


def run_csp_trajectory_via_subscribe(
    network: canopen.Network,
    controller: "ProfilePositionController",
    trajectory: Sequence[Sequence[float]],
) -> Tuple[List[float], List[float], List[float]]:
    """CSP trajectory runner without feedforward; trajectory samples为 [角度, 角速度, 角加速度]."""

    return _run_csp_trajectory_via_subscribe(
        network,
        controller,
        trajectory,
        use_velocity_feedforward=False,
    )


def run_csp_trajectory_with_feedforward(
    network: canopen.Network,
    controller: "ProfilePositionController",
    trajectory: Sequence[Sequence[float]],
) -> Tuple[List[float], List[float], List[float]]:
    """CSP trajectory runner that sends velocity feedforward (torque feedforward 固定为 0)."""

    return _run_csp_trajectory_via_subscribe(
        network,
        controller,
        trajectory,
        use_velocity_feedforward=True,
    )

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


@dataclass
class BusConfig:
    bustype: str = "pcan"
    channel: str = "PCAN_USBBUS1"
    bitrate: int = 1_000_000

def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    bus_cfg = BusConfig() 
    network = canopen.Network()
    bus_kwargs = {
        "bustype": bus_cfg.bustype,
        "channel": bus_cfg.channel,
        "bitrate": bus_cfg.bitrate,
    }

    try:
        network.connect(**bus_kwargs)
    except can.CanInterfaceNotImplementedError as exc:
        log.error("CAN 接口不可用：%s", exc)
        return

    controller = ProfilePositionController(
        network,
        PPConfig(node_id=0x05, sync_period_s=None),
    )
    sync_helper = SyncProducerHelper(network) # 创建 SYNC 生产者助手
    sync_enabled = False

    try:
        controller.initialise()
        # 设置某个节点为 SYNC 生产者
        sync_helper.enable_sync_producer(controller.cfg.node_id, SYNC_PERIOD_MS)  
        sync_enabled = True

        network.sync.stop()
        network.sync.cob_id = 0x80

        controller.clear_faults()
        controller.enable_operation()

        ''' ------测试PP模式------ '''
        # controller.switch_to_profile_position_mode()
        # controller.set_target_angle(20.0)
        # wait_for_target(controller, 20.0, TARGET_REACHED_TIMEOUT_S)
        # time.sleep(1.0)
        # controller.set_target_angle(0.0)
        # time.sleep(3.0)
        # wait_for_target(controller, 0.0, TARGET_REACHED_TIMEOUT_S)
        # time.sleep(3.0)

        ''' ------测试CSV模式------ '''
        # if USE_CSV_VELOCITY_FEEDFORWARD:
        #     controller.switch_to_cyclic_synchronous_velocity()
        #     time.sleep(1.0)
        #     csv_trajectory = plan_quintic_trajectory(start_deg=0)
        #     csv_gains = CSVFeedforwardGains(kp=6.0, kd=0.2, velocity_limit_deg_s=60.0)
        #     (
        #         csv_timestamps,
        #         csv_planned,
        #         csv_actual,
        #         csv_commanded_velocities,
        #         csv_actual_velocities,
        #     ) = run_csv_trajectory_via_subscribe(
        #         network,
        #         controller,
        #         csv_trajectory,
        #         SAMPLE_PERIOD_S,
        #         gains=csv_gains,
        #         use_pseudo_derivative=USE_CSV_PSEUDO_DERIVATIVE,
        #     )
        #     plot_results(csv_timestamps, csv_planned, csv_actual)
        #     controller.switch_to_profile_position_mode()
        #     time.sleep(1.0)

        ''' ------测试CSP模式------ '''
        # if USE_CSP_VELOCITY_FEEDFORWARD:
        #     controller.switch_to_cyclic_synchronous_position(feedforward_control=True)
        # else:
        #     controller.switch_to_cyclic_synchronous_position(feedforward_control=False)
        # time.sleep(1.0)
        # trajectory = plan_quintic_trajectory(start_deg=0)
        # if USE_CSP_VELOCITY_FEEDFORWARD:
        #     timestamps, planned, actual = run_csp_trajectory_with_feedforward(
        #         network,
        #         controller,
        #         trajectory,
        #     )
        # else:
        #     timestamps, planned, actual = run_csp_trajectory_via_subscribe(
        #         network,
        #         controller,
        #         trajectory,
        #     )
        # plot_results(timestamps, planned, actual)

        ''' ------测试PV模式------ '''
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

        ''' ------测试PT模式------ '''
        controller.switch_to_profile_torque_mode(initial_torque=0,pdo_mapping=True)
        time.sleep(1.0)
        controller.set_target_torque(200)
        time.sleep(2.0)
        tor = controller.get_actual_torque_sdo()
        print(f"当前力矩: {tor}")
        pos = controller.get_position_angle()
        print(f"当前角度: {pos} deg")
        time.sleep(1.0)
        controller.set_target_torque(-400)
        time.sleep(1.0)
        tor = controller.get_actual_torque_sdo()
        print(f"当前力矩: {tor}")
        pos = controller.get_position_angle()
        print(f"当前角度: {pos} deg")
        time.sleep(3.0)


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
