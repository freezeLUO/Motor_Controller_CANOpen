import math
from typing import List, Sequence

import matplotlib.pyplot as plt

SAMPLE_PERIOD_S = 0.01  # 假设的采样周期
TRAJECTORY_DURATION_S = 12.0  # 假设的轨迹持续时间


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


def plot_trajectory(trajectory: List[float]):
    plt.figure(figsize=(10, 6))
    plt.plot(trajectory, marker='.', markersize=0.5, linestyle='-', color='b')
    plt.title('Quintic Trajectory')
    plt.xlabel('Time Step')
    plt.ylabel('Position (deg)')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    trajectory = plan_quintic_trajectory()
    plot_trajectory(trajectory)