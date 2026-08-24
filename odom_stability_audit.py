#!/usr/bin/env python3
import json
import math
import statistics
import sys

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


ODOM_TOPIC = "/wamv/sensors/position/ground_truth_odometry"


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * p
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def quaternion_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def unwrap(values):
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        delta = value - result[-1]
        while delta > math.pi:
            value -= 2.0 * math.pi
            delta = value - result[-1]
        while delta < -math.pi:
            value += 2.0 * math.pi
            delta = value - result[-1]
        result.append(value)
    return result


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "/home/lyf040817/bags/bag_09_03_17"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[ODOM_TOPIC]))

    samples = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != ODOM_TOPIC:
            continue
        msg = deserialize_message(data, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        roll, pitch, yaw = quaternion_to_rpy(msg.pose.pose.orientation)
        samples.append(
            (
                stamp,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                roll,
                pitch,
                yaw,
            )
        )

    if len(samples) < 2:
        raise RuntimeError(f"only {len(samples)} odometry samples found")

    times = [sample[0] for sample in samples]
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    rolls = unwrap([sample[4] for sample in samples])
    pitches = unwrap([sample[5] for sample in samples])
    yaws = unwrap([sample[6] for sample in samples])

    translation_steps = []
    yaw_steps = []
    yaw_rates = []
    for index in range(1, len(samples)):
        dt = times[index] - times[index - 1]
        translation_steps.append(
            math.sqrt(
                (xs[index] - xs[index - 1]) ** 2
                + (ys[index] - ys[index - 1]) ** 2
                + (zs[index] - zs[index - 1]) ** 2
            )
        )
        yaw_step = abs(yaws[index] - yaws[index - 1])
        yaw_steps.append(yaw_step)
        if dt > 0.0:
            yaw_rates.append(yaw_step / dt)

    rad_to_deg = 180.0 / math.pi
    report = {
        "samples": len(samples),
        "duration_sec": times[-1] - times[0],
        "translation": {
            "endpoint_displacement_m": math.sqrt(
                (xs[-1] - xs[0]) ** 2
                + (ys[-1] - ys[0]) ** 2
                + (zs[-1] - zs[0]) ** 2
            ),
            "x_range_m": max(xs) - min(xs),
            "y_range_m": max(ys) - min(ys),
            "z_range_m": max(zs) - min(zs),
            "step_median_m": statistics.median(translation_steps),
            "step_p95_m": percentile(translation_steps, 0.95),
            "step_max_m": max(translation_steps),
        },
        "attitude": {
            "roll_range_deg": (max(rolls) - min(rolls)) * rad_to_deg,
            "pitch_range_deg": (max(pitches) - min(pitches)) * rad_to_deg,
            "yaw_range_deg": (max(yaws) - min(yaws)) * rad_to_deg,
            "yaw_endpoint_change_deg": (yaws[-1] - yaws[0]) * rad_to_deg,
            "yaw_step_median_deg": statistics.median(yaw_steps) * rad_to_deg,
            "yaw_step_p95_deg": percentile(yaw_steps, 0.95) * rad_to_deg,
            "yaw_step_max_deg": max(yaw_steps) * rad_to_deg,
            "yaw_rate_p95_deg_s": percentile(yaw_rates, 0.95) * rad_to_deg,
            "yaw_rate_max_deg_s": max(yaw_rates) * rad_to_deg,
        },
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
