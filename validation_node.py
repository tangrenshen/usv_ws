#!/usr/bin/env python3
"""
validation_node.py — 感知模块量化验证工具 (v2)

对应赛题 3.1.2 基础指标：
  1) 鸟瞰图精度        —— 用已知目标真值反算BEV理论像素位置，检验该像素是否命中
  2) 周围障碍物检测精度 —— 检测结果 vs 真值话题，最近邻匹配后算平均位置误差
  3) 周围障碍物检测遗漏率 —— 真值中有多少没被匹配上

v2 改动（针对第一次实测数据发现的问题）：
  - 记录每个话题最近一次收到消息的时间戳，汇总时报告真值/检测数据的时间差，
    避免因两边不是同一时刻的数据导致数量对不上、误判为算法问题
  - 对没匹配上的目标，额外打印"最近的真值-检测距离"，用于区分
    "真的检测不到"还是"检测到了但离阈值差一点点"
  - 修复 warn(once=True) 这个非标准rclpy用法

用法（不需要colcon build，直接跑）：
    source /opt/ros/jazzy/setup.bash
    source ~/usv_ws/install/setup.bash
    python3 validation_node.py
"""

import math
import csv
import time
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


# ────────────────────────────────────────────
#  工具函数
# ────────────────────────────────────────────

def quat_to_yaw(x, y, z, w):
    """从四元数提取yaw角（弧度）"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_boat(px, py, boat_x, boat_y, boat_yaw):
    """世界坐标系点 -> 船体坐标系点"""
    dx = px - boat_x
    dy = py - boat_y
    cos_y = math.cos(boat_yaw)
    sin_y = math.sin(boat_yaw)
    x_boat = dx * cos_y + dy * sin_y
    y_boat = -dx * sin_y + dy * cos_y
    return x_boat, y_boat


def greedy_nearest_match(gt_points, det_points, max_dist):
    """
    贪心最近邻匹配。返回:
      matches=[(gt_idx, det_idx, dist), ...]
      unmatched_gt=[idx,...], unmatched_det=[idx,...]
      nearest_for_unmatched_gt: {gt_idx: 最近的检测点距离（哪怕超过阈值）}
    """
    if not gt_points or not det_points:
        nearest = {}
        return [], list(range(len(gt_points))), list(range(len(det_points))), nearest

    pairs = []
    all_dists = {}  # (i,j) -> dist，用于事后查"最近距离"（哪怕超阈值）
    for i, (gx, gy) in enumerate(gt_points):
        for j, (dx, dy) in enumerate(det_points):
            dist = math.hypot(gx - dx, gy - dy)
            all_dists[(i, j)] = dist
            if dist <= max_dist:
                pairs.append((dist, i, j))
    pairs.sort()

    matched_gt = set()
    matched_det = set()
    matches = []
    for dist, i, j in pairs:
        if i in matched_gt or j in matched_det:
            continue
        matched_gt.add(i)
        matched_det.add(j)
        matches.append((i, j, dist))

    unmatched_gt = [i for i in range(len(gt_points)) if i not in matched_gt]
    unmatched_det = [j for j in range(len(det_points)) if j not in matched_det]

    # 对每个未匹配的真值点，找全局最近的检测点距离（不受阈值限制），用于诊断
    nearest_for_unmatched_gt = {}
    for i in unmatched_gt:
        if det_points:
            best = min(all_dists[(i, j)] for j in range(len(det_points)))
            nearest_for_unmatched_gt[i] = best

    return matches, unmatched_gt, unmatched_det, nearest_for_unmatched_gt


# ────────────────────────────────────────────
#  验证节点
# ────────────────────────────────────────────

class ValidationNode(Node):

    MATCH_THRESHOLDS = {
        'buoys':   1.5,
        'boats':   4.0,
        'pillars': 1.5,
        'blocks':  2.5,
    }

    BEV_RANGE_M = 10.0
    BEV_PX = 1000

    # 真值/检测数据时间戳相差超过这个值(秒)，就在日志里提示"本轮数据可能不同步"
    STALENESS_WARN_SEC = 1.0

    def __init__(self):
        super().__init__('validation_node')

        self.boat_pose = None
        self.gt_cache = {}       # {'buoys': [(x,y),...], ...}
        self.det_cache = {}      # {'buoys': [(x,y),...], ...}
        self.gt_stamp = {}       # {'buoys': 收到时间(墙钟), ...}
        self.det_stamp = {}      # 同上
        self.bev_image = None
        self._warned_bad_encoding = False

        self.csv_path = os.path.expanduser('~/validation_report.csv')
        self._init_csv()

        self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry',
                                  self.on_odom, qos_profile_sensor_data)
        for name in ['buoys', 'boats', 'pillars', 'blocks']:
            self.create_subscription(
                PoseArray, f'/world/obstacles/{name}',
                lambda msg, n=name: self.on_ground_truth(msg, n), 10)
        for name in ['buoys', 'boats', 'pillars', 'blocks']:
            self.create_subscription(
                PoseArray, f'/world/obstacles/{name}_check',
                lambda msg, n=name: self.on_detection(msg, n), 10)
        self.create_subscription(Image, '/world/obstacles/bev_check',
                                  self.on_bev_image, qos_profile_sensor_data)

        self.create_timer(5.0, self.run_validation)
        self.get_logger().info(f'验证节点已启动(v2)，CSV报告将写入: {self.csv_path}')

    def _init_csv(self):
        is_new = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow([
                'timestamp', 'category', 'num_gt', 'num_detected', 'num_matched',
                'mean_pos_error_m', 'miss_rate', 'false_positive_rate',
                'bev_hit_rate', 'gt_det_time_gap_sec', 'median_unmatched_nearest_dist_m'
            ])
            self.csv_file.flush()

    # ── 回调 ──

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.boat_pose = (p.x, p.y, yaw)

    def on_ground_truth(self, msg: PoseArray, category: str):
        self.gt_cache[category] = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.gt_stamp[category] = time.time()

    def on_detection(self, msg: PoseArray, category: str):
        self.det_cache[category] = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.det_stamp[category] = time.time()

    def on_bev_image(self, msg: Image):
        if msg.encoding != 'bgr8':
            if not self._warned_bad_encoding:
                self.get_logger().warn(f'bev_check 编码不是bgr8，实际是{msg.encoding}，跳过本帧')
                self._warned_bad_encoding = True
            return
        try:
            import numpy as np
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.bev_image = arr
        except Exception as e:
            self.get_logger().warn(f'BEV图像解析失败: {e}')

    # ── 核心验证逻辑 ──

    def bev_pixel_for_point(self, boat_x, boat_y):
        res = (2.0 * self.BEV_RANGE_M) / self.BEV_PX
        v = (self.BEV_RANGE_M - boat_x) / res
        u = (self.BEV_RANGE_M - boat_y) / res
        return int(round(u)), int(round(v))

    def check_bev_hit(self, boat_x, boat_y):
        if self.bev_image is None:
            return None
        u, v = self.bev_pixel_for_point(boat_x, boat_y)
        h, w = self.bev_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None
        pixel = self.bev_image[v, u]
        return bool(pixel.max() > 20)

    def run_validation(self):
        if self.boat_pose is None:
            self.get_logger().warn('尚未收到 ground_truth_odometry，无法转换坐标系，跳过本轮验证')
            return

        boat_x, boat_y, boat_yaw = self.boat_pose
        timestamp = time.time()

        for category in ['buoys', 'boats', 'pillars', 'blocks']:
            gt_world = self.gt_cache.get(category, [])
            det_boat = self.det_cache.get(category, [])

            # 时序对齐诊断：真值和检测数据的到达时间差
            gt_t = self.gt_stamp.get(category)
            det_t = self.det_stamp.get(category)
            time_gap = abs(gt_t - det_t) if (gt_t and det_t) else float('nan')
            stale_flag = ''
            if not math.isnan(time_gap) and time_gap > self.STALENESS_WARN_SEC:
                gt_age = (timestamp - gt_t) if gt_t else float('nan')
                det_age = (timestamp - det_t) if det_t else float('nan')
                stale_flag = f' ⚠数据时间差{time_gap:.2f}s，本轮结果可能不可信 [gt上次更新距今{gt_age:.2f}s, det上次更新距今{det_age:.2f}s]'
                if gt_age > 10.0 or det_age > 10.0:
                    self.get_logger().warn(f'[{category}] 数据源疑似已断开(gt距今{gt_age:.1f}s/det距今{det_age:.1f}s)，跳过本轮验证')
                    continue

            gt_boat = [world_to_boat(x, y, boat_x, boat_y, boat_yaw) for x, y in gt_world]

            # 临时调试：把pillars的真值转换结果打出来，核对坐标转换本身对不对
            if category == 'pillars':
                self.get_logger().info(
                    f'[调试:pillars真值(船体系)] 船位姿=({boat_x:.1f},{boat_y:.1f},yaw={math.degrees(boat_yaw):.1f}°) '
                    f'前5个真值坐标: {[(round(x,1), round(y,1)) for x, y in gt_boat[:5]]}'
                )
                self.get_logger().info(
                    f'[调试:pillars检测(船体系)] 前5个检测坐标: {[(round(x,1), round(y,1)) for x, y in det_boat[:5]]}'
                )


            threshold = self.MATCH_THRESHOLDS[category]
            matches, unmatched_gt, unmatched_det, nearest_map = greedy_nearest_match(
                gt_boat, det_boat, threshold)

            num_gt = len(gt_boat)
            num_det = len(det_boat)
            num_matched = len(matches)

            mean_err = (sum(d for _, _, d in matches) / num_matched) if num_matched > 0 else float('nan')
            miss_rate = (1.0 - num_matched / num_gt) if num_gt > 0 else float('nan')
            fp_rate = (1.0 - num_matched / num_det) if num_det > 0 else float('nan')

            # 未匹配真值目标的"最近检测点距离"中位数——诊断是阈值问题还是真检测不到
            nearest_dists = sorted(nearest_map.values())
            median_nearest = nearest_dists[len(nearest_dists) // 2] if nearest_dists else float('nan')

            hits = [self.check_bev_hit(x, y) for x, y in gt_boat]
            valid_hits = [h for h in hits if h is not None]
            bev_hit_rate = (sum(valid_hits) / len(valid_hits)) if valid_hits else float('nan')

            diag = ''
            if not math.isnan(median_nearest):
                diag = f' | 未匹配目标最近检测点中位距离{median_nearest:.2f}m(阈值{threshold}m)'

            self.get_logger().info(
                f'[验证:{category}] 真值{num_gt} 检测{num_det} 匹配{num_matched} '
                f'| 平均误差{mean_err:.2f}m 遗漏率{miss_rate:.1%} 误检率{fp_rate:.1%} '
                f'| BEV命中率{bev_hit_rate:.1%}{diag}{stale_flag}'
            )

            self.csv_writer.writerow([
                f'{timestamp:.2f}', category, num_gt, num_det, num_matched,
                f'{mean_err:.3f}' if num_matched > 0 else '',
                f'{miss_rate:.4f}' if num_gt > 0 else '',
                f'{fp_rate:.4f}' if num_det > 0 else '',
                f'{bev_hit_rate:.4f}' if valid_hits else '',
                f'{time_gap:.3f}' if not math.isnan(time_gap) else '',
                f'{median_nearest:.3f}' if not math.isnan(median_nearest) else '',
            ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()