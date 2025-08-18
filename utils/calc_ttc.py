import rosbag
import numpy as np
import pandas as pd
from geometry_msgs.msg import Pose2D
from ccavt.msg import ShareInfo  # 반드시 본인 패키지명으로 변경
import math

from scipy.interpolate import interp1d
import numpy as np

def interpolate_path(path, resolution=0.05):
    """
    Path[] 메시지를 받아 일정 간격으로 보간된 (x, y) 좌표 리스트를 반환
    path[i]는 ccavt/Path 메시지 구조임
    """
    xs = [p.pose.x for p in path]
    ys = [p.pose.y for p in path]
    
    if len(xs) < 2:
        return list(zip(xs, ys))  # 보간 불가 시 원본 반환

    # 누적 거리 계산
    dists = [0]
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        dists.append(dists[-1] + math.hypot(dx, dy))
    dists = np.array(dists)

    # 선형 보간 함수 생성
    fx = interp1d(dists, xs, kind='linear')
    fy = interp1d(dists, ys, kind='linear')

    total_dist = dists[-1]
    num_points = int(total_dist / resolution)
    interp_dists = np.linspace(0, total_dist, num_points)

    interp_xs = fx(interp_dists)
    interp_ys = fy(interp_dists)

    return list(zip(interp_xs, interp_ys))


def point_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def is_target_on_path(ego_path, target_pos, radius=1):
    interp_path = interpolate_path(ego_path)
    for pt in interp_path:
        if point_distance(pt, target_pos) <= radius:
            return True
    return False

def compute_ttc(ego_pose, ego_vel, target_pose, target_vel):
    rel_pos = np.array([target_pose[0] - ego_pose[0], target_pose[1] - ego_pose[1]])
    rel_vel = np.array([target_vel[0] - ego_vel[0], target_vel[1] - ego_vel[1]])
    rel_speed = np.dot(rel_pos, rel_vel) / np.linalg.norm(rel_pos)
    if rel_speed <= 0:
        return float('inf')  # 충돌 방향 아님
    return np.linalg.norm(rel_pos) / rel_speed

def velocity_vector(vel, theta):
    return [vel * np.cos(theta), vel * np.sin(theta)]

import bisect

def extract_timestamps(msg_list):
    return [t for t, _ in msg_list]

def find_closest_msg(msgs, target_time, max_dt=0.05):
    """
    target_time과 가장 가까운 시간의 메시지를 찾아 반환 (최대 허용 간격: max_dt초)
    """
    timestamps = extract_timestamps(msgs)
    idx = bisect.bisect_left(timestamps, target_time)
    
    candidates = []
    if 0 <= idx < len(msgs):
        candidates.append(msgs[idx])
    if 0 <= idx - 1 < len(msgs):
        candidates.append(msgs[idx - 1])
        
    best = None
    best_diff = float('inf')
    for t, msg in candidates:
        dt = abs(t - target_time)
        if dt < best_diff and dt <= max_dt:
            best = (t, msg)
            best_diff = dt
    return best


def read_and_process_bag(type, bag_path, output_csv):
    bag = rosbag.Bag(bag_path)
    results = []

    ego_msgs = []
    target_msgs = []

    for topic, msg, t in bag.read_messages(topics=[f"/{type}/EgoShareInfo", f"/{type}/TargetShareInfo"]):
        timestamp = t.to_sec()
        if topic == f"/{type}/EgoShareInfo":
            ego_msgs.append((timestamp, msg))
        elif topic == f"/{type}/TargetShareInfo":
            target_msgs.append((timestamp, msg))

    ego_msgs.sort()
    target_msgs.sort()

    # 🔍 1. state transition 시점 찾기 (0 → 1)
    state_trigger_time = None
    prev_state = None
    # for t, msg in ego_msgs:
    #     current_state = msg.state.data
    #     if prev_state == 0 and current_state == 1:
    #         state_trigger_time = t
    #         print(f"[INFO] State transition detected at {t}")
    #         break
    #     prev_state = current_state

    # if state_trigger_time is None:
    #     print("[WARN] No state transition (0→1) found.")
    #     return

    # 🔁 2. trigger 이후부터만 분석
    for i, (t_ego, ego) in enumerate(ego_msgs):
        # if t_ego < state_trigger_time:
        #     continue

        match = find_closest_msg(target_msgs, t_ego)
        if match is None:
            continue
        t_target, target = match

        ego_pos = (ego.pose.x, ego.pose.y)
        target_pos = (target.pose.x, target.pose.y)

        ego_vel = velocity_vector(ego.velocity.data, ego.pose.theta)
        target_vel = velocity_vector(target.velocity.data, target.pose.theta)

        ego_path = ego.paths
        if is_target_on_path(ego_path, target_pos):
            ttc = compute_ttc(ego_pos, ego_vel, target_pos, target_vel)
            if ttc != float('inf'):
                results.append([t_ego, ego.pose.x, ego.pose.y, ego.velocity.data, ttc])

    df = pd.DataFrame(results, columns=["time", "x", "y", "velocity", "ttc"])
    df.to_csv(output_csv, index=False)
    print(f"[INFO] Saved TTC data to {output_csv}")

# 사용 예시
bag_file = "/media/kana/Kana T7/Bag/CCAVT/avante/0416/2025-04-17-00-10-23_ETrA6-WOC.bag"
_type = 'target'
scene = 'WOC-ETrA6'
read_and_process_bag(_type, bag_file, f"/home/kana/Documents/Dataset/CCAVT/0416/ttc/{_type}/{scene}.csv")
