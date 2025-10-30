#!/usr/bin/env python3
"""
Safety Debug Visualization Tool
경로 중첩 및 안전도 계산 시각화
"""

import json
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from pathlib import Path


class SafetyDebugVisualizer:
    def __init__(self, log_file):
        self.log_file = log_file
        self.data = []

    def load_data(self):
        """Load JSON log file"""
        try:
            with open(self.log_file, 'r') as f:
                self.data = json.load(f)
            print(f"Loaded {len(self.data)} log entries from {self.log_file}")
            return True
        except FileNotFoundError:
            print(f"Error: Log file not found: {self.log_file}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error: Failed to parse JSON: {e}")
            return False

    def filter_by_function(self, function_name):
        """Filter logs by function name"""
        return [entry for entry in self.data if entry.get('function') == function_name]

    def visualize_merge_safety(self):
        """경로 중첩 및 merge safety 시각화"""
        merge_logs = self.filter_by_function('merge_safety_calc')

        if not merge_logs:
            print("No merge_safety_calc logs found")
            return

        print(f"Visualizing {len(merge_logs)} merge safety logs...")

        # 6개 서브플롯: 경로 중첩 + 안전도
        fig = plt.figure(figsize=(20, 12))

        # 1. Path Overlap - 전체 시간대 경로 중첩 (애니메이션 효과)
        ax1 = plt.subplot(2, 3, 1)
        ax1.set_title('Path Overlap Over Time', fontsize=14, fontweight='bold')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # 시간순으로 점점 밝아지는 색으로 경로 표시
        n_samples = min(len(merge_logs), 50)  # 너무 많으면 샘플링
        sample_indices = np.linspace(0, len(merge_logs)-1, n_samples, dtype=int)

        for i, idx in enumerate(sample_indices):
            log = merge_logs[idx]
            alpha = 0.1 + 0.9 * (i / n_samples)  # 시간이 지날수록 진해짐

            # Ego path
            ego_path = log.get('ego_path', [])
            if ego_path:
                ego_x = [p[0] for p in ego_path]
                ego_y = [p[1] for p in ego_path]
                ax1.plot(ego_x, ego_y, 'b-', alpha=alpha, linewidth=1)

            # Target path
            target_path = log.get('target_path', [])
            if target_path:
                target_x = [p[0] for p in target_path]
                target_y = [p[1] for p in target_path]
                ax1.plot(target_x, target_y, 'r-', alpha=alpha, linewidth=1)

        # 마지막 프레임의 차량 위치와 교차점
        last_log = merge_logs[-1]
        ego_pose = last_log.get('ego_pose', [0, 0])
        target_pose = last_log.get('target_pose', [0, 0])
        intersection = last_log.get('intersection_point')

        ax1.plot(ego_pose[0], ego_pose[1], 'bo', markersize=12, label='Ego (latest)')
        ax1.plot(target_pose[0], target_pose[1], 'ro', markersize=12, label='Target (latest)')

        if intersection:
            ax1.plot(intersection[0], intersection[1], 'g*', markersize=20,
                    label='Intersection', markeredgecolor='black', markeredgewidth=1.5)

        ax1.legend(loc='best')

        # 2. 마지막 프레임 경로 (확대)
        ax2 = plt.subplot(2, 3, 2)
        ax2.set_title('Latest Frame Paths', fontsize=14, fontweight='bold')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')

        ego_path = last_log.get('ego_path', [])
        target_path = last_log.get('target_path', [])

        if ego_path:
            ego_x = [p[0] for p in ego_path]
            ego_y = [p[1] for p in ego_path]
            ax2.plot(ego_x, ego_y, 'b-', linewidth=2, label='Ego path')
            ax2.plot(ego_pose[0], ego_pose[1], 'bo', markersize=10)

        if target_path:
            target_x = [p[0] for p in target_path]
            target_y = [p[1] for p in target_path]
            ax2.plot(target_x, target_y, 'r-', linewidth=2, label='Target path')
            ax2.plot(target_pose[0], target_pose[1], 'ro', markersize=10)

        if intersection:
            ax2.plot(intersection[0], intersection[1], 'g*', markersize=20,
                    markeredgecolor='black', markeredgewidth=1.5, label='Intersection')

            # 교차점 주변 원 그리기
            circle = plt.Circle(intersection, 1.5, color='green', fill=False,
                              linestyle='--', linewidth=2, label='Intersection radius')
            ax2.add_patch(circle)

        ax2.legend(loc='best')

        # 3. Safety Level Over Time
        ax3 = plt.subplot(2, 3, 3)
        ax3.set_title('Safety Level Over Time', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Frame')
        ax3.set_ylabel('Safety Level')
        ax3.grid(True, alpha=0.3)

        safety_levels = [log.get('safety_level', 0) for log in merge_logs]
        frames = range(len(safety_levels))

        # 색상 매핑: 0=red, 1=green, 2=yellow
        colors = ['red' if s == 0 else 'yellow' if s == 2 else 'green' for s in safety_levels]
        ax3.scatter(frames, safety_levels, c=colors, alpha=0.6, s=20)
        ax3.set_ylim(-0.5, 2.5)
        ax3.set_yticks([0, 1, 2])
        ax3.set_yticklabels(['Unsafe (0)', 'Safe (1)', 'Emergency (2)'])

        # 4. Distance Calculations
        ax4 = plt.subplot(2, 3, 4)
        ax4.set_title('Distance Calculations', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Frame')
        ax4.set_ylabel('Distance (m)')
        ax4.grid(True, alpha=0.3)

        l_o1 = [log.get('l_o1_ego_to_inter', 0) for log in merge_logs]
        l_o2 = [log.get('l_o2_target_time_adj', 0) for log in merge_logs]
        l_o3 = [log.get('l_o3_gap_diff', 0) for log in merge_logs]
        d_TC = [log.get('d_TC_reaction_dist', 0) for log in merge_logs]

        ax4.plot(frames, l_o1, 'b-', label='l_o1 (ego→inter)', linewidth=2)
        ax4.plot(frames, l_o2, 'r-', label='l_o2 (target adj)', linewidth=2)
        ax4.plot(frames, l_o3, 'g-', label='l_o3 (gap diff)', linewidth=2)
        ax4.plot(frames, d_TC, 'orange', label='d_TC (reaction)', linewidth=2, linestyle='--')
        ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        ax4.legend(loc='best')

        # 5. Intersection Detection
        ax5 = plt.subplot(2, 3, 5)
        ax5.set_title('Intersection Detection', fontsize=14, fontweight='bold')
        ax5.set_xlabel('Frame')
        ax5.set_ylabel('Status')
        ax5.grid(True, alpha=0.3)

        intersection_found = [1 if log.get('intersection_found', False) else 0 for log in merge_logs]
        passed_inter = [1 if log.get('passed_intersection', False) else 0 for log in merge_logs]

        ax5.plot(frames, intersection_found, 'g-', label='Intersection Found', linewidth=2)
        ax5.plot(frames, passed_inter, 'b-', label='Passed Intersection', linewidth=2)
        ax5.set_ylim(-0.1, 1.1)
        ax5.set_yticks([0, 1])
        ax5.set_yticklabels(['False', 'True'])
        ax5.legend(loc='best')

        # 6. Detailed Information
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')
        ax6.set_title('Latest Frame Details', fontsize=14, fontweight='bold')

        info_text = f"""
Last Frame Information:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Ego Pose: [{ego_pose[0]:.2f}, {ego_pose[1]:.2f}]
Ego Velocity: {last_log.get('ego_velocity', 0):.2f} m/s
Target Pose: [{target_pose[0]:.2f}, {target_pose[1]:.2f}]
Target Velocity: {last_log.get('target_velocity', 0):.2f} m/s

Intersection Found: {last_log.get('intersection_found', False)}
Intersection Point: {intersection if intersection else 'None'}
Passed Intersection: {last_log.get('passed_intersection', False)}

l_o1 (ego→inter): {last_log.get('l_o1_ego_to_inter', 0):.2f} m
l_o2 (target adj): {last_log.get('l_o2_target_time_adj', 0):.2f} m
l_o3 (gap diff): {last_log.get('l_o3_gap_diff', 0):.2f} m
d_TC (reaction): {last_log.get('d_TC_reaction_dist', 0):.2f} m

Safety Level: {last_log.get('safety_level', 0)}
Confirm Safety: {last_log.get('confirm_safety', False)}
Target Signal: {last_log.get('target_signal', 0)}
        """
        ax6.text(0.05, 0.95, info_text, transform=ax6.transAxes,
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

        plt.tight_layout()

        # Save
        output_file = self.log_file.replace('.json', '_merge_safety.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved merge safety visualization to: {output_file}")

        plt.show()

    def visualize_bsd(self):
        """BSD 시각화"""
        bsd_logs = self.filter_by_function('calc_bsd')

        if not bsd_logs:
            print("No calc_bsd logs found")
            return

        print(f"Visualizing {len(bsd_logs)} BSD logs...")

        fig = plt.figure(figsize=(18, 10))

        # 1. Frenet Coordinates with BSD Zones
        ax1 = plt.subplot(2, 3, 1)
        ax1.set_title('Frenet Coordinates (ts, td)', fontsize=14, fontweight='bold')
        ax1.set_xlabel('ts (longitudinal, m)')
        ax1.set_ylabel('td (lateral, m)')
        ax1.grid(True, alpha=0.3)

        # BSD zones 그리기
        bsd_range = bsd_logs[0].get('bsd_range', [-12, 10, 1.5, 6.0])
        extended_range = bsd_logs[0].get('extended_range', [-50, 30, 1.5, 6.0])

        # Extended zone (배경)
        ext_rect = patches.Rectangle((extended_range[0], extended_range[2]),
                                     extended_range[1] - extended_range[0],
                                     extended_range[3] - extended_range[2],
                                     linewidth=2, edgecolor='orange', facecolor='orange',
                                     alpha=0.1, label='Extended Zone')
        ax1.add_patch(ext_rect)

        # Immediate BSD zone
        bsd_rect = patches.Rectangle((bsd_range[0], bsd_range[2]),
                                     bsd_range[1] - bsd_range[0],
                                     bsd_range[3] - bsd_range[2],
                                     linewidth=2, edgecolor='red', facecolor='red',
                                     alpha=0.2, label='Immediate BSD Zone')
        ax1.add_patch(bsd_rect)

        # 차량 위치 포인트
        ts_vals = [log.get('frenet_ts', 0) for log in bsd_logs]
        td_vals = [log.get('frenet_td', 0) for log in bsd_logs]
        bsd_results = [log.get('bsd_result', False) for log in bsd_logs]

        colors = ['red' if bsd else 'green' for bsd in bsd_results]
        ax1.scatter(ts_vals, td_vals, c=colors, alpha=0.6, s=20)
        ax1.plot([0], [0], 'bo', markersize=15, label='Ego', markeredgecolor='black', markeredgewidth=2)
        ax1.legend(loc='best')

        # 2. BSD State Over Time
        ax2 = plt.subplot(2, 3, 2)
        ax2.set_title('BSD State Over Time', fontsize=14, fontweight='bold')
        ax2.set_xlabel('Frame')
        ax2.set_ylabel('State')
        ax2.grid(True, alpha=0.3)

        frames = range(len(bsd_logs))
        bsd_active = [1 if log.get('bsd_result', False) else 0 for log in bsd_logs]
        immediate = [1 if log.get('immediate_bsd_zone', False) else 0 for log in bsd_logs]
        extended = [1 if log.get('extended_safety_zone', False) else 0 for log in bsd_logs]

        ax2.fill_between(frames, 0, bsd_active, alpha=0.3, color='red', label='BSD Active')
        ax2.plot(frames, immediate, 'r-', label='In Immediate Zone', linewidth=2)
        ax2.plot(frames, extended, 'orange', label='In Extended Zone', linewidth=2)
        ax2.set_ylim(-0.1, 1.1)
        ax2.legend(loc='best')

        # 3. TTC and Distance
        ax3 = plt.subplot(2, 3, 3)
        ax3.set_title('TTC and Distance', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Frame')
        ax3.grid(True, alpha=0.3)

        ttc_vals = [log.get('ttc') for log in bsd_logs]
        ttc_vals = [t if t is not None and t != float('inf') else None for t in ttc_vals]
        distance_vals = [log.get('distance', 0) for log in bsd_logs]
        ttc_threshold = bsd_logs[0].get('ttc_threshold', 5.0)

        ax3_twin = ax3.twinx()

        # TTC
        valid_ttc_frames = [i for i, v in enumerate(ttc_vals) if v is not None]
        valid_ttc_vals = [v for v in ttc_vals if v is not None]
        ax3.plot(valid_ttc_frames, valid_ttc_vals, 'b-', label='TTC', linewidth=2)
        ax3.axhline(y=ttc_threshold, color='blue', linestyle='--', linewidth=1, label=f'TTC Threshold ({ttc_threshold}s)')
        ax3.set_ylabel('TTC (s)', color='b')
        ax3.tick_params(axis='y', labelcolor='b')
        ax3.legend(loc='upper left')

        # Distance
        ax3_twin.plot(frames, distance_vals, 'g-', label='Distance', linewidth=2)
        ax3_twin.set_ylabel('Distance (m)', color='g')
        ax3_twin.tick_params(axis='y', labelcolor='g')
        ax3_twin.legend(loc='upper right')

        # 4. Safety Evaluation
        ax4 = plt.subplot(2, 3, 4)
        ax4.set_title('Safety Evaluation', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Frame')
        ax4.set_ylabel('Safe Status')
        ax4.grid(True, alpha=0.3)

        ttc_safe = [1 if log.get('ttc_safe', True) else 0 for log in bsd_logs]
        delta_v_safe = [1 if log.get('delta_v_safe', True) else 0 for log in bsd_logs]
        gap_safe = [1 if log.get('gap_safe', True) else 0 for log in bsd_logs]
        overall_safe = [1 if log.get('safety_evaluation', True) else 0 for log in bsd_logs]

        ax4.plot(frames, ttc_safe, 'b-', label='TTC Safe', linewidth=2, alpha=0.7)
        ax4.plot(frames, delta_v_safe, 'g-', label='Delta V Safe', linewidth=2, alpha=0.7)
        ax4.plot(frames, gap_safe, 'orange', label='Gap Safe', linewidth=2, alpha=0.7)
        ax4.plot(frames, overall_safe, 'r-', label='Overall Safe', linewidth=3)
        ax4.set_ylim(-0.1, 1.1)
        ax4.legend(loc='best')

        # 5. BSD Count
        ax5 = plt.subplot(2, 3, 5)
        ax5.set_title('BSD Count', fontsize=14, fontweight='bold')
        ax5.set_xlabel('Frame')
        ax5.set_ylabel('Count')
        ax5.grid(True, alpha=0.3)

        bsd_counts = [log.get('bsd_count', 0) for log in bsd_logs]
        ax5.plot(frames, bsd_counts, 'r-', linewidth=2)
        ax5.fill_between(frames, 0, bsd_counts, alpha=0.3, color='red')

        # 6. Detailed Information
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')
        ax6.set_title('Latest Frame Details', fontsize=14, fontweight='bold')

        last_log = bsd_logs[-1]
        info_text = f"""
Last Frame Information:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Frenet ts: {last_log.get('frenet_ts', 0):.2f} m
Frenet td: {last_log.get('frenet_td', 0):.2f} m

Immediate BSD Zone: {last_log.get('immediate_bsd_zone', False)}
Extended Safety Zone: {last_log.get('extended_safety_zone', False)}
BSD Result: {last_log.get('bsd_result', False)}
BSD Count: {last_log.get('bsd_count', 0)}

Distance: {last_log.get('distance', 0):.2f} m
Relative Velocity: {last_log.get('relative_velocity', 0):.2f} m/s
TTC: {last_log.get('ttc') if last_log.get('ttc') is not None else 'inf'} s

TTC Safe: {last_log.get('ttc_safe', True)}
Delta V Safe: {last_log.get('delta_v_safe', True)}
Gap Safe: {last_log.get('gap_safe', True)}
Overall Safety: {last_log.get('safety_evaluation', True)}

Current Signal: {last_log.get('current_signal', 0)}
        """
        ax6.text(0.05, 0.95, info_text, transform=ax6.transAxes,
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

        plt.tight_layout()

        # Save
        output_file = self.log_file.replace('.json', '_bsd.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved BSD visualization to: {output_file}")

        plt.show()

    def print_summary(self):
        """Print statistics summary"""
        merge_logs = self.filter_by_function('merge_safety_calc')
        bsd_logs = self.filter_by_function('calc_bsd')

        print("\n" + "="*60)
        print("SAFETY DEBUG LOG SUMMARY")
        print("="*60)
        print(f"Log File: {self.log_file}")
        print(f"Total Entries: {len(self.data)}")
        print(f"  - merge_safety_calc: {len(merge_logs)}")
        print(f"  - calc_bsd: {len(bsd_logs)}")

        if merge_logs:
            print("\nMerge Safety Statistics:")
            safety_levels = [log.get('safety_level', 0) for log in merge_logs]
            intersections = sum(1 for log in merge_logs if log.get('intersection_found', False))
            print(f"  - Unsafe (0): {safety_levels.count(0)}")
            print(f"  - Safe (1): {safety_levels.count(1)}")
            print(f"  - Emergency (2): {safety_levels.count(2)}")
            print(f"  - Intersections Found: {intersections}/{len(merge_logs)}")

        if bsd_logs:
            print("\nBSD Statistics:")
            bsd_active = sum(1 for log in bsd_logs if log.get('bsd_result', False))
            immediate = sum(1 for log in bsd_logs if log.get('immediate_bsd_zone', False))
            extended = sum(1 for log in bsd_logs if log.get('extended_safety_zone', False))
            print(f"  - BSD Active: {bsd_active}/{len(bsd_logs)}")
            print(f"  - Immediate Zone: {immediate}/{len(bsd_logs)}")
            print(f"  - Extended Zone: {extended}/{len(bsd_logs)}")

        print("="*60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_safety_debug.py <log_file.json>")
        print("\nExample:")
        print("  python3 visualize_safety_debug.py ../log/20250130_150000_slower_ego.json")
        sys.exit(1)

    log_file = sys.argv[1]

    # Check if file exists
    if not os.path.exists(log_file):
        print(f"Error: File not found: {log_file}")

        # Show available files
        log_dir = os.path.join(os.path.dirname(__file__), '..', 'log')
        if os.path.exists(log_dir):
            json_files = [f for f in os.listdir(log_dir) if f.endswith('.json')]
            if json_files:
                print(f"\nAvailable log files in {log_dir}:")
                for f in sorted(json_files, reverse=True)[:10]:  # Show last 10
                    print(f"  - {f}")
        sys.exit(1)

    visualizer = SafetyDebugVisualizer(log_file)

    if not visualizer.load_data():
        sys.exit(1)

    visualizer.print_summary()

    # Interactive menu
    while True:
        print("\nVisualization Options:")
        print("1. Merge Safety Calculation (Path Overlap)")
        print("2. BSD Calculation")
        print("3. Both")
        print("4. Exit")

        choice = input("\nSelect option (1-4): ").strip()

        if choice == '1':
            visualizer.visualize_merge_safety()
        elif choice == '2':
            visualizer.visualize_bsd()
        elif choice == '3':
            visualizer.visualize_merge_safety()
            visualizer.visualize_bsd()
        elif choice == '4':
            print("Exiting...")
            break
        else:
            print("Invalid option. Please try again.")


if __name__ == '__main__':
    main()
