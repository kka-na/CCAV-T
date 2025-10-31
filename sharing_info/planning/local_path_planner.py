

import planning.libs.planning_helper as phelper
import copy
import rospy
import math
import os
from datetime import datetime

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

class LocalPathPlanner:
    def __init__(self, map, type):
        self.MAP = map
        self.type = type
        self.setting_values()
    
    def setting_values(self):
        self.phelper = phelper
        self.phelper.lanelets = self.MAP.lanelets
        self.phelper.tiles = self.MAP.tiles
        self.phelper.tile_size = self.MAP.tile_size

        self.local_pose = None
        self.current_velocity = 0.0
        self.current_signal = 0
        self.scenario = 0
        self.target_signal = 0
        self.change_state = False
        self.change_id = None

        self.reject_cnt = 0
        self.local_path = None
        self.local_lane_number = 1
        self.prev_lane_number = 1
        self.temp_signal = 0
        self.threshold_gap = 2.5

        self.t_reaction_change = 2
        self.minimum_distance = 50
        self.d_lane = 3.5
        self.velocity_range = [0, 50]
        self.theta_range = [15, 20]
        self.L = 4.75
        self.max_path_len = 30
        self.default_len = 120

        self.reject_once = False
        self.reject_once_change_once = False
        self.reject_cnt = 0
        self.accept_received = False

        self.retry_attempted = False
        self.waiting_for_retry = False
        self.saved_signal = 0
        self.bsd_detected_once = False

        self.safety = 0
        self.target_path = []
        self.confirm_safety = False
        self.check_safety = []
        self.intersection_radius = 1.5
        self.inter_pt = None
        self.target_pose = [0,0]

        # BSD range: [ts_rear, ts_front, td_min, td_max]
        self.bsd_range = [-18, 15, 1.5, 6.0]
        self.extended_range = [-50, 30, 1.5, 6.0]

        # Safety thresholds
        self.ttc_threshold = 5.0
        self.delta_v_threshold = 5.0

        self.bsd = False
        self.bsd_cnt = 0
        self.immediate_bsd = False
        self.extended_bsd = False

        self.max_velocity = 0
        self.with_coop = True
        self.test_mode = 'unknown'

        # 차선 변경 효율성 측정용
        self.lane_change_request_time = None  # 차선 변경 요청 시점 (signal 발생)
        self.lane_change_start_time = None    # 차선 변경 실제 시작 시점 (CHANGE 상태)
        self.lane_change_delay = None         # 요청부터 시작까지 걸린 시간

        # 로그 파일 경로 설정
        log_dir = os.path.join(os.path.dirname(__file__), '../../logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'lane_change_efficiency.txt')
    
    def update_value(self,car, user_input, target_info, target_path, dangerous_obstacle):
        self.local_pose = [car['x'], car['y']]
        self.current_velocity = car['v']

        # 차선 변경 요청 시점 기록 (signal이 0,3,4,5 등에서 1,2로 변경될 때)
        prev_signal = self.current_signal
        self.current_signal = user_input['signal']

        if prev_signal not in [1, 2] and self.current_signal in [1, 2]:
            if self.lane_change_request_time is None:
                self.lane_change_request_time = rospy.get_time()
                mode = "WC" if self.with_coop else "WOC"
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                log_msg = f"[{timestamp}] [{self.type}][{mode}][Scenario:{self.scenario}] Lane change REQUESTED at {self.lane_change_request_time:.2f}s (signal: {self.current_signal})"
                rospy.loginfo(log_msg)

                # 파일에 저장
                with open(self.log_file_path, 'a') as f:
                    f.write(log_msg + '\n')

        self.scenario = (user_input['scenario'])
        self.max_velocity = user_input['target_velocity']
        self.with_coop = user_input.get('with_coop', True)
        self.target_signal = target_info[1]
        self.target_velocity = target_info[2]
        self.target_pose = [target_info[3], target_info[4]]
        if len(target_path) > 4:
            self.target_path = phelper.smooth_interpolate(target_path)
        self.dangerous_obstacle = dangerous_obstacle
    
    def current_lane_waypoints(self, local_pose): 
        idnidx = phelper.lanelet_matching(local_pose)
        if idnidx is None:
            return [], 1
        else:
            l_id, _ = idnidx
            lane_number = phelper.lanelets[l_id]['laneNo']
            curr_lane_waypoints = phelper.lanelets[l_id]['waypoints']
            curr_lane_waypoints = curr_lane_waypoints[:200] if len(curr_lane_waypoints) > 200 else curr_lane_waypoints
            return curr_lane_waypoints, lane_number

    def check_planner_state(self, caution):
        if self.local_path == None:
            return 'INIT'

        # ETrA 시나리오에서 caution이 감지되면 emergency 처리
        # WOC 모드에서는 target도 자체적으로 emergency 판단
        if caution:
            if not self.change_state:
                # Emergency 신호 발생
                self.temp_signal = 7
                self.change_state = True
                rospy.logwarn(f"[{self.type}] Caution detected! Emergency lane change triggered.")
                return 'EMERGENCY_CHANGE'
            # 이미 변경 중이라면 계속 진행

        if self.target_signal == 4:
            self.accept_received = True

        if self.accept_received and self.change_state:
            idnidx = self.phelper.lanelet_matching(self.local_pose)
            if idnidx[0] == self.change_id:
                self.change_state = False
                self.accept_received = False
                # 차선 변경 완료 - 측정 변수 리셋
                self._reset_lane_change_timing()
                return 'STRAIGHT'
            else:
                return 'CHANGING'

        if self.current_signal == 3:
            self.temp_signal = self.current_signal
            self.change_state = False
            # 차선 변경 취소 - 측정 변수 리셋
            self._reset_lane_change_timing()
            return 'STRAIGHT'

        if self.waiting_for_retry:
            if not self.immediate_bsd and not self.extended_bsd and not self.bsd:
                self.waiting_for_retry = False
                self.retry_attempted = True
                self.reject_once = False
                self.reject_cnt = 0
                self.current_signal = self.saved_signal
                self.temp_signal = self.saved_signal
                self.change_state = True
                # 차선 변경 시작 (retry 성공)
                self._record_lane_change_start()
                return 'CHANGE'
            else:
                if self.reject_once:
                    self.reject_cnt += 1
                return 'STRAIGHT'

        if self.reject_once and self.reject_cnt < 60:
            self.reject_cnt += 1
            return 'STRAIGHT'

        if self.bsd and self.bsd_cnt < 10:
            return 'STRAIGHT'

        if not self.change_state:
            if self.temp_signal != self.current_signal and self.current_signal in [1, 2]:
                self.temp_signal = self.current_signal
                self.change_state = True
                # 차선 변경 시작 (일반 신호)
                self._record_lane_change_start()
                return 'CHANGE'
            elif self.temp_signal != self.current_signal and self.current_signal in [7]:
                self.temp_signal = self.current_signal
                self.change_state = True
                return 'EMERGENCY_CHANGE'
            elif self.temp_signal != self.target_signal and self.target_signal in [7]:
                self.temp_signal = self.target_signal
                self.change_state = True
                return 'EMERGENCY_CHANGE'
            else:
                return 'STRAIGHT'
        else:
            idnidx = self.phelper.lanelet_matching(self.local_pose)
            if idnidx[0] == self.change_id:
                self.change_state = False
                # 차선 변경 완료 - 측정 변수 리셋
                self._reset_lane_change_timing()
                return 'STRAIGHT'
            else:
                if (self.temp_signal != self.target_signal and self.target_signal == 5):
                    if not self.reject_once and not self.retry_attempted:
                        self.saved_signal = self.temp_signal
                        self.waiting_for_retry = True
                    self.reject_once = True
                    self.temp_signal = 3
                    self.change_state = False
                    # Reject 받아서 중단 (retry 대기 상태로 진입, 리셋하지 않음)
                    return 'STRAIGHT'
                elif self.bsd:
                    if not self.retry_attempted and not self.waiting_for_retry:
                        self.saved_signal = self.current_signal
                        self.waiting_for_retry = True
                        self.bsd_detected_once = True
                    self.temp_signal = 3
                    self.change_state = False
                    # BSD 감지로 중단 (retry 대기 상태로 진입, 리셋하지 않음)
                    return 'STRAIGHT'
                else:
                    return 'CHANGING'

    def _record_lane_change_start(self):
        """차선 변경 실제 시작 시점 기록 및 지연 시간 계산"""
        if self.lane_change_start_time is None and self.lane_change_request_time is not None:
            self.lane_change_start_time = rospy.get_time()
            self.lane_change_delay = self.lane_change_start_time - self.lane_change_request_time
            mode = "WC" if self.with_coop else "WOC"
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            log_msg1 = f"[{timestamp}] [{self.type}][{mode}][Scenario:{self.scenario}] Lane change STARTED at {self.lane_change_start_time:.2f}s"
            log_msg2 = f"[{timestamp}] [{self.type}][{mode}][Scenario:{self.scenario}] === LANE CHANGE DELAY: {self.lane_change_delay:.3f}s ==="

            rospy.loginfo(log_msg1)
            rospy.logwarn(log_msg2)

            # 파일에 저장
            with open(self.log_file_path, 'a') as f:
                f.write(log_msg1 + '\n')
                f.write(log_msg2 + '\n')

    def _reset_lane_change_timing(self):
        """차선 변경 효율성 측정 변수 리셋"""
        if self.lane_change_delay is not None:
            mode = "WC" if self.with_coop else "WOC"
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            log_msg = f"[{timestamp}] [{self.type}][{mode}][Scenario:{self.scenario}] Lane change COMPLETED. Total delay was {self.lane_change_delay:.3f}s"

            rospy.loginfo(log_msg)

            # 파일에 저장
            with open(self.log_file_path, 'a') as f:
                f.write(log_msg + '\n')
                f.write('---\n')  # 구분선

        self.lane_change_request_time = None
        self.lane_change_start_time = None
        self.lane_change_delay = None

    def get_change_path(self, sni,  path_len, to=1):
        wps, uni = self.phelper.get_straight_path(sni, path_len)
        c_pt = wps[-1]
        l_id, r_id = self.phelper.get_neighbor(uni[0])
        n_id = l_id if to==1 else r_id
        if n_id != None:
            r = self.MAP.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = self.phelper.find_nearest_idx(r, c_pt)
            uni = [u_n, u_i]
        else:
            r = wps
        return r, uni
    
    def get_emergency_change_path(self, sni,  path_len):
        wps, uni = self.phelper.get_straight_path(sni, path_len)
        c_pt = wps[-1]
        l_id, r_id = self.phelper.get_neighbor(uni[0])
        n_id = r_id if r_id is not None else l_id
        
        if self.type == 'target':
            if self.scenario in [10,11]:
                n_id = l_id
        elif self.type == 'ego':
            if self.scenario in [8,10,11]:
                n_id = l_id

        if n_id is not None:
            r = self.MAP.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = self.phelper.find_nearest_idx(r, c_pt)
            uni = [u_n, u_i]
        else:
            rospy.logerr("[LocalPathPlanner] There is no Space to Change")
            r = wps
        return r, uni

    def make_path(self, pstate, local_pos):
        start_pose = local_pos 
        idnidx0 = self.phelper.lanelet_matching(start_pose)

        if idnidx0 == None:
            return None
        
        if pstate == 'CHANGE':
            if self.reject_once:
                pstate = 'EMERGENCY_CHANGE'
        
        if pstate == 'CHANGE':
            l_buffer_change = max(0, self.minimum_distance - (self.current_velocity * self.t_reaction_change))
            l_tr1 = self.current_velocity*self.t_reaction_change + l_buffer_change + 5
        elif pstate == 'EMERGENCY_CHANGE':
            
            l_buffer_change = max(0, self.minimum_distance/3- (self.current_velocity * self.t_reaction_change))
            if self.type == 'ego':
                l_tr1 = self.current_velocity*1.5 + l_buffer_change + 5
            else:
                l_tr1 = self.current_velocity + l_buffer_change + 5
        else:
            l_tr1 = self.default_len

        if pstate == 'CHANGING':
            idx = self.phelper.find_nearest_idx(self.local_path, local_pos)
            _local_path = copy.deepcopy(self.local_path)
            tr1 = _local_path[idx:]
        else:
            tr1, idnidx1 = self.phelper.get_straight_path(idnidx0, l_tr1)

        if pstate == 'CHANGE':
            theta = self.theta_range[1] - ((self.current_velocity-self.velocity_range[0])/(self.velocity_range[1]-self.velocity_range[0])) * (self.theta_range[1]-self.theta_range[0])
            l_tr2 = self.d_lane / math.sin(theta)
            tr2, idnidx2 = self.get_change_path(idnidx1, l_tr2, self.temp_signal)
            l_tr3 = self.L * self.current_velocity + self.default_len/2 if self.current_velocity > 0 else self.L * 3 + self.default_len/2 
            tr3, idnidx3 = self.phelper.get_straight_path(idnidx2, l_tr3)
            self.change_id = idnidx2[0]
            local_path = tr1+tr3
        elif pstate == 'EMERGENCY_CHANGE':
            if self.reject_once:
                self.reject_once_change_once = True
            theta = self.theta_range[1] - ((self.current_velocity-self.velocity_range[0])/(self.velocity_range[1]-self.velocity_range[0])) * (self.theta_range[1]-self.theta_range[0])
            l_tr2 = self.d_lane / math.sin(theta)
            tr2, idnidx2 = self.get_emergency_change_path(idnidx1, l_tr2)
            l_tr3 = self.L * self.current_velocity + self.default_len/2 if self.current_velocity > 0 else self.L * 3 + self.default_len/2 
            tr3, idnidx3 = self.phelper.get_straight_path(idnidx2, l_tr3)
            self.change_id = idnidx2[0]
            local_path = tr1+tr3
        elif pstate == 'CHANGING':
            prev_len = len(self.local_path)
            l_tr3 = prev_len-len(tr1) 
            idnidx2 = self.phelper.lanelet_matching(self.local_path[-1])
            tr3, _ = self.phelper.get_straight_path(idnidx2, l_tr3)
            local_path = tr1+tr3
        else:
            local_path = tr1

        return local_path

    def merge_safety_calc(self):
        # 기본 조건 확인
        if self.local_path is None or len(self.target_path) < 2:
            self.safety = 0
            return

        # 교차점 찾기
        find = False
        inter_idx = 0
        l_target = 0
        inter_pt = None

        for hi, hwp in enumerate(self.target_path):
            if find:
                break
            for ti, twp in enumerate(self.local_path):
                if self.is_insied_circle(twp, hwp, self.intersection_radius):
                    inter_idx = ti
                    inter_pt = twp
                    l_target = hi
                    find = True
                    break

        # 이미 교차점 지나쳤는지 확인
        if self.inter_pt is not None:
            if self.is_insied_circle(self.inter_pt, self.local_pose, self.intersection_radius):
                self.safety = 0
                self.confirm_safety = False
                self.check_safety = []
                self.inter_pt = None

        # 안전도 계산
        if find and not self.confirm_safety and self.target_signal in [1, 2]:
            self.inter_pt = inter_pt
            now_idx = phelper.find_nearest_idx(self.local_path, self.local_pose)
            l_o1 = (inter_idx - now_idx)

            c_v = self.current_velocity
            t_v = self.target_velocity
            l_o2 = c_v * ((l_target) / t_v) if t_v != 0 else 0
            l_o3 = l_o1 - l_o2
            d_TC = c_v * (self.t_reaction_change+1)
            if inter_idx <= now_idx + 5:
                safety = 0
            else:
                safety = 1 if abs(l_o3) > d_TC else 2
            
            
            # 안전도 확정 로직
            if self.safety != safety:
                if len(self.check_safety) < 1:
                    self.check_safety.append(safety)
                    self.safety = safety
            else:
                self.check_safety.append(safety)
                if len(self.check_safety) > 5:
                    self.confirm_safety = True
            
            
    def is_insied_circle(self, pt1, pt2, radius):
        if pt1 is None or pt2 is None:
            return False
        distance = math.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)
        if distance <=  radius:
            return True
        else:
            return False
    
    def get_interpt(self):
        return self.inter_pt
    
    def get_bsd_info(self):
        if not self.with_coop and self.current_signal in [1, 2]:
            return {
                'active': True,
                'signal': self.current_signal,
                'bsd_range': self.bsd_range,
                'local_path': self.local_path
            }
        return None

    def calc_bsd(self):
        bsd = False
        ts, td = 0, 0
        immediate_bsd = False
        extended_zone = False
        safety_evaluation = True

        if self.current_signal in [1, 2]:
            ts, td = self.phelper.object_to_frenet(self.local_path, self.target_pose)

            immediate_bsd = self._is_in_immediate_bsd(ts, td)
            if immediate_bsd:
                bsd = True
                self.bsd = bsd
                self.immediate_bsd = True
                self.extended_bsd = False
            else:
                self.immediate_bsd = False
                extended_zone = self._is_in_extended_zone(ts, td)
                if extended_zone:
                    safety_evaluation = self._evaluate_lane_change_safety(ts, td)
                    if not safety_evaluation:
                        bsd = True
                        self.bsd = bsd
                        self.extended_bsd = True
                    else:
                        self.extended_bsd = False
                else:
                    self.extended_bsd = False
        else:
            self.immediate_bsd = False
            self.extended_bsd = False

        bsd_timeout = False
        if self.bsd:
            if self.bsd_cnt < 10:
                self.bsd_cnt += 1
            else:
                self.bsd = False
                self.bsd_cnt = 0
                bsd_timeout = True

        distance = abs(ts) if self.current_signal in [1, 2] else 0
        v_rel = self.target_velocity - self.current_velocity
        ttc = distance / abs(v_rel) if abs(v_rel) > 0.1 else float('inf')
        min_gap = 2.0 * self.current_velocity
        delta_v_safe = abs(v_rel) <= self.delta_v_threshold
        gap_safe = distance >= min_gap
        ttc_safe = ttc > self.ttc_threshold

        return self.bsd

    def _is_in_immediate_bsd(self, ts, td):
        ts_rear, ts_front, td_min, td_max = self.bsd_range

        in_longitudinal = (ts_rear <= ts <= ts_front)

        if self.current_signal == 1:
            in_lateral = (td_min <= td <= td_max) if td > 0 else False
        else:
            in_lateral = (td_min <= -td <= td_max) if td < 0 else False

        return in_longitudinal and in_lateral

    def _is_in_extended_zone(self, ts, td):
        ts_rear, ts_front, td_min, td_max = self.extended_range

        in_longitudinal = (ts_rear <= ts <= ts_front)

        if self.current_signal == 1:
            in_lateral = (td_min <= td <= td_max) if td > 0 else False
        else:
            in_lateral = (td_min <= -td <= td_max) if td < 0 else False

        return in_longitudinal and in_lateral

    def _evaluate_lane_change_safety(self, ts, td):
        distance = abs(ts)
        v_rel = self.target_velocity - self.current_velocity

        if abs(v_rel) > 0.1:
            ttc = distance / abs(v_rel)
        else:
            ttc = float('inf')

        delta_v_safe = abs(v_rel) <= self.delta_v_threshold
        min_gap = 2.0 * self.current_velocity
        gap_safe = distance >= min_gap
        ttc_safe = ttc > self.ttc_threshold

        is_safe = ttc_safe and delta_v_safe and gap_safe

        return is_safe

    def execute(self):
        if self.local_pose is None or self.local_pose[0] == 'inf':
            return None

        # Caution 계산: ETrA 시나리오(7-12)에서만
        caution = False
        is_etra_scenario = 7 <= self.scenario <= 12
        if is_etra_scenario:
            if self.with_coop:
                # WC 모드: ego만 caution 계산
                if self.type == 'ego':
                    caution = self.phelper.calc_caution_by_ttc(self.dangerous_obstacle, self.local_pose, self.current_velocity)
            else:
                # WOC 모드: ego, target 둘 다 caution 계산
                caution = self.phelper.calc_caution_by_ttc(self.dangerous_obstacle, self.local_pose, self.current_velocity)
        # Ego: WC/WOC 모두 BSD 체크 (경로 생성 전)
        bsd = False
        if self.type == 'ego':
            bsd = self.calc_bsd()

        pstate = self.check_planner_state(caution)
        self.local_path = self.make_path(pstate, self.local_pose)
        if self.local_path is None or len(self.local_path) <= 0:
            return [self.local_pose],[self.local_pose],[self.local_pose],self.local_lane_number, caution, self.safety
        local_waypoints, self.local_lane_number = self.current_lane_waypoints(self.local_pose)
        limit_local_path = self.phelper.limit_path_length(self.local_path, self.max_path_len)
        self.local_path = phelper.smooth_interpolate(self.local_path)
        if self.local_lane_number != self.prev_lane_number:
            self.pre_lane_number = self.local_lane_number

        # WC 모드: target이면 merge_safety_calc 실행
        if self.with_coop and self.type == 'target':
            self.merge_safety_calc()

        target_pos = self.set_your_position()
        return self.local_path, limit_local_path, local_waypoints, self.local_lane_number, caution, self.safety, bsd, target_pos 

    def set_your_position(self):
        s, d = phelper.object_to_frenet(self.local_path, self.target_pose)
        if s < 0 :
            ss = 'REAR'
        else:
            ss = 'FRONT'
        
        if d < 0:
            dd = 'LEFT'
        else:
            dd = 'RIGHT'
        
        return ss, dd