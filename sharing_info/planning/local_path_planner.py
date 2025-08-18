

import planning.libs.planning_helper as phelper
import copy
import rospy
import math

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

        self.local_path = None
        self.local_lane_number = 1
        self.prev_lane_number = 1
        self.temp_signal = 0
        self.threshold_gap = 2.5

        self.t_reaction_change = 2
        self.minimum_distance = 50 if self.type == 'ego' else 80
        self.d_lane = 3.5
        self.velocity_range = [0, 80]
        self.theta_range = [15, 20]
        self.L = 4.75
        self.max_path_len = 30
        self.default_len = 120

        self.safety = 0
        self.target_path = []
        self.confirm_safety = False
        self.check_safety = []
        self.intersection_radius = 1.5
        self.inter_pt = None
        self.target_pose = [0,0]

        self.bsd_range = [50, 5]
        self.ttz_th = 5
        self.bsd = False
        self.bsd_cnt = 0
    
    def update_value(self,car, user_input, target_info, target_path, dangerous_obstacle):
        self.local_pose = [car['x'], car['y']]
        self.current_velocity = car['v']
        self.current_signal = user_input['signal']
        self.scenario = (user_input['scenario'])
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
        # signal : 1 left change, 2 right change, 3 straight, 4 merge accept, 5 merge deny, 6 merge reset, 7 emergency
        if self.local_path == None:
            return 'INIT'
        if self.current_signal == 3:
            self.temp_signal = self.current_signal
            self.change_state = False
            return 'STRAIGHT'
        if not self.change_state:
            if self.temp_signal != self.current_signal and self.current_signal in [1, 2]:
                self.temp_signal = self.current_signal
                self.change_state = True
                return 'CHANGE'
            elif self.temp_signal != self.current_signal and self.current_signal in [7] :
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
                return 'STRAIGHT'
            else: # if merging rejected
                if (self.temp_signal != self.target_signal and self.target_signal == 5) or self.bsd: #Target merging rejected
                    self.temp_signal = 3
                    self.change_state = False
                    return 'STRAIGHT'
                else:
                    return 'CHANGING'
    
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
        if self.type == 'ego':
            if self.scenario == 5:
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
            l_buffer_change = max(0, self.minimum_distance - (self.current_velocity * self.t_reaction_change))
            l_tr1 = self.current_velocity*self.t_reaction_change + l_buffer_change + 5
        elif pstate == 'EMERGENCY_CHANGE':
            l_tr1 = self.current_velocity*(self.t_reaction_change-0.5)
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
        if self.local_path is None or len(self.target_path) < 2:
            self.safety = 0 #MERGE ALGORITHM FAIL

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
        
        if self.inter_pt is not None:
            if self.is_insied_circle(self.inter_pt, self.local_pose, self.intersection_radius):
                self.safety = 0
                self.confirm_safety = False
                self.check_safety = []
                self.inter_pt = None

        if find and not self.confirm_safety and self.target_signal in [1,2]:
            self.inter_pt = inter_pt
            now_idx = phelper.find_nearest_idx(self.local_path, self.local_pose)
            l_o1 = (inter_idx-now_idx)
            l_o2 = self.current_velocity * ((l_target)/self.target_velocity) if self.target_velocity != 0 else 0
            l_o3 = l_o1-l_o2
            d_TC = self.current_velocity*(self.t_reaction_change)

            if inter_idx <= now_idx+5:
                safety = 0  
            else:
                safety = 1 if abs(l_o3) > d_TC else 2 # 1 : Safe, 2 : Dangerous
            
            #print(abs(l_o3), d_TC, safety)

            #TODO
            # safety = 1 #safe mode (scenario 1, scenario 3)
            # safety = 2 #dangerous mode (scenario 2)

             # Safety 확인 로직
            if self.safety != safety:
                if len(self.check_safety) < 1:
                    self.check_safety.append(safety)
                    self.safety = safety
                    
            else:
                self.check_safety.append(safety)
                if len(self.check_safety) > 20:
                    self.confirm_safety = True
        
        elif self.safety == 2:
            if not self.is_insied_circle(self.inter_pt, self.local_pose, self.intersection_radius):
                self.confirm_safety = False
                self.check_safety = []
                self.inter_pt = None
            
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
    
    def calc_bsd(self):
        bsd = False
        if self.current_signal in [1,2]:
            ts, td = self.phelper.object_to_frenet(self.local_path, self.target_pose)
            if abs(ts) < self.bsd_range[0] and abs(td) < self.bsd_range[1]:
                d = abs(ts)-6
                v_rel = abs(self.target_velocity - self.current_velocity)
                ttz = d/v_rel
                if ttz < self.ttz_th:
                    bsd = True
                    self.bsd = bsd
                    self.temp_signal = 3
        if self.bsd:
            if self.bsd_cnt < 15:
                self.bsd_cnt += 1
            else:
                self.bsd = False
                self.bsd_cnt = 0
        return self.bsd

    def execute(self):
        if self.local_pose is None or self.local_pose[0] == 'inf':
            return None
        if self.type == 'ego':
            caution = self.phelper.calc_caution_by_ttc(self.dangerous_obstacle, self.local_pose, self.current_velocity)
        else:
            caution = False
        caution = False
        
        pstate = self.check_planner_state(caution)
        self.local_path = self.make_path(pstate, self.local_pose)
        if self.local_path is None or len(self.local_path) <= 0:
            return [self.local_pose],[self.local_pose],[self.local_pose],self.local_lane_number, caution, self.safety
        local_waypoints, self.local_lane_number = self.current_lane_waypoints(self.local_pose)
        limit_local_path = self.phelper.limit_path_length(self.local_path, self.max_path_len)
        self.local_path = phelper.smooth_interpolate(self.local_path)
        if self.local_lane_number != self.prev_lane_number:
            self.pre_lane_number = self.local_lane_number
        
        bsd = False
        if self.type == 'target':
            self.merge_safety_calc()
        # bsd = self.calc_bsd() -> without cooperation

        return self.local_path, limit_local_path, local_waypoints, self.local_lane_number, caution, self.safety, bsd
