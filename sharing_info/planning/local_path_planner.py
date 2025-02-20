

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
        self.target_state = 0
        self.change_state = False
        self.change_id = None

        self.local_path = None
        self.local_lane_number = 1
        self.prev_lane_number = 1
        self.temp_signal = 0
        self.threshold_gap = 2.5

        self.t_reaction_change = 2
        self.minimum_distance = 30
        self.d_lane = 3.5
        self.velocity_range = [0, 80]
        self.theta_range = [15, 20]
        self.L = 4.75
        self.max_path_len = 30
        self.default_len = 120
    
    def update_value(self,car, signal, target_state, dangerous_obstacle):
        self.local_pose = [car['x'], car['y']]
        self.current_velocity = car['v']
        self.current_signal = signal
        self.target_state = target_state
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
    
    def need_update(self):
        if self.local_path == None:
            return 0
        if self.temp_signal != self.current_signal and self.current_signal != 0 and self.current_signal <= 3:
            self.temp_signal = self.current_signal
            return 2
        else:
            return 1
    
    def check_planner_state(self, caution):
        if self.local_path == None:
            return 'INIT'
        if self.current_signal == 3:
            self.change_state = False
            return 'STRAIGHT'
        if not self.change_state:
            if self.temp_signal != self.current_signal and self.current_signal in [1, 2]:
                self.temp_signal = self.current_signal
                self.change_state = True
                return 'CHANGE'
            elif self.temp_signal != self.current_signal and self.current_signal in [4,5,6,7] :
                self.temp_signal = self.current_signal
                self.change_state = True
                return 'EMERGENCY_CHANGE'
            elif self.temp_signal != self.target_state and self.target_state in [4,5,6,7]:
                self.temp_signal = self.target_state
                self.change_state = True
                return 'EMERGENCY_CHANGE'
            else:
                return 'STRAIGHT'
        else:
            idnidx = self.phelper.lanelet_matching(self.local_pose)
            if idnidx[0] == self.change_id:
                self.change_state = False
                return 'INIT'
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
        if self.prev_lane_number == 1 and self.local_lane_number == 2:
            n_id = l_id if self.type == 'ego' else r_id

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
            l_tr1 = self.current_velocity*self.t_reaction_change + l_buffer_change
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
            local_path = tr1
        else:
            local_path = tr1

        return local_path

    
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
        if self.local_path == None or len(self.local_path) <= 0:
            return None
        #self.local_path, local_kappa = self.phelper.interpolate_path(local_path)
        local_waypoints, self.local_lane_number = self.current_lane_waypoints(self.local_pose)
        limit_local_path = self.phelper.limit_path_length(self.local_path, self.max_path_len)
        if self.local_lane_number != self.prev_lane_number:
            self.pre_lane_number = self.local_lane_number

        return self.local_path, limit_local_path, local_waypoints, self.local_lane_number, caution

