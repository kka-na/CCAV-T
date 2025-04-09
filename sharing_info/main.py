#!/usr/bin/env python3
import rospy
import sys
import signal

import setproctitle
setproctitle.setproctitle("sharing_info")
from ros_manager import ROSManager
from planning.local_path_planner import LocalPathPlanner
from perception.obstacle_handler import ObstacleHandler

from hd_map.map import MAP

def signal_handler(sig, frame):
    sys.exit(0)

class SharingInfo():
    def __init__(self, type, map_name, test):
        self.map = MAP(map_name)
        self.lpp = LocalPathPlanner(self.map, type)
        self.oh = ObstacleHandler(self.lpp.phelper)
        self.RM = ROSManager(type, test, self.map, self.oh)
        self.set_values()
    
    def set_values(self):
        self.local_path = None
        self.limit_local_path = None
        self.local_waypoints = None
        self.local_lane_number = None
        self.vp_result = 0
        self.target_signal = 0

    def update_value(self):
        self.lpp.update_value(self.RM.car, self.RM.user_input, self.RM.target_info, self.RM.target_path, self.RM.dangerous_obstacle)
        self.oh.update_value(self.RM.car, self.lpp.local_path) 

    def path_planning(self):
        lpp_result = self.lpp.execute()
        if lpp_result is not None:
            return lpp_result

    def velocity_planning(self):
        if self.RM.target_velocity != self.RM.user_input['target_velocity']:
            self.RM.target_velocity = self.RM.user_input['target_velocity']
            self.vp_result = self.RM.user_input['target_velocity']
        else:
            target_signal = self.RM.target_info[1] 
            if self.target_signal != target_signal and target_signal == 5:
                self.target_signal = target_signal
                self.vp_result = self.vp_result - (7/3.6)
            elif self.target_signal != target_signal and target_signal == 0:
                self.target_signal = target_signal
                self.vp_result = self.vp_result + (7/3.6)
        return self.vp_result
    
    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_value()
            lpp_result = self.path_planning()
            if lpp_result is not None:
                vp_result = self.velocity_planning()
                self.RM.publish(lpp_result, vp_result)
                self.RM.publish_inter_pt(self.lpp.get_interpt())
            rate.sleep()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 4 :
        type = 'ego'
        map_name = 'Solbat'
        test = 0
    else:
        type = str(sys.argv[1])
        map_name = str(sys.argv[2])
        test = int(sys.argv[3])
    
    si = SharingInfo(type, map_name, test)
    si.execute()

if __name__=="__main__":
    main()