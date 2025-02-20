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
    def __init__(self, type, map_name):
        self.map = MAP(map_name)
        self.lpp = LocalPathPlanner(self.map, type)
        self.oh = ObstacleHandler(self.lpp.phelper)
        self.RM = ROSManager(type, self.map, self.oh)
        self.set_values()
    
    def set_values(self):
        self.local_path = None
        self.limit_local_path = None
        self.local_waypoints = None
        self.local_lane_number = None

    def update_value(self):
        self.lpp.update_value(self.RM.car, self.RM.user_input['signal'], self.RM.target_state, self.RM.dangerous_obstacle)
        self.oh.update_value(self.RM.car, self.lpp.local_path) 

    def path_planning(self):
        lpp_result = self.lpp.execute()
        if lpp_result is not None:
            return lpp_result
        
    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_value()
            lpp_result = self.path_planning()
            if lpp_result is not None:
                self.RM.publish(lpp_result)
            rate.sleep()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 3 :
        type = 'ego'
        map_name = 'Solbat'
    else:
        type = str(sys.argv[1])
        map_name = str(sys.argv[2])
    
    si = SharingInfo(type, map_name)
    si.execute()

if __name__=="__main__":
    main()