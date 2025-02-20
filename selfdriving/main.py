#!/usr/bin/env python3
import rospy
import sys
import signal

import setproctitle
setproctitle.setproctitle("self_driving")

from ros_manger import ROSManager
from control.control import Control
import selfdriving_helper as sdhelper

from transmitter.simulator import Simulator

def signal_handler(sig, frame):
    sys.exit(0)

class SelfDriving():
    def __init__(self, type,car, map):
        self.RM = ROSManager(type)
        self.ct = Control(car)
        self.set_values(type, car, map)
    
    def set_values(self, type, car, map):
        self.car = None
        self.local_path = None
        if car == 'simulator':
            self.tm = Simulator(type, map, 1)

    def update_values(self):
        self.car = self.RM.car
        self.local_path = sdhelper.upsample_path_1m(self.RM.local_path)
        self.ct.update_value(self.RM.target_velocity, self.car, self.local_path)
        
    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_values()
            if self.local_path is not None:
                actuator = self.ct.execute()
                self.tm.set_actuator(actuator)
                self.tm.execute()
                self.RM.publish()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 4:
        type = 'ego'
        car = 'simulator'
        map = 'Solbat'
    else:
        type = str(sys.argv[1])
        car = str(sys.argv[2])
        map = str(sys.argv[3])
    
    sd = SelfDriving(type, car, map)
    sd.execute()

if __name__ == "__main__":
    main()