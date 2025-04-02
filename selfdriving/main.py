#!/usr/bin/env python3
import rospy
import sys
import signal
import asyncio

import setproctitle
setproctitle.setproctitle("self_driving")

from ros_manger import ROSManager
from control.control import Control
from transmitter.main import Transmitter
import selfdriving_helper as sdhelper


def signal_handler(sig, frame):
    sys.exit(0)

class SelfDriving():
    def __init__(self, type,car, map):
        self.RM = ROSManager(type)
        self.ct = Control(car)
        self.tm = Transmitter(type, car, map)
        self.set_values()
    
    def set_values(self):
        self.car = None
        self.local_path = None

    def update_values(self):
        self.car = self.RM.car
        self.local_path = sdhelper.upsample_path_1m(self.RM.local_path)
        self.ct.update_value(self.RM.user_input['target_velocity'], self.car, self.local_path)

    async def control(self):
        while not rospy.is_shutdown():
            self.update_values()
            actuator, lh = self.ct.execute()
            self.RM.pub_lh(lh)
            self.tm.target.set_actuator(actuator)
            self.tm.target.set_user_input(self.RM.user_input)
            await asyncio.sleep(0.1) #10hz             

    def execute(self):
        loop = asyncio.get_event_loop()
        control = loop.create_task(self.control())
        transmitter = loop.create_task(self.tm.transmitter())
        loop.run_forever()

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