#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000

import asyncio
import can
import rospy

import sys
import signal

import random

from transmitter.simulator import Simulator

def signal_handler(sig, frame):
    sys.exit(0)

class Transmitter():
    def __init__(self, car, map):
        self.set_values(car, map)
    
    def set_values(self, car, map):
        if car == 'simulator':
            self.target = Simulator('ego', map, 1)

    async def transmitter(self):
        while not rospy.is_shutdown():
            self.target.execute()
            # dicts = self.RH.update_can_inputs()
            # can_messages = self.TH.encode_message(dicts)
            # for can_message in can_messages:
            #     await asyncio.get_event_loop().run_in_executor(None, self.bus0.send, can_message)
            #     await asyncio.sleep(0.02)  # 100Hz, 10ms 간격
            await asyncio.sleep(0.02) #100hz