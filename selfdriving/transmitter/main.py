#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000

import asyncio
import rospy
import sys

from transmitter.simulator import Simulator
from transmitter.avante import Avante
from transmitter.ioniq5 import IONIQ5

def signal_handler(sig, frame):
    sys.exit(0)

class Transmitter():
    def __init__(self, type, car, map):
        self.set_values(type, car, map)
    
    def set_values(self, type, car, map):
        if car == 'simulator':
            self.target = Simulator(type, map)
        elif car == 'avante':
            self.target = Avante()
        elif car == 'ioniq5':
            self.target = IONIQ5()

    async def transmitter(self):
        while not rospy.is_shutdown():
            await self.target.execute()
            await asyncio.sleep(0.02) #100hz
        rospy.on_shutdown(self.target.cleanup)