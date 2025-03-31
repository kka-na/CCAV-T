import time
import sys
import signal
import numpy as np
import configparser

from control.libs.apid import APID
from control.libs.purepursuit import PurePursuit
from control.libs.point import Point 

def signal_handler(sig, frame):
    sys.exit(0)

class Control():
    def __init__(self, car):
        config = self.get_config(car)
        self.APID = APID(config)
        self.PP = PurePursuit(config)
        self.set_values()
    
    def get_config(self, car):
        config_file_path = f'./control/configs/{car}.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        return config
    
    def set_values(self):
        self.max_velocity = 0
        self.target_velocity = 0
        self.state = 0
        self.current_location = Point(x=0, y=0)
        self.current_velocity = 0
        self.current_heading = 0
        self.local_path = []
        
    def update_value(self, max_velocity, car, path):
        self.max_velocity = max_velocity
        self.state = car['state']
        self.current_location = Point(x=car['x'],y=car['y'])
        self.current_velocity = car['v']
        self.current_heading = car['t']
        self.local_path = []
        if path is not None:
            for point in path:
                self.local_path.append(Point(x=point[0], y=point[1]))
        self.calculate_target_velocity(len(path))
            
    def calculate_target_velocity(self, path_len):
        diff = self.max_velocity - self.current_velocity
        if path_len > 50:
            if diff > self.max_velocity / 2 :
                velocity = self.current_velocity + 1
            else:
                velocity = self.current_velocity + 0.5
        else:
            velocity = self.current_velocity - 0.5
        
        self.target_velocity = velocity if velocity < self.max_velocity else self.max_velocity
        

    def execute(self):
        acc = self.APID.execute(self.state, self.target_velocity, self.current_velocity)
        steer, lh= self.PP.execute(self.current_location, self.state, self.local_path, self.current_heading, self.current_velocity)        
        return [acc, steer], lh
        
def main():
    signal.signal(signal.SIGINT, signal_handler)
    control = Control()
    control.execute()

if __name__ == "__main__":
    main()