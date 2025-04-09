#!/usr/bin/env python
import signal
import rospy
import math
import time
import sys
import csv
import os
from datetime import datetime

import setproctitle
setproctitle.setproctitle("make_data")

from novatel_oem7_msgs.msg import INSPVA, CORRIMU
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from ccavt.msg import *


def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")
    
class MakeData:
    def __init__(self, type):
        rospy.init_node(f'{type}_make_data')
        self.type = type
                
        self.set_values()
        self.set_protocols(type)

    def init_csv(self, sc_target_vel, sc_type, sc_number):
        log_dir = f"../log/{self.type}"
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sc_name = 'CLM' if sc_type == 1 else 'ETrA'
        a = 'a'
        if self.type == 'target':
            if self.target_velocity in [30, 50]:
                a = 'b'
            elif self.target_velocity in [35, 55]:
                a = 'c'
            else:
                a = 'a'
        else:
            print(self.sub_scenario)
            if self.sub_scenario == 2:
                a = 'b'
            elif self.sub_scenario == 3:
                a = 'c'
            else:
                a = 'a'
        sc_name = f'{sc_name}{sc_number}{a}_{sc_target_vel}'
        
        self.csv_file = os.path.join(log_dir, f"{sc_name}_{timestamp}.csv")

        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp", "state", "signal", "x", "y", "car_heading", "car_velocity",
                                 "lateral_acc", "longitudinal_acc", "pitch_rate", "roll_rate", "yaw_rate", "packet_rate",
                                 "rtt", "speed", "delay", "car_distance"])

    def set_values(self):

        self.car_pos = [0,0] #lat, long
        self.car_heading = 0
        self.car_velocity = 0
        self.car_accel = [0,0] #lat, long
        self.car_rotation = [0,0,0]
        self.comm_perform = [0,0,0, 0] #packet rate, rtt, speed, delay
        self.car_distance = 0

        self.state, self.signal, self.target_velocity = 0,0, 0
        self.csv_initiation = False
        self.sub_scenario = 1

    def set_protocols(self, type):
        rospy.Subscriber('/novatel/oem7/corrimu', CORRIMU, self.novatel_corrimu_cb)
        rospy.Subscriber(f'/{type}/EgoShareInfo', ShareInfo, self.share_info_cb)
        rospy.Subscriber(f'/{type}/CommunicationPerformance', Float32MultiArray, self.
        communication_performance_cb)
        rospy.Subscriber(f'/{type}/user_input', Float32MultiArray, self.user_input_cb)
    
    def share_info_cb(self, msg):
        self.car_pos = [msg.pose.x, msg.pose.y]
        self.car_heading = msg.pose.theta
        self.car_velocity = msg.velocity.data
        self.car_signal = msg.signal.data

    def novatel_corrimu_cb(self, msg: CORRIMU):
        self.car_accel = [msg.lateral_acc, msg.longitudinal_acc]
        self.car_rotation = [msg.pitch_rate, msg.roll_rate, msg.yaw_rate]
    
    def communication_performance_cb(self, msg:Float32MultiArray):
        self.comm_perform = [msg.data[5], msg.data[2], msg.data[3], msg.data[7]]
        self.car_distance = msg.data[6]

    def user_input_cb(self, msg:Float32MultiArray):
        self.state, self.signal = int(msg.data[0]), int(msg.data[1])
        target_velocity = int(msg.data[2]*3.6)
        if not self.csv_initiation and self.target_velocity != target_velocity:
            self.csv_initiation = True
            self.target_velocity = target_velocity
            self.init_csv(target_velocity, int(msg.data[3]), int(msg.data[4]))
            self.sub_scenario = int(msg.data[5])
        
        # if self.csv_initiation and self.state == 0:
        #     self.csv_initiation = False

    def write_to_csv(self):
        if not self.csv_initiation:
            return
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)

            writer.writerow([time.time(), self.state, self.signal, self.car_pos[0], self.car_pos[1], self.car_heading,
                             self.car_velocity, self.car_accel[0], self.car_accel[1], self.car_rotation[0], self.car_rotation[1], self.car_rotation[2],
                             self.comm_perform[0], self.comm_perform[1], self.comm_perform[2], self.comm_perform[3],
                             self.car_distance])
            
    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.write_to_csv()
            rate.sleep()

if __name__ == "__main__":
    type = str(sys.argv[1])
    make_data = MakeData(type)
    make_data.execute()