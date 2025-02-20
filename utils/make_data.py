#!/usr/bin/env python
import signal
import rospy
import math
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
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"../log/{self.type}_data_{timestamp}.csv"

        self.init_csv()
        self.set_values()
        self.set_protocols(type)

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["state", "x", "y", "car_heading", "car_velocity",
                                 "lateral_acc", "longitudinal_acc", "packet_rate",
                                 "rtt", "speed", "car_distance"])

    def set_values(self):

        self.car_pos = [0,0] #lat, long
        self.car_heading = 0
        self.car_velocity = 0
        self.car_state = 0
        self.car_accel = [0,0] #lat, long
        self.comm_perform = [0,0,0] #packet rate, rtt, speed
        self.car_distance = 0

    def set_protocols(self, type):
        rospy.Subscriber('/novatel/oem7/corrimu', CORRIMU, self.novatel_corrimu_cb)
        rospy.Subscriber(f'/{type}/EgoShareInfo', ShareInfo, self.share_info_cb)
        rospy.Subscriber(f'/{type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
    
    def share_info_cb(self, msg):
        self.car_pos = [msg.pose.x, msg.pose.y]
        self.car_heading = msg.pose.theta
        self.car_velocity = msg.velocity.data
        self.car_state = msg.state.data

    def novatel_corrimu_cb(self, msg: CORRIMU):
        self.car_accel = [msg.lateral_acc, msg.longitudinal_acc]
    
    def communication_performance_cb(self, msg:Float32MultiArray):
        self.comm_perform = [msg.data[5], msg.data[2], msg.data[3]]
        self.car_distance = msg.data[6]

    def write_to_csv(self):
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.car_state, self.car_pos[0], self.car_pos[1], self.car_heading,
                             self.car_velocity, self.car_accel[0], self.car_accel[1],
                             self.comm_perform[0], self.comm_perform[1], self.comm_perform[2],
                             self.car_distance])
            
    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.write_to_csv()
            rate.sleep()

if __name__ == "__main__":
    type = str(sys.argv[1])
    make_data = MakeData(type)
    make_data.execute()