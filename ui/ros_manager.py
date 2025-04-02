#!/usr/bin/env python
import rospy
from datetime import datetime
from ccavt.msg import *
from std_msgs.msg import Float32MultiArray


class RosManager:
    def __init__(self, type):
        self.type = type
        rospy.init_node(f"{type}_ui")
        self.start_time = None
        self.set_values()
        self.set_protocol()
        
    def set_values(self):
        self.Hz = 10
        self.rate = rospy.Rate(self.Hz)
        self.user_input = [0,0,0,0,0] # selfdriving, signal, target_velocity, scenario_type, scenario_number
        self.signals = {
            'ego': 0,
            'target': 0
        }
        self.communication_performance = {
            'comulative_time':0,
            'distance':0,
            'rtt':0,
            'speed':0,
            'delay':0,
            'packet_size':0,
            'packet_rate':0
        }

    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb)
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)

        self.pub_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)

    def ego_share_info_cb(self, msg:ShareInfo):
        self.signals['ego'] = msg.signal.data
    
    def target_share_info_cb(self, msg:ShareInfo):
        self.signals['target']  = msg.signal.data

    def communication_performance_cb(self, msg):
        state, v2x, rtt, mbps, packet_size, packet_rate, distance, delay = msg.data
        if rtt == 0:
            return
        if self.start_time is None:
            self.start_time = datetime.now()
        self.communication_performance['comulative_time'] = str(datetime.now() - self.start_time)
        self.communication_performance['distance'] = str(round(distance,5))
        self.communication_performance['rtt'] = str(round(rtt,5))
        self.communication_performance['speed'] = str(round(mbps,5))
        self.communication_performance['delay'] = str(round(delay,2))
        self.communication_performance['packet_size'] = str(int(packet_size))
        self.communication_performance['packet_rate'] = str(int(packet_rate)) if packet_rate < 100 else str(100)
    
    def publish(self):
        self.pub_user_input.publish(Float32MultiArray(data=list(self.user_input)))