#!/usr/bin/env python
import rospy

from ccavt.msg import *
from std_msgs.msg import Float32MultiArray

class ROSManager:
    def __init__(self, type):
        rospy.init_node(f'{type}_self_driving')
        self.type = type
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car = {'state':0, 'x': 0, 'y':0,'t':0,'v':0}
        self.local_path = []
        self.user_input = {'state': 0, 'signal': 0, 'target_velocity': 10/3.6}

    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/user_input',Float32MultiArray, self.user_input_cb)
    
    def ego_share_info_cb(self, msg):
        self.car['state'] = msg.state.data
        self.car['x'] = msg.pose.x
        self.car['y'] = msg.pose.y
        self.car['t'] = msg.pose.theta
        self.car['v'] = msg.velocity.data
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        self.local_path = path
        self.target_velocity = msg.target_velocity.data
    
    def user_input_cb(self, msg):
        self.user_input['state'] = msg.data[0]
        self.user_input['signal'] = msg.data[1]
        self.user_input['target_velocity'] = msg.data[2]
