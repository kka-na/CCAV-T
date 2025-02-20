#!/usr/bin/env python
import rospy

from ccavt.msg import *

class ROSManager:
    def __init__(self, type):
        rospy.init_node(f'{type}_self_driving')
        self.type = type
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car = {'state':0, 'x': 0, 'y':0,'t':0,'v':0}
        self.local_path = []
        self.target_velocity = 3

    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
    
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
    

    def publish(self):
        pass