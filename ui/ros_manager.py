#!/usr/bin/env python
import rospy
from datetime import datetime
from ccavt.msg import *
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker


class RosManager:
    def __init__(self, type, test):
        self.type = type
        self.test = test
        rospy.init_node(f"{type}_ui")
        self.start_time = None
        self.set_values()
        self.set_protocol()
        
    def set_values(self):
        self.Hz = 10
        self.rate = rospy.Rate(self.Hz)
        self.user_input = [0,0,0,0,1,1,1] # selfdriving, signal, target_velocity, scenario_type, scenario_number, sub_scenario, w/wo coop
        self.signals = {
            'ego': 0,
            'target': 0
        }
        self.ego_pos = [0,0]
        self.test_case = 'Test Case : '
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

        if self.test == 1:
            if self.type == 'ego':
                rospy.Subscriber('/target/EgoShareInfo', ShareInfo, self.target_share_info_cb)
            else:
                rospy.Subscriber('/ego/EgoShareInfo', ShareInfo, self.target_share_info_cb)
        else:
            rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb) 
            
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        rospy.Subscriber(f'/{self.type}/test_case', String, self.test_case_cb)
        self.pub_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)
        self.pub_plot_point = rospy.Publisher(f'/{self.type}/plot_point', Marker, queue_size=1)

    def ego_share_info_cb(self, msg:ShareInfo):
        self.signals['ego'] = msg.signal.data
        self.ego_pos = [msg.pose.x, msg.pose.y]
    
    def target_share_info_cb(self, msg:ShareInfo):
        self.signals['target']  = msg.signal.data
    
    def test_case_cb(self, msg:String):
        self.test_case = f"Test Case : {msg.data}"

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

    def publish_plot_point(self, pt):
        print(pt)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'intersection'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 3
        marker.scale.y = 3
        marker.scale.z = 0
        marker.color.r = 208/255
        marker.color.g = 255/255
        marker.color.b = 0/255
        marker.color.a = 1
        marker.pose.position.x = float(pt[0])
        marker.pose.position.y = float(pt[1])
        marker.pose.position.z = 0.2
        self.pub_plot_point.publish(marker)