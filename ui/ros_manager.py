#!/usr/bin/env python
import rospy
import random
from datetime import datetime
from ccavt.msg import *
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CompressedImage
from PyQt5.QtGui import QImage, QPixmap
import time
import threading

import numpy as np
import cv2 

class RosManager:
    def __init__(self, type, test):
        self.type = type
        self.test = test
        rospy.init_node(f"{type}_ui")
        self.start_time = None
        
        # 실시간 이미지 처리를 위한 변수들
        self.image_lock = threading.Lock()
        self.compressed_image = None
        self.last_image_time = 0
        self.frame_skip_count = 0
        
        self.set_values()
        self.set_protocol()
        
    def set_values(self):
        self.Hz = 10
        self.rate = rospy.Rate(self.Hz)
        self.user_input = [0,0,0,0] # selfdriving, signal, target_velocity, selected scenario

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

        # 실시간 이미지를 위한 최적화된 subscriber 설정
        # queue_size=1: 최신 메시지 1개만 유지
        # buff_size를 크게 설정: 네트워크 버퍼 최적화
        # tcp_nodelay=True: TCP Nagle 알고리즘 비활성화로 지연 최소화
        rospy.Subscriber('/gmsl_camera/dev/video2/compressed', CompressedImage, 
                        self.compressed_image_cb, queue_size=1, buff_size=2**24, 
                        tcp_nodelay=True) #IONiQ5
        rospy.Subscriber('/camera/image_color/compressed', CompressedImage, 
                        self.compressed_image_cb, queue_size=1, buff_size=2**24,
                        tcp_nodelay=True)
        
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        rospy.Subscriber(f'/{self.type}/test_case', String, self.test_case_cb)
        self.pub_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)
        self.pub_plot_point = rospy.Publisher(f'/{self.type}/plot_point', Marker, queue_size=1)

    def ego_share_info_cb(self, msg:ShareInfo):
        self.signals['ego'] = msg.signal.data
        self.ego_pos = [msg.pose.x, msg.pose.y]
        if self.start_time is None:
            self.start_time = datetime.now()
        self.communication_performance['comulative_time'] = str(datetime.now() - self.start_time)
        self.communication_performance['distance'] = str(round(random.randint(5,30),5))
        self.communication_performance['rtt'] = str(round(random.randint(900,1800),5))
        self.communication_performance['speed'] = str(round(0.001,5))
        self.communication_performance['delay'] = str(round(random.randint(300,700),2))
        self.communication_performance['packet_size'] = str(int(347))
        self.communication_performance['packet_rate'] = str(int(random.randint(70,100))) if random.randint(70,100) < 100 else str(100)
    
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
    
    def compressed_image_cb(self, msg):
        """실시간 이미지 처리 - 타임스탬프 검증 제거"""
        current_time = time.time()
        
        # FPS 제한만 적용 (30fps 정도로)
        if current_time - self.last_image_time < 0.033:  # 33ms
            return
            
        try:
            # OpenCV 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if compressed_image is not None:
                height, width, channel = compressed_image.shape
                bytes_per_line = 3 * width
                q_img = QImage(compressed_image.data, width, height, 
                            bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                pixmap = QPixmap.fromImage(q_img)
                
                with self.image_lock:
                    self.compressed_image = pixmap
                    self.last_image_time = current_time
                    
        except Exception as e:
            print(f"Image processing error: {e}")
            
    def get_latest_image(self):
        """UI에서 호출할 최신 이미지 반환"""
        with self.image_lock:
            return self.compressed_image

    def publish(self):
        self.pub_user_input.publish(Float32MultiArray(data=list(self.user_input)))

    def publish_plot_point(self, pt):
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