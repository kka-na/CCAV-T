#!/usr/bin/env python
import rospy
import random
from datetime import datetime
from ccavt.msg import *
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CompressedImage
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal
import time
import threading
import queue

import numpy as np
import cv2 

class ImageProcessor(QThread):
    """별도 스레드에서 이미지 처리를 담당"""
    image_ready = pyqtSignal(object)
    
    def __init__(self):
        super().__init__()
        self.image_queue = queue.Queue(maxsize=2)  # 최대 2개만 큐에 보관
        self.running = True
        
        # 1920x720 비율을 유지하면서 UI 라벨에 맞는 크기 계산
        # 라벨 높이가 300이므로, 비율 유지: 800x300
        self.target_width = 600 # 1920/720 * 300 ≈ 800
        self.target_height = 300
        
    def add_image_data(self, msg_data):
        """새 이미지 데이터 추가 (큐가 가득 차면 오래된 것 제거)"""
        try:
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()  # 오래된 이미지 제거
                except queue.Empty:
                    pass
            self.image_queue.put(msg_data, block=False)
        except queue.Full:
            pass  # 큐가 가득 차면 무시
            
    def run(self):
        """백그라운드에서 이미지 처리"""
        while self.running:
            try:
                msg_data = self.image_queue.get(timeout=0.1)
                
                # OpenCV 디코딩 (가장 빠른 설정)
                np_arr = np.frombuffer(msg_data, np.uint8)
                compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if compressed_image is not None:
                    # 1920x720 비율 유지하면서 리사이즈 (가장 빠른 보간법)
                    resized_image = cv2.resize(compressed_image, 
                                             (self.target_width, self.target_height), 
                                             interpolation=cv2.INTER_NEAREST)  # 가장 빠른 보간법
                    
                    height, width, channel = resized_image.shape
                    bytes_per_line = 3 * width
                    
                    # QImage 생성
                    q_img = QImage(resized_image.data, width, height, 
                                bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                    pixmap = QPixmap.fromImage(q_img)
                    
                    # UI로 신호 전송
                    self.image_ready.emit(pixmap)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Image processing error: {e}")
                continue
                
    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class RosManager:
    def __init__(self, type, test):
        self.type = type
        self.test = test
        rospy.init_node(f"{type}_ui")
        self.start_time = None
        
        # 이미지 처리를 위한 별도 스레드
        self.image_processor = ImageProcessor()
        self.image_processor.start()
        self.emergency_image_msg = None
        self.emergency_img_cnt = 0

        # 현재 이미지 저장
        self.image_lock = threading.Lock()
        self.compressed_image = None
        self.last_image_time = 0
        
        # 프레임 스킵 카운터 (성능 향상을 위해)
        self.frame_count = 0
        self.skip_frames = 1  # 매 2프레임 중 1프레임만 처리
        
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

        # 최적화된 이미지 subscriber 설정
        # queue_size=1: 지연 최소화
        # buff_size 최대: 네트워크 성능 향상
        rospy.Subscriber('/gmsl_camera/dev/video4/compressed', CompressedImage, 
                        self.compressed_image_cb, queue_size=1, buff_size=2**28, 
                        tcp_nodelay=True) #IONiQ5
        rospy.Subscriber('/camera/image_color/compressed', CompressedImage, 
                        self.compressed_image_cb, queue_size=1, buff_size=2**28,
                        tcp_nodelay=True)
        
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        rospy.Subscriber(f'/{self.type}/test_case', String, self.test_case_cb)
        self.pub_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)
        self.pub_plot_point = rospy.Publisher(f'/{self.type}/plot_point', Marker, queue_size=1)
        self.pub_emergency_image = rospy.Publisher(f'/{self.type}/emergency_image', CompressedImage, queue_size=1)

        # 이미지 프로세서의 신호 연결
        self.image_processor.image_ready.connect(self.on_image_processed)

    def ego_share_info_cb(self, msg:ShareInfo):
        self.signals['ego'] = msg.signal.data
        self.ego_pos = [msg.pose.x, msg.pose.y]
        if self.test == 1:
            if self.start_time is None:
                self.start_time = datetime.now()
            self.communication_performance['comulative_time'] = str(datetime.now() - self.start_time)
            self.communication_performance['distance'] = str(round(random.randint(5,30),5))
            self.communication_performance['rtt'] = str(round(random.randint(0,1500),5))
            self.communication_performance['speed'] = str(round((random.randint(0,49))/100,2))
            self.communication_performance['delay'] = str(round(random.randint(0,1000),2))
            self.communication_performance['packet_size'] = str(int(347))
            self.communication_performance['packet_rate'] = str(int(random.randint(0,100))) if random.randint(0,100) < 100 else str(100)
        if self.signals['ego'] == 7 and self.emergency_image_msg is not None:
            if self.emergency_img_cnt < 7:
                self.pub_emergency_image.publish(self.emergency_image_msg)
                self.emergency_img_cnt += 1
        elif self.signals['ego'] != 7:
            self.emergency_img_cnt = 0
    
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
        """ROS 콜백 - 최소한의 처리만 수행"""
        self.frame_count += 1
        
        # 프레임 스킵으로 처리량 조절
        if self.frame_count % (self.skip_frames + 1) != 0:
            return
            
        # 백그라운드 스레드로 이미지 데이터 전달
        self.image_processor.add_image_data(msg.data)
        self.emergency_image_msg = msg
    
    def on_image_processed(self, pixmap):
        """이미지 처리 완료 시 호출 - UI 스레드에서 실행"""
        with self.image_lock:
            self.compressed_image = pixmap
            self.last_image_time = time.time()
            
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
        marker.scale.z = 0.3
        marker.color.r = 208/255
        marker.color.g = 255/255
        marker.color.b = 0/255
        marker.color.a = 1
        marker.pose.position.x = float(pt[0])
        marker.pose.position.y = float(pt[1])
        marker.pose.position.z = 0.2
        self.pub_plot_point.publish(marker)
        
    def cleanup(self):
        """종료 시 리소스 정리"""
        self.image_processor.stop()