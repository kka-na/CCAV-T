import rospy
from datetime import datetime
import sys
import numpy as np
import cv2
import base64
import json

from pyproj import Proj, Transformer

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from ccavt.msg import ShareInfo
from map_config import *
import paho.mqtt.client as mqtt

MAP = 'Solbat'
class ObigoTest:
    def __init__(self, type):
        self.type = type
        rospy.init_node('obigo_test', anonymous=False)
        self.set_mqtt()
        self.set_values() 
        self.set_protocols()

    def set_mqtt(self):
        self.broker_address = "infocabin.obigo.com"
        self.broker_port = 1883
        self.client = mqtt.Client()
        self.client.on_connect = lambda client, userdata, flags, rc: self.on_connect(client, userdata, flags, rc)
        self.client.on_disconnect = lambda client, userdata, rc: self.on_disconnect(client, userdata, rc)
        self.client.on_publish = lambda client, userdata, mid: self.on_publish(client, userdata, mid)
        self.client.connect(self.broker_address, self.broker_port)
        self.client.loop_start()

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected with result code " + str(rc))

    def on_publish(self, client, userdata, mid):
        #print("Message published with mid " + str(mid))
        pass

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker")
        else:
            print("Failed to connect, return code " + str(rc))

    def send_mqtt(self, topic, data):
        if hasattr(self, 'client'):
            payload = json.dumps(data)
            self.client.publish(topic, payload)
        else:
            print("MQTT client is not initialized")


    def set_values(self):
        self.start_time = datetime.now()
        self.base_lla = get_base_lla(MAP)
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])
        self.transformer = Transformer.from_proj(proj_enu, proj_wgs84)
        # 비상 상황 식별자 추가
        self.emergency_type_arr = ['차량 사고','보행자 사고', '고장 차량', '낙하물']
        self.emergency_type = '정상'
        
    def set_protocols(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        rospy.Subscriber(f'/{self.type}/emergency_image',CompressedImage, self.emergency_image_cb)
        rospy.Subscriber = rospy.Subscriber(f'/{self.type}/dangerous_obstacle', Float32MultiArray, self.dangerous_obstacle_cb)

        rospy.loginfo("[set_protocols] Subscribing ... ")
        rospy.spin()

    def ego_share_info_cb(self, msg:ShareInfo):
        # x, y are ENU cooprdinate -> have to change to geodetic coordinate
        longitude, latitude, _ = self.transformer.transform(msg.pose.x, msg.pose.y, 7)
        yaw = msg.pose.theta
        v = msg.velocity.data

        # 비상 상황 식별자 추가
        state = msg.state.data
        if state in [4,5,6,7]:
            self.emergency_type = self.emergency_type_arr[state-4]
        elif self.emergency_type != '정상':
            self.emergency_type = '정상'


        lat_lng_data = {
            'latitude': latitude,
            'logitude':longitude,
            'heading': yaw,
            'velocity': v,
            'type': self.emergency_type
        }
        self.send_mqtt(f'/{self.type}/shar_info',lat_lng_data)

            
        #print(f"[Ego Share Info] State-> latitude: {latitude}, logitude: {longitude}, heading: {yaw}deg, velocity: {v}m/s")
        
        path = []
        for pts in msg.paths:
		#path.append([pts.pose.x, pts.pose.y])
            longitude, latitude, _ = self.transformer.transform(pts.pose.x, pts.pose.y, 7)
            path.append([longitude,latitude])
        #print(f"[Ego Share Info] Path-> start: ({path[0][0]},{path[0][1]}) ~ end:({path[-1][0]},{path[-1][1]})")

        path_data = {
            'path': path,
        }
        self.send_mqtt(f'/{self.type}/shar_info_path',path_data)

        
        obses = []
        for i, obs in enumerate(msg.obstacles):
            obs_longitude, obs_latitude, _ = self.transformer.transform(obs.pose.x, obs.pose.y, 3)
            obses.append([obs_latitude, obs_longitude, obs.pose.theta])
            #print(f"[Ego Share Info] Obstacle{i+1}-> position: ({obs_latitude},{obs_longitude}), heading: {obs.pose.theta}deg")
            obstacles_data = {
                'obs_latitude': obs_latitude,
                'obs_longitude': obs_longitude,
                'heading': f'{obs.pose.theta}deg'
            }
            self.send_mqtt(f'/{self.type}/shar_info_obstacles', obstacles_data)


         

    def communication_performance_cb(self, msg):
        state, v2x, rtt, mbps, packet_size, packet_rate, distance = msg.data
        #현재 시스템 시작 시간 사용 -> 추후 통신 시간으로 업데이트 예정 
        if distance != 0 and self.start_time is None:
            self.start_time = datetime.now()
        comulative_time = str(datetime.now() - self.start_time)
        distance = str(round(distance,5))
        rtt = str(round(rtt,5))
        speed = str(round(mbps,5))
        packet_size = str(int(packet_size))
        packet_rate = str(int(packet_rate)) if packet_rate < 100 else str(100)
        #print(f"[Ego Communication Performance] comulative_time: {comulative_time}, distance between target: {distance}\nrtt: {rtt}, speed: {speed}, packet_size: {packet_size}, packet_rate: {packet_rate}")
        performance_data = {
            'comulative_time':comulative_time,
            'distance_between_target':distance,
            'nrtt':rtt,
            'speed':speed,
            'packet_size':packet_size,
            'packet_rate':packet_rate

        }

        self.send_mqtt(f'/{self.type}/communication_performance',performance_data)


    
    def emergency_image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        _, buffer = cv2.imencode('.jpg', cv_image)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        payload = json.dumps({"image": jpg_as_text})
        #print(f"[Ego Emergency Image] Encoded to JSON")
        #print(f"data2-> {msg}")
        self.send_mqtt(f'/{self.type}/shar_info_image',{"image": jpg_as_text})


    def dangerous_obstacle_cb(self, msg):
        if len(msg.data) > 0 and self.emergency_type != '정상':
            longitude, latitude, _ = self.transformer.transform(msg.data[0], msg.data[1], 7)
            #print(f"[Emergency] {self.emergency_type} 비상 상황이 발생했습니다. 위치 -> latitude: {latitude}, logitude: {longitude}")
            dangerous_obstacle_data ={
                'emergency_type': self.emergency_type,
                'longitude': longitude,
                'latitude': latitude
            } 
            self.send_mqtt(f'/{self.type}/dangerous_obstacle',dangerous_obstacle_data)



if __name__ == "__main__":
    type = str(sys.argv[1])# sim, ego, target
    obigo_test = ObigoTest(type)