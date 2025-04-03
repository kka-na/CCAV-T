#!/usr/bin/python
import tf
import yaml

import math
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

def signal_handler(sig, frame):
    sys.exit(0)

class Vehicle:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0
        self.L = 2.97

    def set(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def next_state(self, dt, actuator):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * dt * math.tan(actuator['steer']) / self.L
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        tar_v = self.v

        if actuator['accel'] > 0 and actuator['brake'] == 0:
            tar_v += actuator['accel'] * dt
        elif actuator['accel'] == 0 and actuator['brake'] >= 0:
            tar_v += -actuator['brake'] * dt
        self.v = max(0, tar_v)
        return self.x, self.y, self.yaw, self.v

class Simulator:
    def __init__(self, type, map, scenario):
        self.ego = None
        self.type = type
        self.scenario = scenario
        self.car = {'state':0, 'x': 0, 'y':0,'t': 0,'v': 0}
        self.actuator = {'steer': 0, 'accel': 0, 'brake': 0}
        self.obstacles = []

        self.set_ego(map)
        self.set_protocol(type)
    
    def set_protocol(self,type):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.simulator_pub = rospy.Publisher(f'/{type}/simulator/inform', Quaternion, queue_size=1)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.ego.set(x, y, yaw)

    def set_actuator(self, msg):
        self.actuator['steer'] = math.radians(msg[1])
        if msg[0] > 0:
            accel = msg[0]
            brake = 0
        else:
            accel = 0
            brake = -msg[0]
        
        self.actuator['accel']= accel
        self.actuator['brake'] = brake

        quat = Quaternion()
        quat.x = self.car['x']
        quat.y = self.car['y']
        quat.z = self.car['v']
        quat.w = self.car['t']

        self.simulator_pub.publish(quat)
    
    def set_user_input(self, msg):
        pass

    def set_ego(self, map):
        with open("./transmitter/yaml/ego_pose.yaml", "r") as f:
            config = yaml.safe_load(f)
        
        map_config = config.get(map, {})
        scenario_data = map_config.get(self.scenario, map_config.get("default", {}))
        type_data = scenario_data.get(self.type, {})
        ego_pose = type_data.get("ego", [0,0,0])
        self.ego = Vehicle(*ego_pose)
        self.obstacles = scenario_data.get("obstacles", [])
    
    async def execute(self):
        dt = 0.02
        self.car['x'], self.car['y'], yaw, self.car['v'] = self.ego.next_state(dt, self.actuator)
        self.car['t'] = math.degrees(yaw)
    
    def cleanup(self):
        pass