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
    def __init__(self, x, y, yaw, vehicle_type='ego'):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0
        self.L = 2.97  # Wheelbase (m)
        self.vehicle_type = vehicle_type

        # 차량 타입별 동역학 파라미터
        if vehicle_type == 'ego':
            # 전기차: 0→30km/h (8.33m/s) in 3초
            self.max_accel = 2.58  # m/s²
            self.max_decel = 4.5   # m/s² (브레이크 성능)
            self.accel_response = 0.5 # 가속 응답성 (0~1, 높을수록 빠름)
        else:  # target
            # 내연기관차: 0→30km/h (8.33m/s) in 4초
            self.max_accel = 2.08  # m/s²
            self.max_decel = 4.0   # m/s²
            self.accel_response = 0.6  # 내연기관은 응답이 느림

    def set(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def next_state(self, dt, actuator):
        # 위치 및 방향 업데이트 (kinematic bicycle model)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * dt * math.tan(actuator['steer']) / self.L
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        # 목표 가속도 계산
        desired_accel = 0
        if actuator['accel'] > 0 and actuator['brake'] == 0:
            # 가속 요청: actuator['accel']은 0~1 정규화된 값으로 가정
            desired_accel = actuator['accel'] * self.max_accel
        elif actuator['accel'] == 0 and actuator['brake'] >= 0:
            # 감속 요청
            desired_accel = -actuator['brake'] * self.max_decel

        # 1차 지연 시스템으로 실제 가속도 계산 (차량 관성 반영)
        # v_new = v_old + accel_response * desired_accel * dt
        actual_accel = self.accel_response * desired_accel

        # 속도 업데이트
        new_v = self.v + actual_accel * dt
        self.v = max(0, new_v)  # 속도는 0 이하로 내려가지 않음

        return self.x, self.y, self.yaw, self.v

class Simulator:
    # Class variable to cache YAML config (shared across all instances)
    _ego_pose_config = None
    _config_loaded = False

    def __init__(self, type, map):
        self.ego = None
        self.type = type
        self.map = map
        self.car = {'state':0, 'x': 0, 'y':0,'t': 0,'v': 0}
        self.actuator = {'steer': 0, 'accel': 0, 'brake': 0}
        self.obstacles = []

        self.scenario = 0
        self.test_mode = 'same'  # Default test mode: slower, same, faster

        # Load config once for all instances
        self._load_config_once()

        self.set_ego()
        self.set_protocol(type)

    def _load_config_once(self):
        """Load YAML config only once and cache it"""
        if not Simulator._config_loaded:
            with open("./transmitter/yamls/ego_pose.yaml", "r") as f:
                Simulator._ego_pose_config = yaml.safe_load(f)
            Simulator._config_loaded = True
            print(f"[Simulator] Loaded ego_pose.yaml (cached for all instances)")

    def set_protocol(self,type):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.simulator_pub = rospy.Publisher(f'/{type}/simulator/inform', Quaternion, queue_size=1, latch=True)
        # Minimal wait for publisher to be ready (reduced from 0.1s to 0.02s)
        rospy.sleep(0.02)
        # Publish initial pose immediately to avoid startup delay
        self.publish_initial_pose()

    def publish_initial_pose(self):
        """Publish initial ego pose immediately after initialization"""
        quat = Quaternion()
        quat.x = self.ego.x
        quat.y = self.ego.y
        quat.z = self.ego.v
        quat.w = math.degrees(self.ego.yaw)
        self.simulator_pub.publish(quat)

        # Update car state
        self.car['x'] = self.ego.x
        self.car['y'] = self.ego.y
        self.car['v'] = self.ego.v
        self.car['t'] = math.degrees(self.ego.yaw)

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
        scenario = int(msg['scenario'])

        # Get test_mode from msg (if available)
        # msg should now include test_mode as a string field
        test_mode = msg.get('test_mode', 'same')  # Default to 'same' if not provided

        # Update ego position if scenario or test_mode changed
        if self.scenario != scenario or self.test_mode != test_mode:
            self.scenario = scenario
            self.test_mode = test_mode
            self.set_ego()
            # Publish updated pose immediately
            self.publish_initial_pose() 

    def set_ego(self):
        # Use cached config instead of reading file every time
        config = Simulator._ego_pose_config
        if config is None:
            raise RuntimeError("Config not loaded! Call _load_config_once() first")

        map_config = config.get(self.map, {})

        # Build scenario key based on map and scenario number
        # For Midan map with CLM scenarios (1-6), use "CLM{num}_{test_mode}" format
        if self.map == 'Midan' and 1 <= self.scenario <= 6:
            scenario_key = f"CLM{self.scenario}_{self.test_mode}"
        elif self.map == 'Midan' and 7 <= self.scenario <= 12:
            # For ETrA scenarios (7-12), use "ETrA{num}" format (no test_mode)
            etra_num = self.scenario - 6  # 7->1, 8->2, ..., 12->6
            scenario_key = f"ETrA{etra_num}"
        else:
            # For other scenarios, use the number as is
            scenario_key = self.scenario

        scenario_data = map_config.get(scenario_key, map_config.get("default", {}))
        type_data = scenario_data.get(self.type, {})
        ego_pose = type_data.get("ego", [0,0,0])
        # Vehicle 생성 시 vehicle_type 전달 (ego 또는 target)
        self.ego = Vehicle(*ego_pose, vehicle_type=self.type)
        self.obstacles = scenario_data.get("obstacles", [])
    
    async def execute(self):
        dt = 0.075
        self.car['x'], self.car['y'], yaw, self.car['v'] = self.ego.next_state(dt, self.actuator)
        self.car['t'] = math.degrees(yaw)
    
    def cleanup(self):
        pass