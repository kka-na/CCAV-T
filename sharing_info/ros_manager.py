#!/usr/bin/env python
import rospy
import math
import tf
import random
import os

from pyproj import Proj, Transformer
from threading import Timer

from ccavt.msg import *
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray,Bool, String


class ROSManager:
    def __init__(self, type,test,  map, oh, lpp):
        rospy.init_node(f'{type}_share_info')
        self.type = type
        self.test = test
        self.map = map
        self.oh = oh
        self.lpp = lpp
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car = {'fix': 'No','x': 0, 'y': 0, 't': 0, 'v': 0}
        self.user_input = {'state': 0, 'signal': 0, 'target_velocity': 0, 'scenario':0, 'mode':0, 'with_coop': True}
        self.lidar_obstacles = []
        self.dangerous_obstacle = []
        self.obstacle_caution = False
        self.target_info = [0,0,0, 0, 0]
        self.target_info_v2v = [0,0,0, 0, 0]  # [state, signal, velocity, x, y]
        self.target_path = []
        self.target_velocity = 0
        self.test_mode = 'same'  # test_mode 저장

        # Auto signal 관련 변수
        self.auto_signal_sent = False  # signal이 자동으로 발송되었는지 여부

        # Endpoint 관련 변수
        self.endpoints = self.load_endpoints()
        self.endpoint_threshold = 3.0  # 3m 이내면 도착으로 간주

        # Emergency 관련 변수 추가
        self.emergency_active = False
        self.emergency_cooldown = False
        self.emergency_timer = None
        self.cooldown_timer = None
        self.emergency_duration = 5.0  # 5초 동안 발행
        self.cooldown_duration = 20.0  # 50초 동안 발행 금지

        # Simulated Lidar 파라미터 (WOC 모드용)
        self.lidar_detection_range = [80, 90]  # 최소 80m, 최대 90m (랜덤)
        self.lidar_blinking_rate = 0.30  # 30% 확률로 detection miss (20Hz 중 6번 drop)

        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)



        rospy.loginfo("Sharing Info set")

    def set_protocol(self):

        if self.test == 1:
            if self.type == 'ego':
                rospy.Subscriber('/target/EgoShareInfo', ShareInfo, self.target_share_info_cb)
            else:
                rospy.Subscriber('/ego/EgoShareInfo', ShareInfo, self.target_share_info_cb)
        else:
            rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb) 

        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_inspva_cb)
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.novatel_odom_cb)
        #if self.type == 'ego':
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
        rospy.Subscriber('/simulator/object', Pose, self.simulator_object_cb )
        rospy.Subscriber(f'/{self.type}/user_input',Float32MultiArray, self.user_input_cb)
        rospy.Subscriber(f'/{self.type}/simulator/inform', Quaternion, self.simulator_inform_cb)
        rospy.Subscriber(f'/{self.type}/with_coop', String, self.with_coop_cb)
        rospy.Subscriber(f'/{self.type}/test_mode', String, self.test_mode_cb)

        self.pub_emergency_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)


        self.pub_ego_share_info = rospy.Publisher(f'/{self.type}/EgoShareInfo', ShareInfo, queue_size=1)
        self.pub_dangerous_obstacle = rospy.Publisher(f'/{self.type}/dangerous_obstacle', Float32MultiArray, queue_size=1 )
        #if self.type == 'ego':
        self.pub_obs_caution = rospy.Publisher(f'{self.type}/obs_caution', Bool, queue_size=1)
            
        self.pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)
        self.pub_inter_pt = rospy.Publisher(f'/{self.type}/inter_pt', Marker, queue_size=10)
        self.pub_bsd_zone = rospy.Publisher(f'/{self.type}/bsd_zone', Marker, queue_size=10)
        self.pub_endpoints = rospy.Publisher('/endpoints', Marker, queue_size=10, latch=True)
        self.pub_endpoint_reached = rospy.Publisher(f'/{self.type}/endpoint_reached', Bool, queue_size=1)
        self.pub_lmap_viz.publish(self.map.lmap_viz)
        self.publish_endpoints()  # Endpoint 마커 발행

    def novatel_inspva_cb(self, msg):
        self.car['fix'] = 'Ok'
        e,n,u =  self.geo2enu_transformer.transform(msg.longitude, msg.latitude, 5)
        self.car['x'] = e
        self.car['y'] = n
        self.car['t'] = 88.5-msg.azimuth
    
    def novatel_odom_cb(self, msg): 
        self.car['v'] = msg.twist.twist.linear.x

    def simulator_inform_cb(self, msg):
        self.car['fix'] = 'Ok'
        self.car['x'] = msg.x
        self.car['y'] = msg.y
        self.car['t'] = msg.w
        self.car['v'] = msg.z

    def target_share_info_cb(self, msg:ShareInfo):
        # Always store V2V data (without target_velocity, will calculate from test_mode)
        self.target_info_v2v  = [int(msg.state.data), int(msg.signal.data), float(msg.velocity.data), msg.pose.x, msg.pose.y]
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        self.target_path = path

        # WOC 모드 && 시뮬레이터 모드: Simulated Lidar 처리
        if self.test == 1 and not self.user_input.get('with_coop', True):
            self.process_simulated_lidar(msg)

        # Update target_info based on with_coop flag
        self.update_target_info() 

    def user_input_cb(self, msg):
        mode = int(msg.data[4]) if len(msg.data) > 4 else 0
        self.user_input['mode'] = mode
        if not self.emergency_active:
            self.user_input['state'] = int(msg.data[0])
            self.user_input['signal'] = int(msg.data[1])
            self.user_input['target_velocity'] = msg.data[2]
            self.user_input['scenario'] = int(msg.data[3])
        else:
            self.user_input['state'] = int(msg.data[0])
            self.user_input['target_velocity'] = msg.data[2]
            self.user_input['scenario'] = int(msg.data[3])
    
    def lidar_cluster_cb(self, msg):
        obstacles = []
        dangerous_id = 99999

        for i, obj in enumerate(msg.boxes):
            if int(obj.header.seq) < 3:
                continue
            enu = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if enu is None:
                continue
            else:
                nx, ny = enu
            lidar_delay = obj.pose.orientation.x
            quaternion = (
                0, 
                obj.pose.orientation.y, 
                obj.pose.orientation.z, 
                obj.pose.orientation.w
            )
            _, _, heading = tf.transformations.euler_from_quaternion(quaternion)
            
            distance = self.oh.distance(self.car['x'], self.car['y'], nx, ny)
            v_rel = ( obj.value if obj.value != 0 else 0 ) + self.car['v']

            obstacles.append([i, nx, ny, heading, v_rel, distance,lidar_delay])
            
        
    
        filtered_obstacles = self.lpp.phelper.filtering_by_lanelet_from_list(obstacles)
        filtered_obstacles, dangerous_id = self.oh.get_frenets_from_list(filtered_obstacles)
        
        lidar_obstacles = []
        for i, obs in enumerate(filtered_obstacles):
            if obs[0] == dangerous_id:
                dangerous = 1
                self.dangerous_obstacle = obs
            else:
                dangerous = 0
            obstacle = Obstacle()
            obstacle.cls.data = 0
            obstacle.id.data = i
            obstacle.pose.x = obs[1]
            obstacle.pose.y = obs[2]
            obstacle.pose.theta = obs[3]
            obstacle.velocity.data = obs[4]
            obstacle.distance.data = obs[5]
            obstacle.dangerous.data = dangerous
            obstacle.lidar_delay.data = obs[6]
            lidar_obstacles.append(obstacle)

        self.lidar_obstacles = lidar_obstacles
        # Update target_info if in without cooperation mode
        self.update_target_info()

    def simulator_object_cb(self, msg):
        obstacles = []
        dangerous_id = 99999
        min_s = 100
        nx = msg.position.x
        ny = msg.position.y
        heading = self.oh.refine_heading_by_lane([nx, ny])
        if heading is None:
            heading = math.degrees(msg.orientation.y)
        distance = self.oh.distance(self.car['x'], self.car['y'], nx, ny)
        v_rel = 0
        frenet = self.oh.object2frenet((nx, ny))
        if frenet is None:
            return
        else:
            s, d = frenet

        obstacles.append([0, nx, ny, heading, v_rel, distance])
        if 1 < s < 100:
            if s < min_s and -2 < d < 2 :
                dangerous_id = 0
                min_s = s
        
        lidar_obstacles = []
        dangerous_obstacle = []
        for i, obs in enumerate(obstacles):
            if obs[0] == dangerous_id:
                dangerous = 1
                dangerous_obstacle = obs
            else:
                dangerous = 0
            obstacle = Obstacle()
            obstacle.cls.data = 0
            obstacle.id.data = 0
            obstacle.pose.x = obs[1]
            obstacle.pose.y = obs[2]
            obstacle.pose.theta = obs[3]
            obstacle.velocity.data = obs[4]
            obstacle.distance.data = obs[5]
            obstacle.dangerous.data = dangerous
            obstacle.lidar_delay.data = 0.0
            lidar_obstacles.append(obstacle)

        self.lidar_obstacles = lidar_obstacles
        self.dangerous_obstacle = dangerous_obstacle
        # Update target_info if in without cooperation mode
        self.update_target_info()

    def with_coop_cb(self, msg):
        """Callback for with_coop topic"""
        self.user_input['with_coop'] = (msg.data == 'true')
        # Update target_info when with_coop flag changes
        self.update_target_info()

    def test_mode_cb(self, msg):
        self.test_mode = msg.data

        if hasattr(self, 'lpp') and self.lpp:
            self.lpp.test_mode = self.test_mode

    def get_closest_lidar_obstacle(self):
        """Find the closest lidar obstacle as 'target' for non-cooperative mode"""
        if not self.lidar_obstacles or len(self.lidar_obstacles) == 0:
            return None

        # Find closest obstacle
        closest_obstacle = None
        min_distance = float('inf')

        for obs in self.lidar_obstacles:
            distance = obs.distance.data
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = obs

        return closest_obstacle

    def build_target_info_from_lidar(self):
        """Build target_info structure from lidar obstacle data"""
        closest_obs = self.get_closest_lidar_obstacle()

        if closest_obs is None:
            # No lidar obstacles, return default
            return [0, 0, 0, 0, 0]

        # Build target_info: [state, signal, velocity, x, y]
        # For lidar-based: state=0, signal=0 (no signal info from lidar)
        target_info = [
            0,  # state (no state info from lidar)
            0,  # signal (no signal info from lidar)
            float(closest_obs.velocity.data),  # velocity from lidar
            float(closest_obs.pose.x),  # x position
            float(closest_obs.pose.y)   # y position
        ]

        return target_info

    def process_simulated_lidar(self, msg):
        """
        WOC 모드에서 시뮬레이터 데이터를 Lidar obstacle처럼 처리
        - Blinking 노이즈 시뮬레이션 (30% drop rate)
        - Detection range: 80~90m
        """
        # 1. Blinking 시뮬레이션 (30% 확률로 detection miss)
        if random.random() < self.lidar_blinking_rate:
            # Detection miss - lidar_obstacles를 비움
            self.lidar_obstacles = []
            return

        # 2. 거리 계산
        target_x = msg.pose.x
        target_y = msg.pose.y
        distance = math.sqrt((target_x - self.car['x'])**2 + (target_y - self.car['y'])**2)

        # 3. Detection range 체크 (80~90m, 매 호출마다 랜덤)
        max_range = random.uniform(self.lidar_detection_range[0], self.lidar_detection_range[1])

        if distance > max_range:
            # Range 밖 - lidar_obstacles를 비움
            self.lidar_obstacles = []
            return

        # 4. ShareInfo → Obstacle 변환
        obstacle = Obstacle()
        obstacle.cls.data = 0  # class (차량)
        obstacle.id.data = 0   # ID
        obstacle.pose.x = target_x
        obstacle.pose.y = target_y

        # Heading 정보 (ShareInfo의 pose.theta 사용)
        if hasattr(msg.pose, 'theta'):
            obstacle.pose.theta = msg.pose.theta
        else:
            obstacle.pose.theta = 0

        obstacle.velocity.data = float(msg.velocity.data)
        obstacle.distance.data = distance
        obstacle.dangerous.data = 0  # 기본적으로 non-dangerous
        obstacle.lidar_delay.data = 0.0  # 시뮬레이터 데이터이므로 delay 없음

        # 5. lidar_obstacles 업데이트 (리스트로)
        self.lidar_obstacles = [obstacle]

    def update_target_info(self):
        """Update target_info based on with_coop flag"""
        if self.user_input.get('with_coop', True):
            # With cooperation: use V2V data
            self.target_info = self.target_info_v2v
        else:
            # Without cooperation: use lidar data
            self.target_info = self.build_target_info_from_lidar()

    def stop_emergency_publishing(self):
        """5초 emergency 발행 종료 후 50초 쿨다운 시작"""
        self.emergency_active = False
        self.emergency_cooldown = True
        rospy.loginfo("Emergency publishing stopped - starting 20s cooldown")
        
        # 50초 쿨다운 타이머 시작
        self.cooldown_timer = Timer(self.cooldown_duration, self.reset_emergency_cooldown)
        self.cooldown_timer.start()

    def reset_emergency_cooldown(self):
        """50초 쿨다운 해제"""
        self.emergency_cooldown = False
        rospy.loginfo("Emergency cooldown finished - ready to publish emergency again")


    def calc_world_pose(self, x, y):
        la, ln, al = self.enu2geo_transformter.transform(x, y, 5)
        return [la, ln]

    def organize_share_info(self, _path, _target_velocity, merge_safety):
        share_info = ShareInfo()
        if self.car['fix'] == 'No':
            return share_info
        share_info.state.data = int(self.user_input['state'])
        if self.type == 'target':
            if merge_safety != 0:
                share_info.signal.data = 4 if merge_safety == 1 else 5
            else:
                share_info.signal.data = int(self.user_input['signal'])
        else:
            share_info.signal.data = int(self.user_input['signal'])

        target_vel = _target_velocity
        share_info.target_velocity.data = target_vel
        share_info.pose.x = self.car['x']
        share_info.pose.y = self.car['y']
        share_info.pose.theta = self.car['t']
        share_info.velocity.data = self.car['v']
        if _path != None:
            for xy in _path:
                path = Path()
                path.pose.x = xy[0]
                path.pose.y = xy[1]
                share_info.paths.append(path)
        for obstacle in self.lidar_obstacles:           
            share_info.obstacles.append(obstacle)
        
        return share_info
    
    def publish(self, lpp_res, vp_res):
        share_info = self.organize_share_info(lpp_res[1],vp_res,lpp_res[5])
        self.pub_ego_share_info.publish(share_info)
        if self.type == 'ego':
            self.pub_obs_caution.publish(Bool(lpp_res[4]))
        self.pub_dangerous_obstacle.publish(Float32MultiArray(data=list(self.dangerous_obstacle)))
    
    def publish_inter_pt(self, inter_pt):
        # WC 모드(With Communication)일 때만 merge point 표시
        if not self.user_input.get('with_coop', True):
            # WOC 모드면 마커 삭제
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = 1
            marker.ns = 'intersection'
            self.pub_inter_pt.publish(marker)
            return

        if inter_pt is not None:
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.header.frame_id = 'world'
            marker.ns = 'intersection'
            marker.id = 1
            marker.lifetime = rospy.Duration(0)
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.r = 33/255
            marker.color.g = 255/255
            marker.color.b = 144/255
            marker.color.a = 0.7
            marker.pose.position.x = inter_pt[0]
            marker.pose.position.y = inter_pt[1]
            marker.pose.position.z = 1.0
            self.pub_inter_pt.publish(marker)
        else:
            # inter_pt가 None이면 마커 삭제
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = 1
            marker.ns = 'intersection'
            self.pub_inter_pt.publish(marker)

    def publish_bsd_zone(self, bsd_info):
        """
        BSD zone 시각화
        bsd_info: {'active': bool, 'signal': int, 'bsd_range': list, 'local_path': list}
        """
        if not bsd_info or not bsd_info.get('active', False):
            # WOC 모드가 아니거나 차선 변경 신호가 없으면 삭제
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = 1
            marker.ns = 'bsd_zone'
            self.pub_bsd_zone.publish(marker)
            return

        signal = bsd_info.get('signal', 0)
        if signal not in [1, 2]:  # 차선 변경 신호가 없으면 표시 안함
            return

        bsd_range = bsd_info.get('bsd_range', [-12, 10, 1.5, 6.0])
        ts_rear, ts_front, td_min, td_max = bsd_range

        # Ego 차량 위치와 heading
        ego_x = self.car['x']
        ego_y = self.car['y']
        ego_heading = math.radians(self.car['t'])

        # BSD zone을 LINE_STRIP으로 그리기
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'bsd_zone'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)  # 0.1초 후 자동 삭제

        # 색상 설정 (signal에 따라)
        if signal == 1:  # 좌회전 - 파란색
            marker.color.r = 0.2
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.5
            lateral_offset = 1  # 왼쪽
        else:  # 우회전 - 노란색
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.5
            lateral_offset = -1  # 오른쪽

        marker.scale.x = 0.3  # 선 두께

        # BSD zone 박스의 네 모서리 계산
        # 차량 좌표계: x=앞, y=왼쪽
        from geometry_msgs.msg import Point

        # 박스 모서리 (차량 좌표계)
        corners = [
            (ts_rear, lateral_offset * td_min),  # 뒤-안쪽
            (ts_front, lateral_offset * td_min), # 앞-안쪽
            (ts_front, lateral_offset * td_max), # 앞-바깥쪽
            (ts_rear, lateral_offset * td_max),  # 뒤-바깥쪽
            (ts_rear, lateral_offset * td_min),  # 닫기
        ]

        # 차량 좌표계 -> 월드 좌표계 변환
        for local_x, local_y in corners:
            # 회전 + 이동
            world_x = ego_x + local_x * math.cos(ego_heading) - local_y * math.sin(ego_heading)
            world_y = ego_y + local_x * math.sin(ego_heading) + local_y * math.cos(ego_heading)

            point = Point()
            point.x = world_x
            point.y = world_y
            point.z = 0.5  # 지면에서 약간 위
            marker.points.append(point)

        self.pub_bsd_zone.publish(marker)


    def publish_emergency(self, emergency):
        """
        Emergency 발행 - 단순 버전
        """
        if self.user_input['scenario'] >= 7:
            
            if emergency == 'emergency':
                # 쿨다운 중이면 무시
                if self.emergency_cooldown:
                    return
                
                # 처음 emergency면 타이머 시작
                if not self.emergency_active:
                    self.emergency_active = True
                    rospy.loginfo("Emergency started")
                    
                    # 5초 후 중단
                    if self.emergency_timer:
                        self.emergency_timer.cancel()
                    self.emergency_timer = Timer(7.0, self.stop_emergency)
                    self.emergency_timer.start()
                
                # signal=7로 발행
                self.publish_signal(7)
                
            # else:  # 'normal'
            #     # 모든 상태 리셋
            #     self.reset_emergency()
            #     # signal=0으로 발행
            #     self.publish_signal(0)

    def publish_signal(self, signal_value):
        """signal 값으로 메시지 발행"""
        msg = Float32MultiArray()
        msg.data = [
            float(self.user_input['state']),
            float(signal_value),
            float(self.user_input['target_velocity']),
            float(self.user_input['scenario'])
        ]
        self.user_input['signal'] = signal_value
        self.pub_emergency_user_input.publish(msg)

    def stop_emergency(self):
        """5초 후 emergency 중단"""
        self.emergency_active = False
        self.emergency_cooldown = True
        rospy.loginfo("Emergency stopped - cooldown started")
        
        # signal=0으로 변경
        self.publish_signal(0)
        
        # 50초 후 쿨다운 해제
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
        self.cooldown_timer = Timer(20.0, self.end_cooldown)
        self.cooldown_timer.start()

    def end_cooldown(self):
        """쿨다운 해제"""
        self.emergency_cooldown = False
        rospy.loginfo("Cooldown ended")

    def reset_emergency(self):
        """Emergency 상태 완전 리셋"""
        self.emergency_active = False
        self.emergency_cooldown = False

        if self.emergency_timer:
            self.emergency_timer.cancel()
        if self.cooldown_timer:
            self.cooldown_timer.cancel()

    def check_auto_signal(self):
        """
        두 차량이 목표 속도에 도달하면 자동으로 signal 발송
        CLM 시나리오별로 다른 signal 발송
        CLM3, 4: target signal 이후 ego signal 발송
        """
        # 이미 signal이 발송되었으면 리턴
        if self.auto_signal_sent:
            return

        # 시나리오가 CLM(1-6)이 아니면 리턴
        scenario = self.user_input.get('scenario', 0)
        if scenario < 1 or scenario > 6:
            return

        # 자신의 속도와 목표 속도
        my_velocity = self.car['v'] * 3.6  # m/s → km/h
        my_target_velocity = self.user_input['target_velocity'] * 3.6  # m/s → km/h

        # 상대방의 속도 (target_info_v2v에서 가져옴)
        if len(self.target_info_v2v) < 3:
            return

        other_velocity = self.target_info_v2v[2] * 3.6  # m/s → km/h

        # test_mode 기반으로 상대방의 목표 속도 계산
        # ego 입장: slower = target 느림(-5), same = 동일, faster = target 빠름(+5)
        # target 입장: slower = ego 빠름(+5), same = 동일, faster = ego 느림(-5)
        speed_offset_map = {
            'WC_slower': -5.0 if self.type == 'ego' else 5.0,
            'WC_same': 0.0,
            'WC_faster': 5.0 if self.type == 'ego' else -5.0,
            'WOC_slower': -5.0 if self.type == 'ego' else 5.0,
            'WOC_same': 0.0,
            'WOC_faster': 5.0 if self.type == 'ego' else -5.0,
        }
        speed_offset = speed_offset_map.get(self.test_mode, 0.0)
        other_target_velocity = my_target_velocity + speed_offset
        # 두 차량 모두 목표 속도 ±1km/h 도달 체크
        my_reached = abs(my_velocity - my_target_velocity) <= 1.0
        other_reached = abs(other_velocity - other_target_velocity) <= 1.0
        # CLM3, 4: ego는 target signal을 받은 후에 signal 발송
        if scenario in [3, 4] and self.type == 'ego':
            # Ego는 자신의 속도만 도달하면 되고, target signal을 기다림
            if not my_reached:
                return
            # Target signal 체크 (1 또는 2가 들어와야 함)
            target_signal = self.target_info_v2v[1] if len(self.target_info_v2v) > 1 else 0
            if target_signal not in [1, 2]:
                return  # Target signal이 아직 안 들어옴
        else:
            # 다른 시나리오 또는 target: 두 차량 모두 목표 속도 도달 필요
            if not (my_reached and other_reached):
                return

        # 시나리오별 signal 결정
        signal_map = {
            # scenario: {type: signal}
            1: {'ego': 2, 'target': None},
            2: {'ego': 1, 'target': None},
            3: {'ego': 2, 'target': 1},
            4: {'ego': 1, 'target': 2},  # CLM4: ego=right(2), target=left(1)
            5: {'ego': 2, 'target': None},
            6: {'ego': 2, 'target': None},
        }

        signal_to_send = signal_map.get(scenario, {}).get(self.type, None)

        if signal_to_send is not None:
            # Signal 자동 발송
            self.user_input['signal'] = signal_to_send
            self.auto_signal_sent = True
            rospy.loginfo(f"[AUTO SIGNAL] {self.type} sent signal {signal_to_send} for CLM{scenario} (test_mode={self.test_mode})")

            # 3초 후 signal 자동 리셋
            def reset_signal():
                self.user_input['signal'] = 0
                rospy.loginfo(f"[AUTO SIGNAL] {self.type} signal reset to 0")

            Timer(3.0, reset_signal).start()

    def load_endpoints(self):
        """ui/yaml/end_point.yaml에서 endpoint 좌표 로드"""
        import yaml
        import os
        yaml_path = os.path.join(os.path.dirname(__file__), '../ui/yamls/end_point.yaml')
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            endpoints = []
            for key in ['point1', 'point2', 'point3', 'point4', 'point5', 'point6']:
                if key in data:
                    endpoints.append(data[key])
            rospy.loginfo(f"[ENDPOINT] Loaded {len(endpoints)} endpoints")
            return endpoints
        except Exception as e:
            rospy.logwarn(f"[ENDPOINT] Failed to load endpoints: {e}")
            return []

    def check_endpoint(self):
        """차량이 endpoint에 도달했는지 확인"""
        if not self.endpoints:
            return False

        car_x = self.car['x']
        car_y = self.car['y']

        for i, ep in enumerate(self.endpoints):
            dist = math.sqrt((car_x - ep[0])**2 + (car_y - ep[1])**2)
            if dist < self.endpoint_threshold:
                rospy.loginfo(f"[ENDPOINT] {self.type} reached endpoint {i+1}")
                # UI에 endpoint 도달 알림
                self.pub_endpoint_reached.publish(Bool(data=True))
                return True
        return False

    def publish_endpoints(self):
        """Endpoint 위치를 마커로 시각화"""
        if not self.endpoints:
            return

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "endpoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 3.0
        marker.scale.y = 3.0
        marker.scale.z = 3.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        for ep in self.endpoints:
            p = Point()
            p.x = ep[0]
            p.y = ep[1]
            p.z = 0.0
            marker.points.append(p)

        self.pub_endpoints.publish(marker)
