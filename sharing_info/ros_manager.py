#!/usr/bin/env python
import rospy
import math

from pyproj import Proj, Transformer
from threading import Timer

from ccavt.msg import *
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray,Bool, String


class ROSManager:
    def __init__(self, type,test,  map, oh):
        rospy.init_node(f'{type}_share_info')
        self.type = type
        self.test = test
        self.map = map
        self.oh = oh
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car = {'fix': 'No','x': 0, 'y': 0, 't': 0, 'v': 0}
        self.user_input = {'state': 0, 'signal': 0, 'target_velocity': 30/3.6, 'scenario':0}
        self.lidar_obstacles = []
        self.dangerous_obstacle = []
        self.obstacle_caution = False
        self.target_info = [0,0,0, 0, 0]
        self.target_path = []
        self.target_velocity = 0

        # Emergency 관련 변수 추가
        self.emergency_active = False
        self.emergency_cooldown = False
        self.emergency_timer = None
        self.cooldown_timer = None
        self.emergency_duration = 5.0  # 5초 동안 발행
        self.cooldown_duration = 20.0  # 50초 동안 발행 금지

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
        if self.type == 'ego':
            rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
            rospy.Subscriber('/simulator/object', Pose, self.simulator_object_cb )
        rospy.Subscriber(f'/{self.type}/user_input',Float32MultiArray, self.user_input_cb)
        rospy.Subscriber(f'/{self.type}/simulator/inform', Quaternion, self.simulator_inform_cb)

        self.pub_emergency_user_input = rospy.Publisher(f'/{self.type}/user_input', Float32MultiArray, queue_size=1)


        self.pub_ego_share_info = rospy.Publisher(f'/{self.type}/EgoShareInfo', ShareInfo, queue_size=1)
        self.pub_dangerous_obstacle = rospy.Publisher(f'/{self.type}/dangerous_obstacle', Float32MultiArray, queue_size=1 )
        if self.type == 'ego':
            self.pub_obs_caution = rospy.Publisher(f'{self.type}/obs_caution', Bool, queue_size=1)
            
        self.pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)
        self.pub_inter_pt = rospy.Publisher(f'/{self.type}/inter_pt', Marker, queue_size=10)
        self.pub_lmap_viz.publish(self.map.lmap_viz)

    def novatel_inspva_cb(self, msg):
        self.car['fix'] = 'Ok'
        e,n,u =  self.geo2enu_transformer.transform(msg.longitude, msg.latitude, 5)
        self.car['x'] = e
        self.car['y'] = n
        self.car['t'] = 90-msg.azimuth
    
    def novatel_odom_cb(self, msg): 
        self.car['v'] = msg.twist.twist.linear.x

    def simulator_inform_cb(self, msg):
        self.car['fix'] = 'Ok'
        self.car['x'] = msg.x
        self.car['y'] = msg.y
        self.car['t'] = msg.w
        self.car['v'] = msg.z

    def target_share_info_cb(self, msg:ShareInfo):
        self.target_info  = [int(msg.state.data), int(msg.signal.data), float(msg.velocity.data),msg.pose.x, msg.pose.y]
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        self.target_path = path 

    def user_input_cb(self, msg):
        if not self.emergency_active:
            self.user_input['state'] = int(msg.data[0])
            self.user_input['signal'] = int(msg.data[1])
            #self.user_input['target_velocity'] = msg.data[2]
            self.user_input['scenario'] = int(msg.data[3])
        else:
            self.user_input['state'] = int(msg.data[0])
            #self.user_input['target_velocity'] = msg.data[2]
            self.user_input['scenario'] = int(msg.data[3])
    
    def lidar_cluster_cb(self, msg):
        obstacles = []
        dangerous_id = 99999
        min_s = 100
        for i, obj in enumerate(msg.boxes):
            if int(obj.header.seq) < 3:
                continue
            enu = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if enu is None:
                continue
            else:
                nx, ny = enu
            
            heading = self.oh.refine_heading_by_lane([nx, ny])
            if heading is None:
                continue
            distance = self.oh.distance(self.car['x'], self.car['y'], nx, ny)
            v_rel = ( obj.value if obj.value != 0 else 0 ) + self.car['v']

            frenet = self.oh.object2frenet(enu)
            if frenet is None:
                continue
            else:
                s, d = frenet
            
            obstacles.append([i, nx, ny, heading, v_rel, distance])

            if 0 < s < 100:
                if s < min_s and -1 < d < 1 :
                    dangerous_id = i
                    min_s = s
                    
        lidar_obstacles = []
        for i, obs in enumerate(obstacles):
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
            lidar_obstacles.append(obstacle)

        self.lidar_obstacles = lidar_obstacles
    
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
            if s < min_s and -1 < d < 1 :
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
            lidar_obstacles.append(obstacle)

        self.lidar_obstacles = lidar_obstacles
        self.dangerous_obstacle = dangerous_obstacle

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

        share_info.target_velocity.data = _target_velocity
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
    
    
    def publish_emergency(self, emergency):
        """
        Emergency 발행 - 단순 버전
        """
        if self.user_input['scenario'] >= 4:
            
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
            self.user_input['target_velocity'],
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

