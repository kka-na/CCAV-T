#!/usr/bin/env python
import rospy
from pyproj import Proj, Transformer

from ccavt.msg import *
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray,Bool
    
class ROSManager:
    def __init__(self, type, map, oh):
        rospy.init_node(f'{type}_share_info')
        self.type = type
        self.map = map
        self.oh = oh
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car = {'fix': 'No','x': 0, 'y': 0, 't': 0, 'v': 0}
        self.user_input = {'state': 0, 'signal': 0, 'target_velocity': 10/3.6, 'scenario_type':1, 'scenario_number':1}
        self.lidar_obstacles = []
        self.dangerous_obstacle = []
        self.obstacle_caution = False
        self.target_info = [0,0,0]
        self.target_path = []

        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb) 
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_inspva_cb)
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.novatel_odom_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
        rospy.Subscriber(f'/{self.type}/user_input',Float32MultiArray, self.user_input_cb)
        rospy.Subscriber('/simulator/inform', Quaternion, self.simulator_inform_cb)

        self.pub_ego_share_info = rospy.Publisher(f'/{self.type}/EgoShareInfo', ShareInfo, queue_size=1)
        if self.type == 'ego':
            self.pub_obs_caution = rospy.Publisher(f'{self.type}/obs_caution', Bool, queue_size=1)
        self.pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)
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
        self.target_info  = [int(msg.state.data), int(msg.signal.data), float(msg.velocity.data)]
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        self.target_path = path 

    def user_input_cb(self, msg):
        self.user_input['state'] = int(msg.data[0])
        self.user_input['signal'] = int(msg.data[1])
        self.user_input['target_velocity'] = msg.data[2]
        self.user_input['scenario_type'] = int(msg.data[3])
        self.user_input['scenario_number'] = int(msg.data[4])

    def lidar_cluster_cb(self, msg):
        obstacles = []
        dangerous_id = 0
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

    def calc_world_pose(self, x, y):
        la, ln, al = self.enu2geo_transformter.transform(x, y, 5)
        return [la, ln]

    def organize_share_info(self, _path, merge_safety):
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


        share_info.target_velocity.data = float(self.user_input['target_velocity'])
        share_info.pose.x = self.car['x']
        share_info.pose.y = self.car['y']
        share_info.pose.theta = self.car['t']
        share_info.velocity.data = self.car['v']
        if _path != None:
            share_info.path_len.data = int(len(_path))
            for xy in _path:
                path = Path()
                path.pose.x = xy[0]
                path.pose.y = xy[1]
                share_info.paths.append(path)
        for obstacle in self.lidar_obstacles:           
            share_info.obstacles.append(obstacle)
        
        return share_info
    
    def publish(self, lpp_res):
        share_info = self.organize_share_info(lpp_res[1],lpp_res[5])
        self.pub_ego_share_info.publish(share_info)
        if self.type == 'ego':
            self.pub_obs_caution.publish(Bool(lpp_res[4]))