import numpy as np
import math

import time 

class ObstacleHandler:
    def __init__(self, phelper):
        self.setting_values(phelper)

    def setting_values(self, phelper):
        self.phelper = phelper
        self.local_pose = None
        self.local_path = None 
        self.prev_local_pose = None
        self.lidar_obstacles = None
        self.current_heading = 0.0

        self.stopped_vehicle_start_time = None
        self.emergency_threshold = 0.3  # 3초

    def update_value(self, car, local_path, lidar_obstacles):
        self.local_pose = [car['x'], car['y']]
        self.current_heading = car['t']
        if self.prev_local_pose is None:
            self.prev_local_pose = self.local_pose
        if local_path is not None:
            self.local_path = local_path
        self.lidar_obstacles = lidar_obstacles
        
    def check_dimension(self, dimension):
        if dimension[0] > 2 and dimension[1] > 1.5 :
            return True
        else:
            return False

    def object2enu(self,  obs_pose):
        rad = np.radians(self.current_heading)

        nx = math.cos(rad) * obs_pose[0] - math.sin(rad) * obs_pose[1]
        ny = math.sin(rad) * obs_pose[0] + math.cos(rad) * obs_pose[1]

        obj_x = self.local_pose[0] + nx
        obj_y = self.local_pose[1] + ny

        return obj_x, obj_y
    
    def filtering_by_lane_num(self, lane_num, fred_d):
        if lane_num == 1:
            if -7.15 < fred_d < 1.45:
                return True
            else:
                return False
        elif lane_num == 2:
            if -1.45 < fred_d < 4.15:
                return True
            else:
                return False
        elif lane_num == 3:
            if -4.15 < fred_d < 7.15:
                return True
            else:
                return False
        elif lane_num == 4 or lane_num == 5:
            if -1.45 < fred_d < 10.15 :
                return True
            else:
                return False
        else:
            if -1.45< fred_d < 4.15:
                return True
            else:
                return False

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2    )

    def object2frenet(self, obs_pose):
        trim_path = self.local_path

        if trim_path is None:
            return None
        
        if len(trim_path) > 0:  
            centerline = np.array([(point[0], point[1]) for point in trim_path])
            if centerline.shape[0] < 2:
                return None  # Or handle this case accordingly

            point = np.array(obs_pose)

            tangents = np.gradient(centerline, axis=0)
            tangents = tangents / np.linalg.norm(tangents, axis=1)[:, np.newaxis]
            
            normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
            
            distances = np.linalg.norm(centerline - point, axis=1)
            
            closest_index = np.argmin(distances)
            closest_point = centerline[closest_index]
            tangent = tangents[closest_index]
            normal = normals[closest_index]
            
            vector_to_point = point - closest_point
            d = np.dot(vector_to_point, normal)

            s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=1))

            vector_from_start = point - centerline[0]  
            if np.dot(tangents[0], vector_from_start) < 0:  
                s = -np.linalg.norm(vector_from_start)  

            return s, d
        else:
            return None
    
    def refine_heading_by_lane(self, obs_pos):
        idnidx = self.phelper.lanelet_matching2(obs_pos)
        if idnidx is not None:
            waypoints = self.phelper.lanelets[idnidx[0]]['waypoints']
            next_idx = idnidx[1]+3 if idnidx[1]+3 < len(waypoints)-4 else len(waypoints)-4
            
            prev_point = waypoints[idnidx[1]]
            next_point = waypoints[next_idx]

            delta_x = next_point[0] - prev_point[0]
            delta_y = next_point[1] - prev_point[1]
            
            heading = math.degrees(math.atan2(delta_y, delta_x))

            return heading
        else:
            return None
    
    def check_emergency(self, obs):
        # obs[4]가 velocity라고 가정
        if len(obs) < 6:  # obs[5]까지 필요하므로 6개 이상 확인
            #print(f"[DEBUG] obs 길이 부족: {len(obs)} < 6")
            return 'normal'

        velocity = obs[4]
        distance = obs[5]
        stopped_threshold = 1  # 정지 상태로 간주할 속도 임계값
        
        #print(f"[DEBUG] 현재 상태 - velocity: {velocity:.2f}, distance: {distance:.2f}")
        
        if abs(velocity) < stopped_threshold or distance < 55:
            # 차량이 멈춘 상태
            current_time = time.time()  # 또는 rospy.Time.now().to_sec()
            
            if self.stopped_vehicle_start_time is None:
                # 처음 멈춘 것을 감지
                self.stopped_vehicle_start_time = current_time
                #print(f"[DEBUG] 정지 상태 감지 시작 - 시간: {current_time:.2f}")
            else:
                # 멈춘 시간 계산
                stopped_duration = current_time - self.stopped_vehicle_start_time
                #print(f"[DEBUG] 정지 지속 시간: {stopped_duration:.2f}초 / 임계값: {self.emergency_threshold}초")
                
                if stopped_duration >= self.emergency_threshold:
                    #print(f"[DEBUG] 🚨 응급상황 감지! 정지 시간 {stopped_duration:.2f}초 초과")
                    return "emergency"
        else:
            # 차량이 움직이고 있으면 타이머 리셋
            if self.stopped_vehicle_start_time is not None:
                print(f"[DEBUG] 차량 이동 감지 - 타이머 리셋")
            self.stopped_vehicle_start_time = None
        
        #print(f"[DEBUG] 정상 상태 반환")
        return "normal"