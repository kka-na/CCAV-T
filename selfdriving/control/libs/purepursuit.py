import numpy as np
import math
MPS_TO_KPH = 3.6

class PurePursuit(object):
    def __init__(self, config):
        self.set_configs(config)
        self.prev_steer = 0

    def set_configs(self, config):
        pp_config = config['PurePursuit']
        self.lfd_gain = float(pp_config['lfd_gain'])
        self.min_lfd = float(pp_config['min_lfd'])
        self.max_lfd = float(pp_config['max_lfd'])
        self.lfd_offset = float(pp_config['lfd_offset'])
        cm_config = config['Common']
        self.wheelbase = float(cm_config['wheelbase'])
        self.steer_ratio = float(cm_config['steer_ratio'])
        self.steer_max = float(cm_config['steer_max'])
        self.saturation_th = float(cm_config['saturation_th'])

    def execute(self, current_location, state, local_path, current_heading, current_velocity):
        if len(current_location) < 1:
            return 0, [current_location.x, current_location.y]
        lfd = self.lfd_gain * current_velocity * MPS_TO_KPH

        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        point = current_location
        route = local_path
        heading = math.radians(current_heading)
        
        steering_angle = 0.
        lh_point = point

        for i, path_point in enumerate(route):
            diff = path_point - point
            rotated_diff = diff.rotate(-heading)
            if rotated_diff.x > 0:
                dis = rotated_diff.distance()
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd*self.lfd_offset)
                    lh_point = path_point
                    break

        steering_angle = math.degrees(steering_angle)
        steering_angle = max(-15, min(steering_angle, 15))
        steer_offset = min(max(current_velocity * MPS_TO_KPH * 0.02 + 0.1, 1), 2.2)
        if current_velocity * MPS_TO_KPH > 30:
            steering_angle = steering_angle * steer_offset
        steer = np.clip(steering_angle*self.steer_ratio, -500, 500)
        return steer,(lh_point.x, lh_point.y)