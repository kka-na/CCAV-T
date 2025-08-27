# side 2.5-3.0m
# rear 3-9m
# ttz d/v_rel (threshold 3-5sec)

class VelocityPlanner:
    def __init__(self, type):
        self.type = type
        self.setting_values()
    
    def setting_values(self):
        self.max_vel = 0
        self.ego_sig = 0
        self.ego_vel = 0
        self.tar_vel = 0
        self.tar_sig = 0
        self.temp_vel = self.max_vel
        self.temp_sig = self.tar_sig
        self.decel_value = 0.05
        self.decel_count = 0
        self.accel_count = 0

    
    def update_value(self, user_input, ego, target):
        self.max_vel = user_input['target_velocity']
        self.ego_sig = user_input['signal']
        self.ego_vel = ego['v']
        self.tar_vel = target[2]
        self.tar_sig = target[1]
        
    
    def execute(self, lpp_result):
        vel = self.max_vel
    
        if self.type == 'ego':
            # if deny the lane change
            if self.tar_sig == 5 or self.temp_sig == 5:
                # 10번까지만 감속
                vel = self.max_vel - 1.5
                self.decel_count += 1  # 감속 카운터 증가
                self.temp_sig = self.tar_sig

            elif self.tar_sig == 0 or self.tar_sig == 4:
                if 10 < self.decel_count < 30:
                    vel = self.max_vel - 1
                    self.decel_count += 1
                else:
                    vel = self.max_vel
                    self.decel_count = 0
                self.temp_sig = self.tar_sig
            else:
                print("Happy happy happy")

        elif self.type == 'target':
            if len(lpp_result) < 8:
                pass
            else:
                target_pos = lpp_result[7]
                safety = lpp_result[5]
                if safety == 1: 
                    if target_pos[0] == 'REAR':
                        vel = self.max_vel + 1.5
                        self.accel_count += 1
                    else:
                        vel = self.max_vel - 1.5
                        self.decel_count += 1
                elif safety == 2:
                    vel = self.max_vel + 1.8
                    self.accel_count += 1
                else:
                    if 10 < self.decel_count < 30:
                        self.decel_count += 1
                        vel = self.max_vel - 1
                    elif 10 < self.accel_count < 30:
                        self.accel_count += 1
                        vel = self.max_vel + 1
                    else:
                        vel = self.max_vel
                        self.decel_count = 0
                        self.accel_count = 0

        self.temp_vel = vel
        return vel

