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
        self.with_coop = True
        self.decel_value = 0.0002
    
    def update_value(self, user_input, ego, target):
        self.max_vel = user_input['target_velocity']
        self.ego_sig = user_input['signal']
        self.with_coop = True if user_input['with'] == 1 else False
        self.ego_vel = ego['v']
        self.tar_vel = target[2]
        self.tar_sig = target[1]
        
    
    def execute(self, lpp_result):
        if self.temp_sig == 0 and self.temp_vel != self.max_vel:
            self.temp_vel = self.max_vel
        else:
            if self.with_coop:
                if self.tar_sig == 5 or self.temp_sig == 5:
                    self.temp_vel = min(20/3.6, self.temp_vel - (self.max_vel*self.decel_value))
                    self.temp_sig = self.tar_sig

                elif self.tar_sig == 0 or self.tar_sig == 4:
                    self.temp_vel = min(self.max_vel, self.temp_vel + (self.max_vel-self.temp_vel)*0.008)
                    self.temp_sig = self.tar_sig
            else:
                bsd = lpp_result[6]
                if bsd:
                    self.temp_vel = self.temp_vel - (self.max_vel*(0.8))
                else:
                    self.temp_vel = min(self.max_vel, self.temp_vel + (self.max_vel-self.temp_vel)*0.05)

        return self.temp_vel