# side 2.5-3.0m
# rear 3-9m
# ttz d/v_rel (threshold 3-5sec)

import math

class VelocityPlanner:
    def __init__(self, type):
        self.type = type
        self.setting_values()

    def setting_values(self):
        self.max_vel = 0
        self.ego_sig = 0
        self.ego_vel = 0
        self.ego_pos = [0, 0]
        self.tar_vel = 0
        self.tar_sig = 0
        self.tar_pos = [0, 0]
        self.temp_vel = self.max_vel
        self.temp_sig = self.tar_sig
        self.decel_count = 0
        self.accel_count = 0
        self.cooperative_mode = True
        self.with_coop = True
        self.scenario = 0

        # TTC 기반 파라미터
        self.t_safe = 2.5  # 안전 시간 여유 (초)
        self.d_min = 5.0   # 최소 안전 거리 (m)
        self.k_coop = 0.3  # 협력 계수 (0~1)
        self.t_response = 1.5  # 응답 시간 (초)
        self.ttc_threshold = 5.0  # TTC 임계값 (초)

        # BSD 감속 상태
        self.bsd_decel_active = False
        self.bsd_detected_once = False


    def update_value(self, user_input, ego, target):
        self.max_vel = user_input['target_velocity']
        self.ego_sig = user_input['signal']
        self.ego_vel = ego['v']
        self.ego_pos = [ego['x'], ego['y']]
        self.tar_vel = target[2]
        self.tar_sig = target[1]
        self.tar_pos = [target[3], target[4]] if len(target) > 4 else [0, 0]
        self.cooperative_mode = user_input.get('mode', 0) == 1
        self.with_coop = user_input.get('with_coop', True)
        self.scenario = user_input.get('scenario', 0)


    def calculate_ttc_and_gap(self):
        """TTC와 Gap 계산"""
        distance = math.sqrt((self.ego_pos[0] - self.tar_pos[0])**2 +
                           (self.ego_pos[1] - self.tar_pos[1])**2)

        v_rel = abs(self.ego_vel - self.tar_vel)

        # TTC 계산
        if v_rel > 0.1:
            ttc = distance / v_rel
        else:
            ttc = float('inf')

        # Required Gap: 안전한 차선 변경에 필요한 거리
        g_required = self.ego_vel * self.t_safe + self.d_min

        # Available Gap: 현재 사용 가능한 거리
        g_available = distance

        return ttc, g_required, g_available, distance

    def calculate_cooperative_accel(self, g_required, g_available):
        """협력 가속도 계산 (m/s 단위로 반환)"""
        gap_deficit = g_required - g_available

        if gap_deficit > 0:
            # Gap이 부족 - 가감속 필요
            accel = self.k_coop * gap_deficit / self.t_response
            return min(accel, 2.0)  # 최대 2.0 m/s로 제한
        else:
            # Gap 충분 - 원래 속도로 복귀
            return 0.0

    def execute(self, lpp_result):
        vel = self.max_vel

        if self.type == 'ego':
            # WOC 모드: BSD 기반 감속
            if not self.with_coop:
                # lpp_result에서 BSD 상태 확인 (index 6)
                bsd_active = lpp_result[6] if len(lpp_result) >= 7 else False

                if bsd_active:
                    # BSD 감속 활성화
                    self.bsd_decel_active = True
                    # 협력 가속도 계산
                    ttc, g_req, g_avail, dist = self.calculate_ttc_and_gap()
                    accel = self.calculate_cooperative_accel(g_req, g_avail)
                    vel = max(self.max_vel - accel, self.max_vel * 0.7)  # 최소 70%까지 감속
                    self.decel_count += 1

                elif self.bsd_decel_active:
                    # BSD 해제 후 점진적 복귀 (WOC 모드에서는 signal 검사 안함)
                    vel = self.max_vel - 0.3
                    self.decel_count = max(0, self.decel_count - 1)
                    if self.decel_count == 0:
                        self.bsd_decel_active = False
                else:
                    # 정상 주행
                    vel = self.max_vel
                    self.decel_count = 0

            # WC 모드: Reject 신호 기반 감속 (WOC와 유사하게)
            else:
                # lpp_result에서 BSD 상태 확인
                bsd_active = lpp_result[6] if len(lpp_result) >= 7 else False

                if self.tar_sig == 5:  # 거절 신호
                    # Reject 받으면 감속 시작
                    self.bsd_decel_active = True
                    ttc, g_req, g_avail, dist = self.calculate_ttc_and_gap()
                    accel = self.calculate_cooperative_accel(g_req, g_avail)
                    vel = max(self.max_vel - accel, self.max_vel * 0.7)  # 최소 70%까지 감속
                    self.decel_count += 1
                    self.temp_sig = self.tar_sig

                elif self.bsd_decel_active:
                    # Reject 후 감속 중: 점진적 복귀 (WOC와 동일)
                    vel = self.max_vel - 0.3
                    self.decel_count = max(0, self.decel_count - 1)
                    if self.decel_count == 0:
                        self.bsd_decel_active = False
                    self.temp_sig = self.tar_sig

                else:
                    # 정상 주행
                    vel = self.max_vel
                    self.decel_count = 0
                    self.temp_sig = self.tar_sig

        elif self.type == 'target':
            if len(lpp_result) >= 8:
                target_pos = lpp_result[7]
                safety = lpp_result[5]

                if safety == 1:  # 협력 필요
                    ttc, g_req, g_avail, dist = self.calculate_ttc_and_gap()
                    if ttc < self.ttc_threshold:
                        # Gap 부족 - 협력 가감속
                        accel = self.calculate_cooperative_accel(g_req, g_avail)

                        if target_pos[0] == 'REAR':
                            vel = min(self.max_vel + accel, self.max_vel * 1.3)  # 최대 130%
                            self.accel_count += 1
                        else:
                            vel = max(self.max_vel - accel, self.max_vel * 0.7)  # 최소 70%
                            self.decel_count += 1
                    else:
                        vel = self.max_vel

                elif safety == 2:  # 긴급 협력
                    ttc, g_req, g_avail, dist = self.calculate_ttc_and_gap()
                    accel = self.calculate_cooperative_accel(g_req * 1.2, g_avail)

                    if target_pos[0] == 'FRONT':
                        vel = min(self.max_vel + accel, self.max_vel * 1.35)
                        self.accel_count += 1
                    else:  # REAR
                        vel = self.max_vel

                else:  # 점진적 복귀
                    if self.decel_count > 0:
                        vel = self.max_vel - 0.3
                        self.decel_count = max(0, self.decel_count - 1)
                    elif self.accel_count > 0:
                        vel = self.max_vel + 0.3
                        self.accel_count = max(0, self.accel_count - 1)
                    else:
                        vel = self.max_vel

        self.temp_vel = vel
        return vel

