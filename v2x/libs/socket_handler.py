
import socket
import os
from ctypes import *
from datetime import datetime, timedelta
import struct
import math
import time
import logging
from ctypes import string_at

from collections import deque

from .v2x_interface import *
from .crc16 import calc_crc16

DEST_PORT = 47347
V2V_PSID = EM_V2V_MSG

class SocketHandler:
    def __init__(self, type, interface, chip):
        self.interface = interface
        self.interface_list = [b'', b'enp4s0', b'enx00e04e69e57a']
        self.chip = chip
        self.communication_performance = {
            'state': 0,
            'v2x': 0,
            'rtt': 0,
            'mbps': 0,
            'packet_size': 0,
            'packet_rate': 0,
            'distance': 0,
            'delay' : 0,
        }
        self.rtt_ts_list = []
        self.tx_message = ""

        self.window_size = 10.0  # 10초 윈도우
        self.tx_history = deque()  # (timestamp, bytes_sent) 튜플들 저장
        self.total_bytes_sent = 0
        self.first_tx_time = None
        self.last_mbps_calc_time = 0
        self.mbps_calc_interval = 1.0  # 1초마다 mbps 계산

        self.backoff = 0.5  # 초, 재연결 시작 backoff
        self.backoff_max = 5.0

        # 새로 추가: 10초 패킷 윈도우
        self.window_sec = 10.0
        self.rx_pkt_window = deque() 

        self._W = 10.0
        self._win = deque()   # (t_mono, seq)
        self._seen = set()
        self.communication_performance2 = {
            'packet_success_pct': 0.0,
            'packet_receive_rate_pps': 0.0,
            'rx_window_span': 0.0,
            'rx_unique_in_win': 0,
            'rx_min_seq': 0,
            'rx_max_seq': 0,
        }

        self._rx_stream = bytearray()
        self.set_logger(type)

    def set_logger(self, type):
        self.logger = logging.getLogger('v2x')
        self.logger.setLevel(logging.DEBUG)
        formatted_datetime = datetime.today().strftime('%m%d%H%M')
        log_dir = f"../log/{type}"
        os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.FileHandler(f'../log/{type}/{self.chip}_{formatted_datetime}.log')
        file_handler.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        #self.logger.addHandler(console_handler)



    def connect(self, IP):
        try:
            self.fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # V2X 통신에 최적화된 소켓 설정
            self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Nagle 비활성화
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)  # 1MB 송신 버퍼
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)  # 1MB 수신 버퍼
            
            # 논블로킹 모드는 사용하지 않고, 타임아웃만 줄임
            # self.fd.setblocking(False)  # 이 줄은 주석처리
            
            if self.interface > 0:
                interface_name = self.interface_list[self.interface]
                try:
                    self.fd.setsockopt(socket.SOL_SOCKET, 25, interface_name)
                except PermissionError:
                    self.logger.error("Permission denied for SO_BINDTODEVICE")
                    return -1
                
            servaddr = socket.getaddrinfo(IP, DEST_PORT, socket.AF_INET, socket.SOCK_STREAM)
            self.fd.connect(servaddr[0][4])
            
            # 연결 후 Keep-Alive 설정
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            
            self.logger.debug("Socket Opened with optimized settings")
            return 1
        except socket.error as e:
            self.logger.error(f"Connection error: {e}")
            return -1
    
    def register(self):
        buf, hdr = self.get_base()

        hdr.contents.len = socket.htons(SIZE_WSR_DATA-6)
        hdr.contents.seq = 0
        hdr.contents.payload_id = socket.htons(eV2x_App_Payload_Id.ePayloadId_WsmServiceReq)

        wsr = cast(addressof(hdr.contents) + V2x_App_Hdr.data.offset, POINTER(V2x_App_WSR_Add_Crc))
        wsr.contents.action = 0 #add
        wsr.contents.psid = socket.htonl(V2V_PSID) 
        
        _crc16 = calc_crc16(buf[SIZE_MAGIC_NUMBER_OF_HEADER:], SIZE_WSR_DATA - 4 - 2)
        crc16 = pointer(c_uint16.from_buffer(buf, SIZE_WSR_DATA - 2))
        crc16.contents.value = socket.htons(_crc16)
        data = buf[:SIZE_WSR_DATA]

        self.tx_cnt = 0
        self.tx_start_time = time.time()

        return self.send(data, SIZE_WSR_DATA)
    
    def tx(self, state, paths, obstacles):
        try:
            # 입력 데이터 검증
            if not state or len(state) < 6:
                self.logger.error(f"Invalid state data: {state}")
                return -1
                
            # 장애물 데이터 검증 및 정리
            valid_obstacles = []
            for i, obs in enumerate(obstacles):
                if len(obs) >= 7:  # 필수 필드 개수 확인
                    # 데이터 타입 검증
                    try:
                        cls = int(obs[0])
                        enu_x = float(obs[1])
                        enu_y = float(obs[2]) 
                        heading = float(obs[3])
                        velocity = float(obs[4])
                        distance = float(obs[5])
                        dangerous = int(obs[6])
                        
                        # NaN이나 무한대 값 체크
                        if any(not math.isfinite(val) for val in [enu_x, enu_y, heading, velocity, distance]):
                            self.logger.warning(f"Skipping obstacle {i} due to invalid float values")
                            continue
                            
                        valid_obstacles.append([cls, enu_x, enu_y, heading, velocity, distance, dangerous])
                    except (ValueError, TypeError) as e:
                        self.logger.warning(f"Skipping obstacle {i} due to data conversion error: {e}")
                        continue
                else:
                    self.logger.warning(f"Skipping obstacle {i} due to insufficient fields: {len(obs)}")
                    
            # 안전한 장애물 개수 제한 - 점진적 테스트를 위해 작게 시작
            MAX_OBSTACLES = min(12, len(valid_obstacles))  # 우선 10개로 제한
            safe_obstacles = valid_obstacles[:MAX_OBSTACLES]
            
            self.logger.info(f"Processing {len(safe_obstacles)} obstacles out of {len(obstacles)} input")
            
            p_overall = self.get_p_overall(self.tx_cnt)
            self.set_tx_values(state)

            size = sizeof(V2x_App_SI_TLVC) + sizeof(ObstacleInformation) * len(safe_obstacles)
            self.logger.debug(f"Calculated payload size: {size} bytes")
            
            if self.chip == 'out':
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall_V2), POINTER(V2x_App_SI_TLVC))
            else:
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_SI_TLVC))
                
            p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)
            p_dummy.contents.len = socket.htons(size + 2)
        
            p_share_info = cast(addressof(p_dummy.contents.data), POINTER(SharingInformation))
            
            # 기본 정보 설정 with validation
            p_share_info.contents.tx_cnt = socket.htonl(self.tx_cnt)
            p_share_info.contents.tx_cnt_from_rx = socket.htonl(self.tx_cnt_from_rx)
            p_share_info.contents.timestamp = self.htobe64(int(time.time() * 1000))
            
            try:
                p_share_info.contents.state = int(state[0])
                p_share_info.contents.signal = int(state[1])
                p_share_info.contents.latitude = float(state[2])
                p_share_info.contents.longitude = float(state[3])
                p_share_info.contents.heading = float(state[4])
                p_share_info.contents.velocity = float(state[5])
            except (ValueError, IndexError) as e:
                self.logger.error(f"Error setting vehicle state: {e}")
                return -1

            # Path 정보 설정
            if len(paths) > 0 and len(paths[0]) > 0:
                path_len = min(30, len(paths[0]), len(paths[1]))
                for i in range(path_len):
                    try:
                        p_share_info.contents.path_x[i] = float(paths[0][i])
                        p_share_info.contents.path_y[i] = float(paths[1][i])
                    except (ValueError, IndexError) as e:
                        self.logger.warning(f"Error setting path point {i}: {e}")
                        break
            
            # 장애물 정보 설정
            
            p_share_info.contents.obstacle_num = socket.htons(len(safe_obstacles))
            
            if len(safe_obstacles) > 0:
                for o, obj in enumerate(safe_obstacles):
                    try:
                        obstacle = cast(
                            addressof(p_share_info.contents) + sizeof(SharingInformation) + (o * sizeof(ObstacleInformation)), 
                            POINTER(ObstacleInformation)
                        )
                        obstacle.contents.cls = int(obj[0])
                        obstacle.contents.enu_x = float(obj[1])
                        obstacle.contents.enu_y = float(obj[2])
                        obstacle.contents.heading = float(obj[3])
                        obstacle.contents.velocity = float(obj[4])
                        obstacle.contents.distance = float(obj[5])
                        obstacle.contents.dangerous = int(obj[6])
                    except Exception as e:
                        self.logger.error(f"Error setting obstacle {o}: {e}")
                        return -1
            
            # CRC 계산
            try:
                crc16_ptr = cast(addressof(p_dummy.contents) + size, POINTER(c_uint16))
                crc_bytes = string_at(addressof(p_dummy.contents), size)  # ✅ size만큼 정확히 바이트 추출
                crc16_ptr.contents.value = socket.htons(calc_crc16(crc_bytes, size))
            except Exception as e:
                self.logger.error(f"CRC calculation error: {e}")
                return -1
            
            package_len = 8 + size
            p_overall.contents.len_package = socket.htons(package_len)

            self.add_ext_status_data(p_overall, package_len)

            # 헤더 길이 계산
            if self.chip == 'out':
                total_len = sizeof(V2x_App_Hdr) + 6 + sizeof(TLVC_Overall_V2) + socket.ntohs(p_overall.contents.len_package)
            else:
                total_len = sizeof(V2x_App_Hdr) + 6 + sizeof(TLVC_Overall) + socket.ntohs(p_overall.contents.len_package)
                
            self.hdr.contents.len = socket.htons(total_len - 6)  # magic+len 제외
            self.hdr.contents.seq = 0
            self.hdr.contents.payload_id = socket.htons(0x10)
            self.tx_msg.contents.psid = socket.htonl(EM_V2V_MSG)
            
            send_len = total_len
            
            # 패킷 크기 최종 검증
            MAX_SAFE_SIZE = 1000  # 안전한 크기로 제한
            if send_len > MAX_SAFE_SIZE:
                self.logger.error(f"Packet size {send_len} exceeds safe limit {MAX_SAFE_SIZE}")
                return -1
                
            self.logger.info(f"Final packet size: {send_len} bytes")

            # 최종 CRC 계산
            try:
                crc16_final = pointer(c_uint16.from_buffer(self.tx_buf, send_len - 2))
                _crc16 = calc_crc16(self.tx_buf[SIZE_MAGIC_NUMBER_OF_HEADER:], send_len - 6)
                crc16_final.contents.value = socket.htons(_crc16)
            except Exception as e:
                self.logger.error(f"Final CRC calculation error: {e}")
                return -1
            
            data = self.tx_buf[:send_len]
            
            self.communication_performance['packet_size'] = send_len
            self.tx_message = self.get_log_datum(state, paths, safe_obstacles)
            
            return self.send(data, send_len)
            
        except Exception as e:
            self.logger.error(f"Unexpected error in tx(): {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return -1

    # socket_handler.py
    def send(self, data, send_size):
        try:
            # 연결 시 한번만 설정: self.fd.settimeout(2.0)
            total_sent = 0
            max_retries = 7
            backoff = 0.1  # 100ms

            while total_sent < send_size:
                try:
                    sent = self.fd.send(data[total_sent:])
                    if sent == 0:
                        raise socket.error("Connection broken")
                    total_sent += sent
                except (socket.timeout, BlockingIOError, InterruptedError):
                    if max_retries <= 0:
                        self.logger.error("Send stalled, dropping this frame gracefully")
                        return -1
                    time.sleep(backoff)
                    max_retries -= 1
                    backoff = min(backoff * 2, 0.8)  # max 800ms
                    continue

            if total_sent == send_size:
                self.logger.info(f"Tx cnt:{self.tx_cnt}\n"+self.tx_message)
                self.logger.debug(f"Sent {send_size} bytes successfully")
                self.rtt_ts_list.append([self.tx_cnt, time.time()])
                self.tx_cnt += 1
                self.update_throughput_metrics(send_size)
                return 1
            else:
                self.logger.error(f"Partial send: {total_sent}/{send_size}")
                return -1

        except socket.error as e:
            self.logger.error(f"Socket send error: {e}")
            return -1


    def rx(self):
        data = self.receive()
        if data == None:
            return [0,0,0]
        if len(data) > SIZE_WSR_DATA:
            self.rx_cnt += 1
            self.rx_rate += 1
            hdr_ofs = V2x_App_Hdr.data.offset
            rx_ofs = V2x_App_RxMsg.data.offset
            if self.chip == 'out':
                ovr_ofs = sizeof(TLVC_Overall_V2)
            else:
                ovr_ofs = sizeof(TLVC_Overall)
            tlvc_ofs =  hdr_ofs+rx_ofs+ovr_ofs
            tlvc = V2x_App_SI_TLVC.from_buffer_copy(data,tlvc_ofs)
            sharing_information = tlvc.data
            self.set_rx_values(sharing_information)
            self.tx_cnt_from_rx = socket.ntohl(sharing_information.tx_cnt)

            # if self.rx_rate > 0 and not self.start_prr :
            #     self.center_value = self.tx_cnt_from_rx-1
            #     self.start_prr = True

            # if self.center_value > 0:
            #     prr = self.rx_rate / (self.tx_cnt_from_rx - self.center_value) * 100
            #     self.communication_performance['packet_rate'] = prr

            self._update_prr_pps(self.tx_cnt_from_rx)

            self.calc_rtt(socket.ntohl(sharing_information.tx_cnt_from_rx))
            self.calc_delay(sharing_information.timestamp)


            # --- 장애물 개수 경계 검사 추가 ---
            reported_obs = socket.ntohs(sharing_information.obstacle_num)

            # tlvc_ofs는 이미 계산됨
            base_size = sizeof(V2x_App_SI_TLVC)  # SharingInformation 포함
            obs_size  = sizeof(ObstacleInformation)

            # 프레임에서 실제로 파싱 가능한 최대 장애물 수
            if len(data) < tlvc_ofs + base_size:
                max_obs_by_len = 0
            else:
                payload_room = len(data) - (tlvc_ofs + base_size)
                max_obs_by_len = max(0, payload_room // obs_size)

            obs_count = min(reported_obs, max_obs_by_len)
            # -----------------------------------


            obstacles = []
            for i in range(obs_count):
                ofs = tlvc_ofs+sizeof(V2x_App_SI_TLVC)+(i*sizeof(ObstacleInformation))
                if len(data) < ofs + sizeof(ObstacleInformation):
                    pass
                else:
                    obstacle = ObstacleInformation.from_buffer_copy(data[ofs:ofs+sizeof(ObstacleInformation)])
                    obstacles.append(obstacle)
            state, path, obstacles = self.organize_data(len(data), sharing_information, obstacles)
            rx_message = self.get_log_datum(state, path, obstacles)
            self.logger.info(f"Rx cnt:{self.tx_cnt_from_rx}\n"+rx_message)
            self.logger.info(self.get_performance_log())  
            return [state, path, obstacles]
        else:
            return [0, 0, 0]

    def set_tx_values(self, state):
        self.tx_latitude = state[2]
        self.tx_longitude = state[3]
    
    def set_rx_values(self, sharing_information):
        self.rx_latitude = sharing_information.latitude
        self.rx_longtude = sharing_information.longitude

    def calc_rtt(self, tx_cnt_from_rx):
        rtt = 0
        for ts in self.rtt_ts_list:
            if ts[0] == tx_cnt_from_rx:
                rtt = round((time.time()-ts[1])*1000)
                self.communication_performance['rtt'] = rtt
                break
        self.rtt_ts_list[:] = [ts for ts in self.rtt_ts_list if ts[0] >= tx_cnt_from_rx]
    
    def calc_comm(self):
        if self.tx_cnt < 1 or self.rx_cnt < 1:
            return -1
        else:
            distance = math.sqrt((self.rx_latitude - self.tx_latitude) ** 2 + (self.rx_longtude - self.tx_longitude) ** 2)
            self.communication_performance['v2x'] = 1
            self.communication_performance['distance'] = distance
            
            return self.communication_performance        

    def calc_rate(self, hz):
        #rx_rate = min(100, (float(self.rx_rate)/self.tx_rate)*100) if self.rx_rate > 0 else 0
        # self.communication_performance['packet_rate'] = rx_rate
        # if self.rx_rate >= hz:
        #     self.rx_rate = 1
        # if self.tx_rate >= hz:
        #     self.tx_rate = 1
        return 1

    def calc_delay(self, rx_timestamp):
        try:
            rx_ts_ms = self.be64toh(rx_timestamp)  # 64-bit big-endian -> host
        except Exception:
            rx_ts_ms = int(rx_timestamp)  # 호환 (기존 환경 대응)
        delay = int(time.time()*1000) - rx_ts_ms
        self.communication_performance['delay'] = delay
        
    
    def receive(self):
        # TCP 스트림 -> 프레임 분리
        # 헤더: magic(4B, u32, network order) + len(2B, u16, network order)
        # total_frame_len = 6 + len_field
        self.fd.settimeout(5)
        try:
            chunk = self.fd.recv(sizeof(c_char)*MAX_TX_PACKET_TO_OBU)
            if not chunk:
                return None
            self._rx_stream.extend(chunk)

            frames = []
            while True:
                # 헤더가 부족하면 더 받기
                if len(self._rx_stream) < 6:
                    break
                # 헤더 파싱 (네트워크 바이트순서)
                magic = struct.unpack_from('!I', self._rx_stream, 0)[0]
                frame_len_field = struct.unpack_from('!H', self._rx_stream, 4)[0]
                total_len = 6 + frame_len_field

                # total_len 이상 도착했는지 확인
                if len(self._rx_stream) < total_len:
                    break

                # 한 프레임 추출
                frame = bytes(self._rx_stream[:total_len])
                del self._rx_stream[:total_len]
                frames.append(frame)

            # 한 번에 여러 프레임이 들어올 수 있지만,
            # 현재 상위 rx() 로직은 한 번에 하나만 처리하므로 가장 최근 프레임을 반환
            if frames:
                return frames[-1]
            return None
        except socket.timeout as t:
            self.logger.error(f"{t}")
            return None

        
    def get_base(self):
        buf = (c_char * MAX_TX_PACKET_TO_OBU)()
        memset(addressof(buf), 0, sizeof(buf))
        hdr = cast(buf, POINTER(V2x_App_Hdr))
        hdr.contents.magic = V2V_INF_EXT_MAGIC
        
        return buf, hdr

    def set_tx(self):
        self.tx_buf, self.hdr = self.get_base()
        self.tx_msg = cast(addressof(self.hdr.contents) + V2x_App_Hdr.data.offset, POINTER(V2x_App_TxMsg))
        if self.tx_buf == None or self.hdr == None or self.tx_msg == None:
            self.logger.error("V2x_APP_Hdr memory setting error")
            return -1
        else:  
            self.logger.debug("Tx Send Set")
            return 1
    
    def set_rx(self):
        self.rx_buf = (c_char * MAX_TX_PACKET_TO_OBU)()
        self.rx_cnt = 0
        self.tx_cnt_from_rx = 0
        self.rx_rate = 0
        self.rx_latitude = 0
        self.rx_longtude = 0
        self.start_prr = False
        self.center_value = 0
        if self.rx_buf == None:
            self.logger.error("Receive memory setting error")
            return -1
        else:  
            self.logger.debug("Rx Send Set")
            return 1

    def get_p_overall(self, cnt):
        if self.chip == 'out':
            p_overall = cast(addressof(self.tx_msg.contents)+V2x_App_TxMsg.data.offset, POINTER(TLVC_Overall_V2))
            p_overall.contents.len = socket.htons(sizeof(TLVC_Overall_V2)-6)
        else:
            p_overall = cast(addressof(self.tx_msg.contents)+V2x_App_TxMsg.data.offset, POINTER(TLVC_Overall))
            p_overall.contents.len = socket.htons(sizeof(TLVC_Overall)-6)
        p_overall.contents.type = socket.htonl(EM_PT_OVERALL)
        p_overall.contents.magic=b'EMOP'
        p_overall.contents.version = 2
        p_overall.contents.num_package = cnt
        if self.chip == 'out':
            p_overall.contents.bitwize = 0x77
        crc_data = bytearray(p_overall.contents)
        if self.chip == 'out':
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall_V2)-2))
        else:
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall)-2))
        return p_overall

    def organize_data(self, data_size, sharing_information, obstacles):

        tx_cnt = socket.ntohl(sharing_information.tx_cnt)
        tx_cnt_from_rx = socket.ntohl(sharing_information.tx_cnt_from_rx)
        vehicle_state = [ sharing_information.state, sharing_information.signal,
                         sharing_information.latitude,sharing_information.longitude,
                         sharing_information.heading, sharing_information.velocity ]
        vehicle_path = [ list(sharing_information.path_x),list(sharing_information.path_y) ]
        vehicle_obstacles = []
        for obs in obstacles:
            vehicle_obstacles.append([obs.cls, obs.enu_x, obs.enu_y, obs.heading, obs.velocity, obs.distance, obs.dangerous])
        return vehicle_state, vehicle_path, vehicle_obstacles


    def add_ext_status_data(self, p_overall, package_len):
        if self.chip == 'out':
            p_status = cast(addressof(p_overall.contents)+sizeof(TLVC_Overall_V2)+package_len, POINTER(TLVC_STATUS_CommUnit_V2))
            package_len += sizeof(TLVC_STATUS_CommUnit_V2)
        else:
            p_status = cast(addressof(p_overall.contents)+sizeof(TLVC_Overall)+package_len, POINTER(TLVC_STATUS_CommUnit))
            package_len += sizeof(TLVC_STATUS_CommUnit)
        p_overall.contents.num_package += 1
        
        p_overall.contents.len_package = socket.htons(package_len)
        crc_data = bytearray(p_overall.contents)
        if self.chip == 'out':
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall_V2)-2))
        else:
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall)-2))
        
        p_status.contents.type = socket.htonl(EM_PT_STATUS)
        if self.chip == 'out':
            p_status.contents.len = socket.htons(sizeof(TLVC_STATUS_CommUnit_V2)-6)
        else:
            p_status.contents.len = socket.htons(sizeof(TLVC_STATUS_CommUnit)-6)
        p_status.contents.dev_type = eV2x_App_Ext_Status_DevType.eStatusDevType_Obu
        p_status.contents.tx_rx = 0
        p_status.contents.dev_id = socket.htonl(1)
        p_status.contents.hw_ver = socket.htons(2)
        p_status.contents.sw_ver = socket.htons(3)
        p_status.contents.timestamp = self.htobe64(self.get_keti_time())
        crc_data = bytearray(p_status.contents)
        if self.chip == 'out':
            p_status.contents.cpu_temp = 50
            p_status.contents.peri_temp = 50
            p_status.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_STATUS_CommUnit_V2)-2))
        else:
            p_status.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_STATUS_CommUnit)-2))

    def get_keti_time(self):
        now = datetime.now() + timedelta(hours=9)

        # 필요한 포맷으로 시간 조합
        keti_time = (now.year * 1000000000000000 +
                    now.month * 10000000000000 +
                    now.day * 100000000000 +
                    now.hour * 1000000000 +
                    now.minute * 10000000 +
                    now.second * 100000 +
                    now.microsecond // 10) 
        return keti_time

    def htobe64(self, value):
        packed_value = struct.pack('>Q', value)
        return struct.unpack('>Q', packed_value)[0]

    def be64toh(self, value):
        # value가 이미 host로 들어왔다면 그대로 반환해도 되지만,
        # ctypes 구조체가 네트워크 순서로 담겨있다면 아래처럼 바꿉니다.
        return struct.unpack('>Q', struct.pack('>Q', value))[0]


    def print_hexa(self, data):
        self.logger.info("Binary data in hexadecimal: ", end="")
        for byte in data:  # buf의 각 바이트에 대해 반복
            print(f"{byte:02X} ", end="")  # 각 바이트를 16진수 형식으로 출력
        print()  # 줄바꿈
    
    def get_log_datum(self, vehicle_state, vehicle_path, vehicle_obstacles):
        if len(vehicle_state) > 0:
            state = f"Shared Message\nstate:{vehicle_state[0]} signal:{vehicle_state[1]} lat:{vehicle_state[2]} lng:{vehicle_state[3]} h:{vehicle_state[4]} v:{vehicle_state[5]}\n"
        else:
            state = "No message to Send\n"
        if vehicle_path != [] and len(vehicle_path[0]) > 1:
            path = f"path: x={vehicle_path[0][0]} y={vehicle_path[0][1]} ~ x={vehicle_path[-1][0]} y={vehicle_path[-1][1]}\n"
        else:
            path = f"Threre is no path\n"
        obstacle_number = f"There are {len(vehicle_obstacles)} obstacles\n"
        obstacles_info = ""
        if len(vehicle_obstacles) > 0:
            for i, obs in enumerate(vehicle_obstacles):
                obstacles_info += f"[{i}] cls:{obs[0]} enu_x:{obs[1]} enu_y:{obs[2]} h:{obs[3]} v:{obs[4]}\n\n"
        return state+path+obstacle_number+obstacles_info

    def get_performance_log(self):
        performance_str = "Communication Performance \n" + " ".join([f"{key}: {value}" for key, value in self.communication_performance.items()])
        return performance_str+"\n\n"
    
    def calculate_window_mbps(self, current_time):
        """윈도우 기반 mbps 계산"""
        # 윈도우 크기를 벗어난 오래된 데이터 제거
        window_start_time = current_time - self.window_size
        while self.tx_history and self.tx_history[0][0] < window_start_time:
            self.tx_history.popleft()
        
        # 윈도우 내 총 바이트 계산
        if len(self.tx_history) < 2:
            return 0.0
        
        window_bytes = sum(entry[1] for entry in self.tx_history)
        actual_window_time = current_time - self.tx_history[0][0]
        
        if actual_window_time <= 0:
            return 0.0
        
        # Mbps 계산: (bytes * 8 bits/byte) / (time_seconds * 1,000,000)
        mbps = (window_bytes * 8) / (actual_window_time * 1000000)
        return round(mbps, 3)

    def update_throughput_metrics(self, send_size):
        """throughput 메트릭 업데이트"""
        current_time = time.time()
        
        # 첫 전송 시간 기록
        if self.first_tx_time is None:
            self.first_tx_time = current_time
        
        # 전송 기록 추가
        self.tx_history.append((current_time, send_size))
        self.total_bytes_sent += send_size
        
        # 일정 간격으로만 mbps 계산 (성능 최적화)
        if current_time - self.last_mbps_calc_time >= self.mbps_calc_interval:
            # 윈도우 기반 mbps 계산
            window_mbps = self.calculate_window_mbps(current_time)
            self.communication_performance['mbps'] = window_mbps

            
            self.last_mbps_calc_time = current_time
    
    def _update_prr_pps(self, seq: int):
        t = time.monotonic()
        s = int(seq)

        # 0) 푸시
        self._win.append((t, s))

        L_ms = 200
        cutoff = t - (self._W + L_ms/1000.0)

        # 1) 10초 밖 제거
        #cutoff = t - self._W
        changed = False
        while self._win and self._win[0][0] < cutoff:
            self._win.popleft()
            changed = True
        # 제거가 있었으면 seen 재구성(간단/안전)
        if changed:
            self._seen = {x[1] for x in self._win}

        # 2) 고유 seq 추가
        self._seen.add(s)

        if not self._win or not self._seen:
            # 비었을 때 0 처리
            self.communication_performance2['packet_success_pct'] = 0.0
            self.communication_performance2['packet_receive_rate_pps'] = 0.0
            self.communication_performance2['rx_window_span'] = 0.0
            self.communication_performance2['rx_unique_in_win'] = 0
            self.communication_performance2['rx_min_seq'] = 0
            self.communication_performance2['rx_max_seq'] = 0
            self.communication_performance['packet_rate'] = 0.0  # 기존 필드 유지 시
            return

        # 3) 집계
        span = max(1e-3, self._win[-1][0] - self._win[0][0])
        uniq = len(self._seen)
        mn, mx = min(self._seen), max(self._seen)
        sent_est = max(1, mx - mn + 1)

        prr = (uniq / sent_est) * 100.0         # PRR (%)
        pps = uniq / self._W                    # PPS (pkt/s, 수신율)

        # 4) 저장/출력 (필요한 것만)
        self.communication_performance2['packet_success_pct'] = round(prr, 1)
        self.communication_performance2['packet_receive_rate_pps'] = round(pps, 2)
        self.communication_performance2['rx_window_span'] = round(span, 3)
        self.communication_performance2['rx_unique_in_win'] = uniq
        self.communication_performance2['rx_min_seq'] = mn
        self.communication_performance2['rx_max_seq'] = mx

        # 기존 UI에서 packet_rate가 퍼센트 의미면 그대로 매핑
        self.communication_performance['packet_rate'] = round(prr, 2)