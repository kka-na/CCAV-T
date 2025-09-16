import socket
import os
from ctypes import *
from datetime import datetime, timedelta
import struct
import math
import time
import logging
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

        # 윈도우 기반 throughput 계산을 위한 변수들
        self.tx_rate = 0
        self.rx_rate = 0
        self.window_size = 10.0  # 10초 윈도우
        self.tx_history = deque()  # (timestamp, bytes_sent) 튜플들 저장
        self.tx_prr_history = deque()
        self.total_bytes_sent = 0
        self.first_tx_time = None
        self.last_mbps_calc_time = 0
        self.mbps_calc_interval = 1.0  # 1초마다 mbps 계산
        
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
        # 프로덕션에서는 콘솔 로깅 비활성화 (필요시 주석 해제)
        # self.logger.addHandler(console_handler)

    def connect(self, IP):
        try:
            self.fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # 소켓 최적화 옵션 추가
            self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Nagle 알고리즘 비활성화
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 송신 버퍼 증가
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)  # 수신 버퍼 증가
            
            if self.interface > 0:
                interface_name = self.interface_list[self.interface]
                try:
                    # macOS 호환성 체크
                    import sys
                    if sys.platform != 'darwin':  # macOS가 아닌 경우만
                        self.fd.setsockopt(socket.SOL_SOCKET, 25, interface_name)
                    else:
                        self.logger.info("macOS: SO_BINDTODEVICE not supported, using default interface")
                except (PermissionError, OSError) as e:
                    self.logger.warning(f"Interface binding failed: {e}, using default interface")
                
            servaddr = socket.getaddrinfo(IP, DEST_PORT, socket.AF_INET, socket.SOCK_STREAM)
            self.fd.connect(servaddr[0][4])
            self.logger.debug("Socket Opened with optimizations")
            return 1
        except socket.error as e:
            self.logger.error(f"{e}")
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
    
    def add_ext_status_data(self, p_overall, package_len):
        """최적화된 add_ext_status_data 함수 - 디버깅 코드 제거됨"""
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

    def rx(self):
        data = self.receive()
        if data is None:
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
            self.calc_rtt(socket.ntohl(sharing_information.tx_cnt_from_rx))
            self.calc_delay(sharing_information.timestamp)
            obstacles = []
            for i in range(socket.ntohs(sharing_information.obstacle_num)):
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
    
    def tx(self, state, paths, obstacles):
        # This function is long, so we're only showing the corrected part.
        # Replace your existing tx function with this one.

        try:
            p_overall = self.get_p_overall(self.tx_cnt)
            self.set_tx_values(state)

            # BUG FIX 1: Correct the size calculation.
            # The V2x_App_SI_TLVC struct already includes the obstacle array,
            # so we must not add the size again.
            size = sizeof(V2x_App_SI_TLVC)

            if self.chip == 'out':
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall_V2), POINTER(V2x_App_SI_TLVC))
            else:
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_SI_TLVC))
            
            p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)

            # BUG FIX 2: Correct the length field.
            # The length is the size of the data part (SharingInformation),
            # which is the total struct size minus the 'type' (4) and 'len' (2) fields.
            p_dummy.contents.len = socket.htons(size - 6)

            p_share_info = cast(addressof(p_dummy.contents.data), POINTER(SharingInformation))
            p_share_info.contents.tx_cnt = socket.htonl(self.tx_cnt)
            p_share_info.contents.tx_cnt_from_rx = socket.htonl(self.tx_cnt_from_rx)
            p_share_info.contents.timestamp = int(time.time()*1000)
            p_share_info.contents.state = state[0]
            p_share_info.contents.signal = state[1]
            p_share_info.contents.latitude = state[2]
            p_share_info.contents.longitude = state[3]
            p_share_info.contents.heading = state[4]
            p_share_info.contents.velocity = state[5]

            if len(paths) > 0:
                for i, x in enumerate(paths[0]):
                    p_share_info.contents.path_x[i] = x
                for i, y in enumerate(paths[1]):
                    p_share_info.contents.path_y[i] = y
            
            # We use the *actual* number of obstacles, up to the max array size.
            num_obstacles_to_send = min(len(obstacles), 12)
            p_share_info.contents.obstacle_num = socket.htons(num_obstacles_to_send)

            # Use safe, direct array access instead of risky pointer math.
            for i in range(num_obstacles_to_send):
                obj = obstacles[i]
                p_share_info.contents.obstacle[i].cls = int(obj[0])
                p_share_info.contents.obstacle[i].enu_x = float(obj[1])
                p_share_info.contents.obstacle[i].enu_y = float(obj[2])
                p_share_info.contents.obstacle[i].heading = float(obj[3])
                p_share_info.contents.obstacle[i].velocity = float(obj[4])
                p_share_info.contents.obstacle[i].distance = float(obj[5])
                p_share_info.contents.obstacle[i].dangerous = int(obj[6])

            # The first package length is the struct + its CRC.
            package_len = size + 2
            
            crc_data = bytearray(p_dummy.contents)
            crc16 = cast(addressof(p_dummy.contents) + size, POINTER(c_uint16))
            crc16.contents.value = socket.htons(calc_crc16(crc_data, size))

            self.add_ext_status_data(p_overall, package_len)

            # BUG FIX 3: Correct the final header length calculation.
            # We need to use htons (host-to-network), not ntohs.
            total_payload_len = 0
            if self.chip == 'out':
                total_payload_len = sizeof(TLVC_Overall_V2) + socket.ntohs(p_overall.contents.len_package)
            else:
                total_payload_len = sizeof(TLVC_Overall) + socket.ntohs(p_overall.contents.len_package)
            
            # This is the total length of the payload, starting after the TxMsg struct.
            self.hdr.contents.len = socket.htons(sizeof(V2x_App_TxMsg) + total_payload_len)
            self.hdr.contents.seq = 0
            self.hdr.contents.payload_id = socket.htons(0x10)
            self.tx_msg.contents.psid = socket.htonl(V2V_PSID)

            # The final length to send is the V2x_App_Hdr + its payload length.
            send_len = sizeof(V2x_App_Hdr) + socket.ntohs(self.hdr.contents.len)
            
            # Calculate the final, overall CRC. The range is everything after the magic number.
            _crc16 = calc_crc16(self.tx_buf[4:], send_len - 4)
            crc16_final = pointer(c_uint16.from_buffer(self.tx_buf, send_len))
            crc16_final.contents.value = socket.htons(_crc16)
            
            # The final data includes the main packet + its 2-byte CRC.
            data = self.tx_buf[:send_len + 2]
            
            self.communication_performance['packet_size'] = len(data)
            self.tx_message = self.get_log_datum(state, paths, obstacles)
            return self.send(data, len(data))

        except Exception as e:
            self.logger.error(f"TX preparation error: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return -1

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
        rx_rate = min(100, (float(self.rx_rate)/self.tx_rate)*100) if self.tx_rate > 0 and self.rx_rate > 0 else 0
        #self.communication_performance['packet_rate'] = rx_rate
        return 1

    def calc_delay(self, rx_timestamp):
        delay = int(time.time()*1000)-rx_timestamp
        self.communication_performance['delay'] = delay

    def send(self, data, send_size):
        try:
            self.fd.sendall(data)
            if self.rx_rate >= 1:
                self.tx_rate += 1
            self.logger.info(f"Tx cnt:{self.tx_cnt} {send_size}\n"+self.tx_message)
            self.rtt_ts_list.append([self.tx_cnt, time.time()])
            self.tx_cnt += 1
            self.update_throughput_metrics(send_size)
            return 1
        except socket.error as e:
            self.logger.error(f"{e}")
            return -1
    
    def receive(self):
        self.fd.settimeout(5)
        try:
            data = self.fd.recv(sizeof(c_char)*MAX_TX_PACKET_TO_OBU)
            return data
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
        self.tx_rate = 0
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

    def get_keti_time(self):
        now = datetime.now() + timedelta(hours=9)
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

    def get_log_datum(self, vehicle_state, vehicle_path, vehicle_obstacles):
        if len(vehicle_state) > 0:
            state = f"Shared Message\nstate:{vehicle_state[0]} signal:{vehicle_state[1]} lat:{vehicle_state[2]} lng:{vehicle_state[3]} h:{vehicle_state[4]} v:{vehicle_state[5]}\n"
        else:
            state = "No message to Send\n"
        if vehicle_path != [] and len(vehicle_path[0]) > 1:
            path = f"path: x={vehicle_path[0][0]} y={vehicle_path[0][1]} ~ x={vehicle_path[-1][0]} y={vehicle_path[-1][1]}\n"
        else:
            path = f"There is no path\n"
        obstacle_number = f"There are {len(vehicle_obstacles)} obstacles\n"
        obstacles_info = ""
        if len(vehicle_obstacles) > 0:
            for i, obs in enumerate(vehicle_obstacles):
                obstacles_info += f"[{i}] cls:{obs[0]} enu_x:{obs[1]} enu_y:{obs[2]} h:{obs[3]} v:{obs[4]}\n\n"
        return state+path+obstacle_number+obstacles_info

    def get_performance_log(self):
        performance_str = "Communication Performance \n" + " ".join([f"{key}: {value}" for key, value in self.communication_performance.items()])
        return performance_str+"\n\n"
    
    def calculate_window_prr(self,current_time):
        window_start_time = current_time - self.window_size
        while self.tx_prr_history and self.tx_prr_history[0][0] < window_start_time:
            self.tx_prr_history.popleft()
        
        if len(self.tx_prr_history) < 3:
            return 0.0
        
        rx_rates = sum(entry[1] for entry in self.tx_prr_history)
        tx_rates = sum(entry[2] for entry in self.tx_prr_history)
        actual_window_time = current_time - self.tx_prr_history[0][0]
        
        if actual_window_time <= 0:
            return 0.0
        prr = rx_rates/tx_rates * 100 if tx_rates and rx_rates > 0 else 0
        print(rx_rates, tx_rates)
        return min(100, int(prr))

    def calculate_window_mbps(self, current_time):
        """윈도우 기반 mbps 계산"""
        window_start_time = current_time - self.window_size
        while self.tx_history and self.tx_history[0][0] < window_start_time:
            self.tx_history.popleft()
        
        if len(self.tx_history) < 2:
            return 0.0
        
        window_bytes = sum(entry[1] for entry in self.tx_history)
        actual_window_time = current_time - self.tx_history[0][0]
        if actual_window_time <= 0:
            return 0.0
        
        mbps = ((window_bytes * 8) / actual_window_time) / 1000000
        return round(mbps, 3)

    def update_throughput_metrics(self, send_size):
        """throughput 메트릭 업데이트"""
        current_time = time.time()
        
        if self.first_tx_time is None:
            self.first_tx_time = current_time
        
        self.tx_history.append((current_time, send_size))
        self.tx_prr_history.append((current_time, self.rx_rate, self.tx_rate))
        self.total_bytes_sent += send_size
        
        if current_time - self.last_mbps_calc_time >= self.mbps_calc_interval:
            window_mbps = self.calculate_window_mbps(current_time)
            self.communication_performance['mbps'] = window_mbps
            prr = self.calculate_window_prr(current_time)
            self.last_mbps_calc_time = current_time
            self.communication_performance['packet_rate'] = prr