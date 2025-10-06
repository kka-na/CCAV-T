#!/usr/bin/env python
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
from queue import Queue
from threading import Thread
from functools import lru_cache

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

        self.window_size = 10.0
        self.tx_history = deque()
        self.total_bytes_sent = 0
        self.first_tx_time = None
        self.last_mbps_calc_time = 0
        self.mbps_calc_interval = 1.0

        self.backoff = 0.5
        self.backoff_max = 5.0

        self.window_sec = 10.0
        self.rx_pkt_window = deque() 

        self._W = 10.0
        self._win = deque()
        self._seen = set()
        self._rx_stream = bytearray()
        
        # 비동기 로깅 큐
        self._log_queue = Queue(maxsize=1000)
        self._log_thread = Thread(target=self._async_logger, daemon=True)
        self._log_thread.start()
        
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

    def _async_logger(self):
        """비동기 로깅 워커 스레드"""
        while True:
            try:
                log_type, tx_cnt, message = self._log_queue.get(timeout=1)
                if log_type == 'tx':
                    self.logger.info(f"Tx cnt:{tx_cnt}\n{message}")
                elif log_type == 'rx':
                    self.logger.info(f"Rx cnt:{tx_cnt}\n{message}")
                elif log_type == 'perf':
                    self.logger.info(message)
            except:
                continue

    def connect(self, IP):
        try:
            self.fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # 최저 레이턴시 설정
            self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 버퍼 축소
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            
            # TCP Quick ACK (리눅스)
            try:
                self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
            except:
                pass
            
            if self.interface > 0:
                interface_name = self.interface_list[self.interface]
                try:
                    self.fd.setsockopt(socket.SOL_SOCKET, 25, interface_name)
                except PermissionError:
                    self.logger.error("Permission denied for SO_BINDTODEVICE")
                    return -1
                
            servaddr = socket.getaddrinfo(IP, DEST_PORT, socket.AF_INET, socket.SOCK_STREAM)
            self.fd.connect(servaddr[0][4])
            
            self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            
            # 연결 타임아웃 최소화
            self.fd.settimeout(0.5)
            
            self.logger.debug("Socket opened with ultra-low latency settings")
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
        wsr.contents.action = 0
        wsr.contents.psid = socket.htonl(V2V_PSID) 
        
        _crc16 = calc_crc16(buf[SIZE_MAGIC_NUMBER_OF_HEADER:], SIZE_WSR_DATA - 4 - 2)
        crc16 = pointer(c_uint16.from_buffer(buf, SIZE_WSR_DATA - 2))
        crc16.contents.value = socket.htons(_crc16)
        data = buf[:SIZE_WSR_DATA]

        self.tx_cnt = 0
        self.tx_start_time = time.perf_counter()

        return self.send(data, SIZE_WSR_DATA)
    
    def tx(self, state, paths, obstacles):
        try:
            if not state or len(state) < 6:
                self.logger.error(f"Invalid state data: {state}")
                return -1
                
            valid_obstacles = []
            for i, obs in enumerate(obstacles):
                if len(obs) >= 7:
                    try:
                        cls = int(obs[0])
                        enu_x = float(obs[1])
                        enu_y = float(obs[2]) 
                        heading = float(obs[3])
                        velocity = float(obs[4])
                        distance = float(obs[5])
                        dangerous = int(obs[6])
                        
                        if any(not math.isfinite(val) for val in [enu_x, enu_y, heading, velocity, distance]):
                            continue
                            
                        valid_obstacles.append([cls, enu_x, enu_y, heading, velocity, distance, dangerous])
                    except (ValueError, TypeError):
                        continue
                    
            MAX_OBSTACLES = min(12, len(valid_obstacles))
            safe_obstacles = valid_obstacles[:MAX_OBSTACLES]
            
            p_overall = self.get_p_overall(self.tx_cnt)
            self.set_tx_values(state)

            size = sizeof(V2x_App_SI_TLVC) + sizeof(ObstacleInformation) * len(safe_obstacles)
            
            if self.chip == 'out':
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall_V2), POINTER(V2x_App_SI_TLVC))
            else:
                p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_SI_TLVC))
                
            p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)
            p_dummy.contents.len = socket.htons(size + 2)
        
            p_share_info = cast(addressof(p_dummy.contents.data), POINTER(SharingInformation))
            
            # 밀리초 단위 Unix 타임스탬프 (통신 및 로그 모두 사용)
            timestamp_ms = int(time.time() * 1000)
            
            p_share_info.contents.tx_cnt = socket.htonl(self.tx_cnt)
            p_share_info.contents.tx_cnt_from_rx = socket.htonl(self.tx_cnt_from_rx)
            p_share_info.contents.timestamp = self.htobe64(timestamp_ms)
            
            # 로그용 타임스탬프도 동일하게 저장
            self.current_tx_timestamp = timestamp_ms
            
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

            if len(paths) > 0 and len(paths[0]) > 0:
                path_len = min(30, len(paths[0]), len(paths[1]))
                for i in range(path_len):
                    try:
                        p_share_info.contents.path_x[i] = float(paths[0][i])
                        p_share_info.contents.path_y[i] = float(paths[1][i])
                    except (ValueError, IndexError):
                        break
            
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
            
            # CRC 최적화 - 캐싱
            try:
                crc16_ptr = cast(addressof(p_dummy.contents) + size, POINTER(c_uint16))
                crc_bytes = string_at(addressof(p_dummy.contents), size)
                crc16_ptr.contents.value = socket.htons(calc_crc16(crc_bytes, size))
            except Exception as e:
                self.logger.error(f"CRC calculation error: {e}")
                return -1
            
            package_len = 8 + size
            p_overall.contents.len_package = socket.htons(package_len)

            # Status data 추가 (KETI 형식은 status에만 사용)
            self.add_ext_status_data(p_overall, package_len)

            if self.chip == 'out':
                total_len = sizeof(V2x_App_Hdr) + 6 + sizeof(TLVC_Overall_V2) + socket.ntohs(p_overall.contents.len_package)
            else:
                total_len = sizeof(V2x_App_Hdr) + 6 + sizeof(TLVC_Overall) + socket.ntohs(p_overall.contents.len_package)
                
            self.hdr.contents.len = socket.htons(total_len - 6)
            self.hdr.contents.seq = 0
            self.hdr.contents.payload_id = socket.htons(0x10)
            self.tx_msg.contents.psid = socket.htonl(EM_V2V_MSG)
            
            send_len = total_len
            
            MAX_SAFE_SIZE = 1000
            if send_len > MAX_SAFE_SIZE:
                self.logger.error(f"Packet size {send_len} exceeds safe limit {MAX_SAFE_SIZE}")
                return -1

            try:
                crc16_final = pointer(c_uint16.from_buffer(self.tx_buf, send_len - 2))
                _crc16 = calc_crc16(self.tx_buf[SIZE_MAGIC_NUMBER_OF_HEADER:], send_len - 6)
                crc16_final.contents.value = socket.htons(_crc16)
            except Exception as e:
                self.logger.error(f"Final CRC calculation error: {e}")
                return -1
            
            # memoryview로 zero-copy
            data = memoryview(self.tx_buf)[:send_len]
            
            self.communication_performance['packet_size'] = send_len
            # 로그용 메시지 생성 시 실제 통신 타임스탬프 사용
            self.tx_message = self.get_log_datum(self.current_tx_timestamp, state, paths, safe_obstacles)
            
            return self.send(data, send_len)
            
        except Exception as e:
            self.logger.error(f"Unexpected error in tx(): {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return -1

    def send(self, data, send_size):
        try:
            # TCP_CORK로 패킷 병합 방지 및 즉시 전송
            try:
                self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_CORK, 1)
            except:
                pass
            
            total_sent = 0
            max_retries = 3  # 재시도 횟수 감소
            backoff = 0.01  # 10ms

            while total_sent < send_size:
                try:
                    sent = self.fd.send(data[total_sent:])
                    if sent == 0:
                        raise socket.error("Connection broken")
                    total_sent += sent
                except (socket.timeout, BlockingIOError, InterruptedError):
                    if max_retries <= 0:
                        self.logger.error("Send stalled, dropping frame")
                        return -1
                    time.sleep(backoff)
                    max_retries -= 1
                    backoff = min(backoff * 1.5, 0.1)
                    continue

            # 즉시 전송
            try:
                self.fd.setsockopt(socket.IPPROTO_TCP, socket.TCP_CORK, 0)
            except:
                pass

            if total_sent == send_size:
                # 비동기 로깅
                try:
                    self._log_queue.put_nowait(('tx', self.tx_cnt, self.tx_message))
                except:
                    pass
                
                # 고정밀 타이밍
                self.rtt_ts_list.append([self.tx_cnt, time.perf_counter()])
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
        data_list = self.receive()
        if data_list is None:
            return [0,0,0]
        latest_result = [0,0,0]
        for data in data_list:
            if len(data) > SIZE_WSR_DATA:
                self.rx_cnt += 1
                self.rx_rate += 1
                hdr_ofs = V2x_App_Hdr.data.offset
                rx_ofs = V2x_App_RxMsg.data.offset
                if self.chip == 'out':
                    ovr_ofs = sizeof(TLVC_Overall_V2)
                else:
                    ovr_ofs = sizeof(TLVC_Overall)
                tlvc_ofs = hdr_ofs+rx_ofs+ovr_ofs
                tlvc = V2x_App_SI_TLVC.from_buffer_copy(data,tlvc_ofs)
                sharing_information = tlvc.data
                self.set_rx_values(sharing_information)
                self.tx_cnt_from_rx = socket.ntohl(sharing_information.tx_cnt)

                self._update_prr_pps(self.tx_cnt_from_rx)

                self.calc_rtt(socket.ntohl(sharing_information.tx_cnt_from_rx))
                get_time = sharing_information.timestamp
                self.calc_delay(get_time)
                distance = math.sqrt((self.rx_latitude - self.tx_latitude) ** 2 + (self.rx_longtude - self.tx_longitude) ** 2)
                self.communication_performance['v2x'] = 1
                self.communication_performance['distance'] = distance

                reported_obs = socket.ntohs(sharing_information.obstacle_num)
                base_size = sizeof(V2x_App_SI_TLVC)
                obs_size = sizeof(ObstacleInformation)

                if len(data) < tlvc_ofs + base_size:
                    max_obs_by_len = 0
                else:
                    payload_room = len(data) - (tlvc_ofs + base_size)
                    max_obs_by_len = max(0, payload_room // obs_size)

                obs_count = min(reported_obs, max_obs_by_len)

                obstacles = []
                for i in range(obs_count):
                    ofs = tlvc_ofs+sizeof(V2x_App_SI_TLVC)+(i*sizeof(ObstacleInformation))
                    if len(data) < ofs + sizeof(ObstacleInformation):
                        pass
                    else:
                        obstacle = ObstacleInformation.from_buffer_copy(data[ofs:ofs+sizeof(ObstacleInformation)])
                        obstacles.append(obstacle)
                
                state, path, obstacles = self.organize_data(len(data), sharing_information, obstacles)
                rx_message = self.get_log_datum(self.be64toh(get_time), state, path, obstacles)
                
                # 비동기 로깅
                try:
                    self._log_queue.put_nowait(('rx', self.tx_cnt_from_rx, rx_message))
                    self._log_queue.put_nowait(('perf', 0, self.get_performance_log()))
                except:
                    pass
                
                latest_result = [state, path, obstacles]

            return latest_result
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
                rtt = round((time.perf_counter()-ts[1])*1000)
                self.communication_performance['rtt'] = rtt
                break
        self.rtt_ts_list[:] = [ts for ts in self.rtt_ts_list if ts[0] >= tx_cnt_from_rx]
    
    def get_communication_performance(self):
        if self.tx_cnt < 1 or self.rx_cnt < 1:
            return None
        else:
            return self.communication_performance

    def calc_delay(self, rx_timestamp):
        try:
            rx_ts_ms = self.be64toh(rx_timestamp)  # 밀리초 Unix timestamp
        except Exception:
            rx_ts_ms = int(rx_timestamp)
        
        # 밀리초 단위로 정확한 delay 계산
        current_ms = int(time.time() * 1000)
        delay_ms = current_ms - rx_ts_ms
        
        self.communication_performance['delay'] = round(delay_ms, 2)
        
    def receive(self):
        # 수신 타임아웃 최소화
        self.fd.settimeout(0.003)  # 10ms
        try:
            chunk = self.fd.recv(sizeof(c_char)*MAX_TX_PACKET_TO_OBU)
            if not chunk:
                return None
            self._rx_stream.extend(chunk)

            frames = []
            while True:
                if len(self._rx_stream) < 6:
                    break
                magic = struct.unpack_from('!I', self._rx_stream, 0)[0]
                frame_len_field = struct.unpack_from('!H', self._rx_stream, 4)[0]
                total_len = 6 + frame_len_field

                if len(self._rx_stream) < total_len:
                    break

                frame = bytes(self._rx_stream[:total_len])
                del self._rx_stream[:total_len]
                frames.append(frame)

            
            return frames if frames else None
        except socket.timeout:
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
        
        # Status용 타임스탬프 (KETI 형식)
        keti_time = self.get_keti_time()
        p_status.contents.timestamp = self.htobe64(keti_time)
        
        crc_data = bytearray(p_status.contents)
        if self.chip == 'out':
            p_status.contents.cpu_temp = 50
            p_status.contents.peri_temp = 50
            p_status.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_STATUS_CommUnit_V2)-2))
        else:
            p_status.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_STATUS_CommUnit)-2))
    
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

    def be64toh(self, value):
        return struct.unpack('>Q', struct.pack('>Q', value))[0]

    def print_hexa(self, data):
        self.logger.info("Binary data in hexadecimal: ", end="")
        for byte in data:
            print(f"{byte:02X} ", end="")
        print()
    
    def get_log_datum(self, time_stamp, vehicle_state, vehicle_path, vehicle_obstacles):
        # 타임스탬프를 읽기 쉬운 형식으로 변환
        try:
            dt = datetime.fromtimestamp(time_stamp / 1000.0)
            timestamp_str = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        except:
            timestamp_str = str(time_stamp)
            
        if len(vehicle_state) > 0:
            state = f"Shared Message\nts:{timestamp_str} state:{vehicle_state[0]} signal:{vehicle_state[1]} lat:{vehicle_state[2]:.3f} lng:{vehicle_state[3]:.3f} h:{vehicle_state[4]:.2f} v:{vehicle_state[5]:.2f}\n"
        else:
            state = "No message to Send\n"
        if vehicle_path != [] and len(vehicle_path[0]) > 1:
            path = f"path: x={vehicle_path[0][0]:.3f} y={vehicle_path[0][1]:.3f} ~ x={vehicle_path[-1][0]:.3f} y={vehicle_path[-1][1]:.3f}\n"
        else:
            path = f"There is no path\n"
        obstacle_number = f"There are {len(vehicle_obstacles)} obstacles\n"
        obstacles_info = ""
        if len(vehicle_obstacles) > 0:
            for i, obs in enumerate(vehicle_obstacles):
                obstacles_info += f"[{i}] cls:{obs[0]} x:{obs[1]:.2f} y:{obs[2]:.2f} h:{obs[3]:.2f} v:{obs[4]:.2f}\n"
        return state+path+obstacle_number+obstacles_info

    def get_performance_log(self):
        performance_str = "Communication Performance \n" + " ".join([f"{key}: {value}" for key, value in self.communication_performance.items()])
        return performance_str+"\n\n"
    
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
        
        mbps = (window_bytes * 8) / (actual_window_time * 1000000)
        return round(mbps, 3)

    def update_throughput_metrics(self, send_size):
        """throughput 메트릭 업데이트"""
        current_time = time.perf_counter()
        
        if self.first_tx_time is None:
            self.first_tx_time = current_time
        
        self.tx_history.append((current_time, send_size))
        self.total_bytes_sent += send_size
        
        if current_time - self.last_mbps_calc_time >= self.mbps_calc_interval:
            window_mbps = self.calculate_window_mbps(current_time)
            self.communication_performance['mbps'] = window_mbps
            self.last_mbps_calc_time = current_time
    
    def _update_prr_pps(self, seq: int):
        t = time.monotonic()
        s = int(seq)

        self._win.append((t, s))

        L_ms = 200
        cutoff = t - (self._W + L_ms/1000.0)

        changed = False
        while self._win and self._win[0][0] < cutoff:
            self._win.popleft()
            changed = True
        if changed:
            self._seen = {x[1] for x in self._win}

        self._seen.add(s)

        if not self._win or not self._seen:
            self.communication_performance['packet_rate'] = 0.0
            return

        uniq = len(self._seen)
        mn, mx = min(self._seen), max(self._seen)
        sent_est = max(1, mx - mn + 1)

        prr = (uniq / sent_est) * 100.0

        self.communication_performance['packet_rate'] = round(prr, 2)