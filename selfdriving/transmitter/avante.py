#!/usr/bin/python3
import can
import cantools
import asyncio

import rospy

class Avante():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.dbc = cantools.database.load_file('./dbc/cn7.dbc')
        self.setup_message_dicts()
        self.setup_encode_handler()

        self.current_velocity = 0.0
        self.alive_cnt = -1

    def set_actuator(self, msg):
        self.EAIT_Control_02['EPS_Cmd'] = msg[1]
        self.EAIT_Control_02['ACC_Cmd'] = msg[0]
    
    def set_user_input(self, msg):
        state = int(msg['state'])
        self.EAIT_Control_01['EPS_En'] = state
        self.EAIT_Control_01['ACC_En'] = state

    def update_alive_cnt(self):
        self.alive_cnt += 1
        if self.alive_cnt >= 256:
            self.alive_cnt = 0
        self.EAIT_Control_01['Aliv_Cnt'] = self.alive_cnt
    
    def update_can_inputs(self):
        self.update_alive_cnt()
        dicts = []
        for values in self.encode_handler.values():
            dicts.append(values)
        return dicts

    async def execute(self):
        dicts = self.update_can_inputs()
        can_messages = self.encode_message(dicts)
        for can_message in can_messages:
            await asyncio.get_event_loop().run_in_executor(None, self.bus.send, can_message)

    def encode_message(self, dicts):
        can_messages = []
        for i, (key,value) in enumerate(self.encode_dbc.items()):
            message = self.dbc.encode_message(value, dicts[i])
            can_message = can.Message(arbitration_id=key, data=message, is_extended_id=False)
            can_messages.append(can_message)
        return can_messages
    
    def setup_encode_handler(self):
        self.encode_handler = {
            0x156: self.EAIT_Control_01,
            0x157: self.EAIT_Control_02,
        }

        self.encode_dbc = {
            0x156: 'EAIT_Control_01',
            0x157: 'EAIT_Control_02'
        }

    def setup_message_dicts(self):

        self.EAIT_Control_01 = {
            'EPS_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Override_Ignore': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Speed': 10,  # 1.0 * (value) [0,250]
            'ACC_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'AEB_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'Turn_Signal': 0x00,  # 0x00: Off, 0x01: Emergency, 0x02: Left, 0x03: Right
            'AEB_decel_value': 0,  # 0.01 * (value) [0.0,1.0]
            'Aliv_Cnt': 0  # 1.0 * (value) [0,255]
        }

        self.EAIT_Control_02 = {
            'EPS_Cmd': 0,  # 0.1 * (value) [-500, 500] "deg"
            'ACC_Cmd': 0  # 0.01 * (value + 10.23) [-3, 1] "m/s^2"
        }
    
    def cleanup(self):
        self.bus.shutdown()