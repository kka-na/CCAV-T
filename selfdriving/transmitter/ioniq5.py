#!/usr/bin/python3
import can
import cantools
import asyncio

import rospy

class IONIQ5():
    def __init__(self):
        try:
            self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        except Exception as e:
            print("No CAN")
        self.dbc = cantools.database.load_file('./transmitter/dbc/ioniq5.dbc')
        self.setup_message_dicts()
        self.setup_encode_handler()

        self.current_velocity = 0.0
        self.alive_cnt = -1
        self.state = 0

    def set_actuator(self, msg):
        if msg[0] > 0:
            accel = msg[0]
            brake = 0
        else:
            accel = 0
            brake = -msg[0]

        if self.state == 1:
            self.Control['Target_Accel'] = accel
            self.Control['Target_Brake'] = brake*0.6
            self.Control['PA_StrAngCmd'] = msg[1]
        else:
            self.Control['Target_Accel'] = 0
            self.Control['Target_Brake'] = 20
            self.Control['PA_StrAngCmd'] = 0

    
    def set_user_input(self, msg):
        if self.state == 1 and int(msg['state']) == 0:
            self.reset = 1
        self.state = int(msg['state'])
        self.Control['PA_Enable'] = self.state
        self.Control['LON_Enable'] = self.state
        if int(msg['signal']) == 1:
            self.Control['TURN_SIG_LEFT'] = 1
        elif int(msg['signal']) == 2:
            self.Control['TURN_SIG_RIGHT'] = 1
        else:
            self.Control['TURN_SIG_LEFT'] = 0
            self.Control['TURN_SIG_RIGHT'] = 0

    def update_alive_cnt(self):
        self.alive_cnt += 1
        if self.alive_cnt >= 256:
            self.alive_cnt = 0
        self.Control['Alive_cnt'] = self.alive_cnt
    
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
            0x210: self.Control
        }

        self.encode_dbc = {
            0x210: 'Control'
        }

    def setup_message_dicts(self):

        self.Control = {
            'PA_Enable': 0x00,  
            'PA_StrAngCmd': 0,  
            'LON_Enable': 0x00,  
            'Target_Brake': 0,  
            'Target_Accel': 0,  
            'Alive_cnt': 0,  
            'Reset_Flag': 0,  
            'TURN_SIG_LEFT': 0,
            'TURN_SIG_RIGHT': 0
        }

    def cleanup(self):
        self.bus.shutdown()