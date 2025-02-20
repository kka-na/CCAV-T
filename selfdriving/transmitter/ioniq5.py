#!/usr/bin/python3
import can
import cantools
import rospy

class IONIQ5():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('./dbc_files/ioniq5.dbc')
        self.current_velocity = 0.0

    def receiver(self):
        data = self.bus.recv(0.2)
        try:
            if (data.arbitration_id == 0x280):
                res = self.db.decode_message(data.arbitration_id, data.data)
                RL = res['Gway_Wheel_Velocity_RL']
                RR = res['Gway_Wheel_Velocity_RR']
                self.current_velocity = (RR + RL)/7.2
        except Exception as e:
            print(e)

    def cleanup(self):
        self.bus.shutdown()

    def execute(self):
        while not rospy.is_shutdown():
            self.receiver()
        rospy.on_shutdown(self.cleanup)