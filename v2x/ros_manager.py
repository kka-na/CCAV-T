#!/usr/bin/env python
import rospy
import threading
import signal
import atexit
from datetime import datetime

from ccavt.msg import *
from std_msgs.msg import Float32MultiArray

def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")

class RosManager:
    def __init__(self, v2v_sharing, type):
        self.type = type
        rospy.init_node(f"{type}_v2v_sharing")
        self.v2v_sharing = v2v_sharing
        self.set_values()
        #self.init_log_file()
        self.set_protocol()
        
    def set_values(self):
        self.Hz = 5
        self.rate = rospy.Rate(self.Hz)
        self.info_received = False
        self.shutdown_event = threading.Event()

        self.vehicle_state = [0,0,0,0,0,0]
        self.vehicle_path = []
        self.vehicle_obstacles = []


    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.share_info_cb)
        self.pub_target_info = rospy.Publisher(f'{self.type}/TargetShareInfo', ShareInfo, queue_size=1)
        self.pub_communication_performance = rospy.Publisher(f'{self.type}/CommunicationPerformance', Float32MultiArray, queue_size=1)

    def share_info_cb(self, msg):
        self.vehicle_state = [ msg.state.data, msg.signal.data, msg.pose.x, msg.pose.y, msg.pose.theta, msg.velocity.data ]
        if len(self.vehicle_state) > 0:
            self.info_received = True
        paths = [[],[]]
        for p in msg.paths:
            paths[0].append(p.pose.x)
            paths[1].append(p.pose.y)
        self.vehicle_path = paths
        obstacles = []
        for o in msg.obstacles:
            obstacles.append((o.cls.data, o.pose.x, o.pose.y, o.pose.theta, o.velocity.data, o.distance.data, o.dangerous.data))
        self.vehicle_obstacles = obstacles
    
    def publish_calc(self, system):
        self.pub_communication_performance.publish(Float32MultiArray(data=list(system.values())))
        #self.log_system_values(system)
    
    def log_system_values(self, system):
        values = list(system.values())
        self.log_file.write(','.join(map(str, values)) + '\n')
        self.log_file.flush()
    
    def init_log_file(self):
        self.log_file = open(f"../log/{self.type}/{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt", "w")
        atexit.register(self.close_log_file)

    def close_log_file(self):
        if self.log_file:
            self.log_file.close()

    def publish(self, result):
        if result == [0,0,0]:
            rospy.logwarn("No Target Share Info to Publish")
            return
        
        vehicle_state = result[0]
        vehicle_path = result[1]
        vehicle_obstalce = result[2]

        share_info = ShareInfo()
        share_info.state.data = vehicle_state[0]
        share_info.signal.data = vehicle_state[1]
        share_info.pose.x = vehicle_state[2]
        share_info.pose.y = vehicle_state[3]
        share_info.pose.theta = vehicle_state[4]
        share_info.velocity.data = vehicle_state[5]
        
        if len(vehicle_path[0]) > 0:
            for i, xs in enumerate(vehicle_path[0]):
                path = Path()
                path.pose.x = xs
                path.pose.y = vehicle_path[1][i]
                share_info.paths.append(path)
        
        if len(vehicle_obstalce) > 0:
            for obs in vehicle_obstalce:
                obstacle = Obstacle()
                obstacle.cls.data = obs[0]
                obstacle.pose.x = obs[1]
                obstacle.pose.y = obs[2]
                obstacle.pose.theta = obs[3]
                obstacle.velocity.data = obs[4]
                obstacle.distance.data = obs[5]
                share_info.obstacles.append(obstacle)
            
        self.pub_target_info.publish(share_info)


    def do_tx(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            tx_res = self.v2v_sharing.do_tx(self.vehicle_state, self.vehicle_path, self.vehicle_obstacles)
            if tx_res<0:
                rospy.logerr("[V2X ROSManager] Tx Send Data Failed")
                return -1
            self.rate.sleep()
    
    def do_rx(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            rx_res = self.v2v_sharing.do_rx()
            if rx_res == None:
                rospy.logerr("[V2X ROSManager] No Rx Data from Target")
                return -1
            else:
                self.publish(rx_res)
            self.rate.sleep()
    
    def do_calc_rate(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            calc_rates_res = self.v2v_sharing.do_calc_rate(self.Hz)
            if calc_rates_res < 0:
                rospy.logerr("[V2X ROSManager] Rx Not Working")
                rate.sleep()
                continue
            rate.sleep()

    def do_calc(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            calc_res = self.v2v_sharing.do_calc()
            if calc_res == -1:
                pass
            else:
                self.publish_calc(calc_res)
            self.rate.sleep()
            

    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        rospy.loginfo("RosManager V2V Sharing Start")
        
        if self.v2v_sharing.set_obu() < 0 :
            return -1
        
        sharing_state = 1

        thread1 = threading.Thread(target=self.do_tx)
        thread2 = threading.Thread(target=self.do_rx)
        thread3 = threading.Thread(target=self.do_calc)
        thread4 = threading.Thread(target=self.do_calc_rate)

        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()

        try:
            thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()
        
        except KeyboardInterrupt:
            self.shutdown_event.set()
            thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()       
        
        return sharing_state
        
        

