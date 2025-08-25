import rospy
import sys
import signal
import tf
import math

from geometry_msgs.msg import PoseStamped, Pose

def signal_handler(sig, frame):
    sys.exit(0)

class ObjectSimulator:
    def __init__(self):
        self.object = Pose()
        self._steer = 0
        self._accel = 0
        self._brake = 0
        self.pub_sim_object = rospy.Publisher('/simulator/object', Pose, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

    def goal_cb(self, msg):
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        pose = Pose()
        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y
        pose.position.z = 1
        pose.orientation.x = 0.1
        pose.orientation.y = math.radians(yaw)
        self.object = pose

    def publish_object(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pub_sim_object.publish(self.object)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('object_simulator', anonymous=False)
    object_simulator = ObjectSimulator()
    object_simulator.publish_object()

if __name__ == "__main__":
    main()
