import sys
import rospy
from hd_map.map import MAP
from visualization_msgs.msg import MarkerArray

rospy.init_node('map_only')
if len(sys.argv) != 3 :
    map_name = 'Solbat'
else:
    map_name = str(sys.argv[1])

map = MAP(map_name)
pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)

rate = rospy.Rate(0.05)
while not rospy.is_shutdown():
    pub_lmap_viz.publish(map.lmap_viz)
    rate.sleep()
