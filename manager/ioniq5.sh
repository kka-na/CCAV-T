#!/bin/bash
cd ../sharing_info
python3 main.py ego Solbat 0&
cd ../ui/visualizer
python3 visualizer.py ego 0&
cd ../
python3 ui.py ego 0&
cd ../v2x
python3 main.py ego 0 out &
cd ../selfdriving
python3 main.py ego ioniq5 Solbat &
cd ../utils
python3 make_data.py ego &
cd ~/Documents/bags/0825
rosbag record /clicked_point /ego/look_a_head /ego/look_a_head_array /ego/plot_point /ego/plot_point_array /gmsl_camera/dev/video4/compressed /imu/data_raw /mobinha/perception/lidar/track_box /novatel/oem7/bestpos /novatel/oem7/bestutm /novatel/oem7/corrimu /novatel/oem7/inspva /novatel/oem7/odom /ego/TargetShareInfo /ego/user_input /ego/CommunicationPerformance /ego/test_case 