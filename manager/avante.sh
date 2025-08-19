#!/bin/bash
cd ../sharing_info
python3 main.py target Midan 0&
cd ../ui/visualizer
python3 visualizer.py target 0&
cd ../
python3 ui.py target 0&
cd ../v2x
python3 main.py target 0 out &
cd ../selfdriving
python3 main.py target avante Midan &
cd ../utils
python3 make_data.py target &
cd ~/Documents/bags/0819
rosbag record rosbag record /clicked_point /target/look_a_head /target/look_a_head_array /target/plot_point /target/plot_point_array /imu/data_raw /novatel/oem7/bestpos /novatel/oem7/bestutm /novatel/oem7/corrimu /novatel/oem7/inspva /novatel/oem7/odom /target/TargetShareInfo /target/user_input /target/CommunicationPerformance /target/test_case 