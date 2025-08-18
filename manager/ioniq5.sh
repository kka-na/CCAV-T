#!/bin/bash
cd ../sharing_info
python3 main.py ego Midan 0&
cd ../ui/visualizer
python3 visualizer.py ego 0&
cd ../
python3 ui.py ego 0&
cd ../v2x
python3 main.py ego 0 out &
cd ../selfdriving
python3 main.py ego ioniq5 Midan &
cd ../utils
python3 make_data.py ego &
cd ~/Documents/bags/0818
rosbag record /clicked_point /ego/look_a_head /ego/look_a_head_array /ego/plot_point /ego/plot_point_array /ego/visualizer/ego_car /ego/visualizer/ego_car_array /ego/visualizer/ego_car_info /ego/visualizer/ego_car_info_array /ego/visualizer/ego_obstacles /ego/visualizer/local_ego_path /ego/visualizer/local_ego_path_array /ego/visualizer/local_target_path /ego/visualizer/local_target_path_array /ego/visualizer/target_car /ego/visualizer/target_car_array /ego/visualizer/target_car_info /ego/visualizer/target_car_info_array /ego/visualizer/target_obstacles /gmsl_camera/dev/video2/compressed /gps/fix /gps/gps /gps/imu /imu/data_raw /initialpose /lmap_viz /mobinha/perception/lidar/track_box /move_base_simple/goal /novatel/oem7/bestgnsspos /novatel/oem7/bestpos /novatel/oem7/bestutm /novatel/oem7/bestvel /novatel/oem7/corrimu /novatel/oem7/inspva /novatel/oem7/odom /rosout /rosout_agg /tf