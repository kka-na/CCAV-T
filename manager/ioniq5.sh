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
cd ~/Documents/bags/0416
rosbag record -a