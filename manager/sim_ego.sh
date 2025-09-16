#!/bin/bash
cd ../sharing_info
python3 main.py ego KIAPI 1&
cd ../ui/visualizer
python3 visualizer.py ego 1&
cd ../
python3 ui.py ego 1 &
cd ../selfdriving
python3 main.py ego simulator KIAPI &
cd ../v2x
python3 main.py ego 0 out &
cd ../utils
python3 make_data.py ego 
#netstat -anp | grep 47347