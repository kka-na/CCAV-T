#!/bin/bash
cd ../sharing_info
python3 main.py ego Solbat &
cd ../ui/visualizer
python3 visualizer.py ego &
cd ../
python3 ui.py ego &
cd ../v2x
python3 main.py ego 1 out &
cd ../selfdriving
python3 main.py ego simulator Solbat 
#netstat -anp | grep 47347