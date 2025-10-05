#!/bin/bash
cd ../sharing_info
python3 main.py ego Midan 0&
cd ../ui/visualizer
python3 visualizer.py ego 0&
cd ../
python3 ui.py ego 0 &
cd ../selfdriving
python3 main.py ego simulator Midan &
cd ../v2x
python3 main.py ego 0 out &
cd ../utils
python3 make_data.py ego 
#netstat -anp | grep 47347