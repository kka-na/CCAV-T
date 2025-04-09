#!/bin/bash
cd ../sharing_info
python3 main.py ego Midan 1&
cd ../ui/visualizer
python3 visualizer.py ego 1&
cd ../
python3 ui.py ego 1 &
# cd ../v2x
# python3 main.py ego 1 out &
cd ../selfdriving
python3 main.py ego simulator Midan 
#netstat -anp | grep 47347