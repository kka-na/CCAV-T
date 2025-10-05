#!/bin/bash
# cd ../sharing_info
# python3 main.py ego Midan 0&
cd ../v2x
python3 main.py target 0 out &
cd ../ui/visualizer
python3 visualizer.py target 0&
cd ../
python3 ui.py target 0