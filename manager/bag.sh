#!/bin/bash
cd ../sharing_info
python3 main.py ego Midan 0&
cd ../ui/visualizer
python3 visualizer.py ego 0&
cd ../
python3 ui.py ego 0