#!/bin/bash
cd ../sharing_info
python3 main.py target Solbat&
cd ../ui/visualizer
python3 visualizer.py target &
cd ../
python3 ui.py target &
cd ../v2x
python3 main.py target 2 out &
cd ../selfdriving
python3 main.py target simulator Solbat 
