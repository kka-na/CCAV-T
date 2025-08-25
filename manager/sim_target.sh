#!/bin/bash
cd ../sharing_info
python3 main.py target Solbat 1&
cd ../ui/visualizer
python3 visualizer.py target 1&
cd ../
python3 ui.py target 1&
# cd ../v2x
# python3 main.py target 2 in &
cd ../selfdriving
python3 main.py target simulator Solbat  
