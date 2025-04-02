#!/bin/bash
cd ../sharing_info
python3 main.py target Solbat &
cd ../ui/visualizer
python3 visualizer.py target &
cd ../
python3 ui.py target &
cd ../v2x
python3 main.py target 0 out &
cd ../selfdriving
python3 main.py target avante Solbat &
cd ../utils
python3 make_data.py target