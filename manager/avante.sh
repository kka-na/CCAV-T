#!/bin/bash
cd ../sharing_info
python3 main.py target Midan 0&
cd ../ui/visualizer
python3 visualizer.py target 0&
cd ../
python3 ui.py target 0&
cd ../v2x
python3 main.py target 0 out &
cd ../selfdriving
python3 main.py target avante Midan &
cd ../utils
python3 make_data.py target