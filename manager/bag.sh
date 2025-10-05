#!/bin/bash
# cd ../sharing_info
<<<<<<< HEAD
# python3 main.py ego KIAPI 0&
=======
# python3 main.py ego Midan 0&
cd ../v2x
python3 main.py ego 0 out &
>>>>>>> origin
cd ../ui/visualizer
python3 visualizer.py ego 0&
cd ../
python3 ui.py ego 0