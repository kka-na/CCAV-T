#!/bin/bash
cd ../ui/visualizer
python3 visualizer.py target 0&
cd ../
python3 ui.py target 0