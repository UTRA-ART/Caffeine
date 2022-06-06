#!/usr/bin/python3

import cProfile

pr = cProfile.Profile() 

import os
import sys
import time
sys.path.insert(0, os.path.abspath(__file__))
import lane_detection_model_inference
pr.enable()
lane_detection_model_inference.main()
pr.disable()

pr.print_stats(sort='time')
input()