#!/bin/bash

source /home/pi/.profile
workon cv
cd /home/pi/Tracking-Turret
python turret.py -m 1 -v y
