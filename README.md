# Self-Driving_Car_Capstone
Final project for Udacity's Self-Driving Car nano degree program
## Current State
-  waypoint updater is partially finished(publish waypoints in front of the car without regarding the traffic light or speed limit etc)
-  dbw node is partilly finished (use yaw_controller provided by udacity to get steering anges, use pid.py to get throttle value, the pid controller is initialized using 10, 0, 1 which arbitrarily choosen)
## issues
-  sometimes the car is able to move, but sometimes not. and when the car is not able to move, after killing the launch process, the car will move for a little while which indicates that the program is somehow blocked.
## Need Attention
It said in the course that the dbw_enabled is always true in the simlulator, which is not the truth, need to uncheck the box before "Manual" to set the simulator to "dirve by wire" mode
