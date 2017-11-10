# waypoint_updater.py update
# TODO: Make a full break without restart attempt while redlight is on
# TODO:Check twist_controller  for break control 
##  1. Code restructure
Since both pose_cb and traffic_cb update final_waypoints,the final_waypoints_pub should be extracted out from pose_cb


## 2. Stop Strategy
- If not meet the min distance, never make the v to 0
- If meet the min distance and light is still red, make v to 0 and stop update wp
- If red light detected but distance less than min distance(which means light turn red after car run through the cross), dont decelerate


## 3. Break logic code redundency
