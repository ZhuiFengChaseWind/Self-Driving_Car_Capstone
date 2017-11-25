# ChaseWind (追风) Self-driving Car 
## Team Members:
- Hang Wang (Team Lead) : hwang12wou@gmail.com
- Yanqing Jing : 1410854263@qq.com
- Xiao Wang : wangxiaonku@gmail.com
- Calvin Low : calsaviour@gmail.com
- Yuzhu Li : rainbamboo554155@hotmail.com

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space


* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Issues and bug report
##  1. Code restructure
Since both pose_cb and traffic_cb update final_waypoints,the final_waypoints_pub should be extracted out from pose_cb


## 2. Stop Strategy
- If not meet the min distance, never make the v to 0
- If meet the min distance and light is still red, make v to 0 and stop update wp
- If red light detected but distance less than min distance(which means light turn red after car run through the cross), dont decelerate


## 3. Break logic code redundency
- Solved

## 4. Speed control
- Speed in real test either too fast or too slow
