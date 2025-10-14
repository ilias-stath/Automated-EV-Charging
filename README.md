# Automated-EV-Charging
## Overview
This project was created during my internship at Parity IKE and it is an automated electric vehicle charging procedure that charges vehicles having the ccs2 charging port. 
The robotic arm used is the ur16e from Universal Robotics outfitted with a camera and a ccs2-outlet.
It uses the camera to detect the ccs2-inlet port and find its 3d position and orientation. 
After that it takes the neccesairy steps to approach the inlet and begin the charging.
When the charging is complete it returns to its original location and the proccess can begin again.

<img width="1684" height="948" alt="Screenshot from 2025-10-13 22-35-40" src="https://github.com/user-attachments/assets/fd1f2ccb-93a4-463f-a72f-247dfa0f10b3" />
![charging_procedure](https://github.com/user-attachments/assets/de89eef9-21f0-48b5-b7ad-6ee6cd4595e9)



## System requirements
The project was compiled in ros2 humble using python3.10.
The simulation was done in gazebo fortress.
You must install the specific requirement of the project in order to run correctly.

## Getting started
* Download the project using the git clone command.
```
https://github.com/ilias-stath/Automated-EV-Charging/tree/main
```
* After that install the specific requirements
```
pip install -r requirements.txt
```
* Some other installations that you might need to do are the following
Run this command to be sure that everything is correctly installed:
```
rosdep install --from-paths src --ignore-src -r -y
```
After that install also this libraries:
```
sudo apt install \
  ros-humble-filters \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-interface \
  ros-humble-hardware-interface \
  ros-humble-control-toolbox \
  ros-humble-controller-manager \
  ros-humble-gz-ros2-control \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-realtime-tools \
  ros-humble-generate-parameter-library
```





