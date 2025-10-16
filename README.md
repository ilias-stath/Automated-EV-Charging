# Automated-EV-Charging
## Overview
This project was created during my internship at Parity IKE and it is an automated electric vehicle charging procedure that charges vehicles having the ccs2 charging port. 
The robotic arm used is the ur16e from Universal Robotics outfitted with a camera and a ccs2-outlet.
It uses the camera to detect the ccs2-inlet port and find its 3d position and orientation. 
After that it takes the neccesairy steps to approach the inlet and begin the charging.
When the charging is complete it returns to its original location and the proccess can begin again.

<img width="1684" height="948" alt="Screenshot from 2025-10-13 22-35-40" src="https://github.com/user-attachments/assets/fd1f2ccb-93a4-463f-a72f-247dfa0f10b3" />

![charging_procedure](https://github.com/user-attachments/assets/9d590b2d-2743-4ff1-a864-6b38f613ed35)




## System requirements
The project was created in a vulcanexus docker container.<br>
The project was compiled in ros2 humble using python3.10.<br>
The simulation was done in gazebo fortress.<br>
You must install the specific requirement of the project in order to run correctly.<br>
**Also read the notes at the end of the Readme**

## Getting started
* First create the necessary folders
```
mkdir -p ev_charge/src
```
* Download the project inside the src folder using the git clone command.
```
git clone https://github.com/ilias-stath/Automated-EV-Charging.git
```
* Then install the specific requirements:
```
pip install -r requirements.txt
```
* After that, you must also install these libraries:
```
sudo apt update
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
* Next run these commands:
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
* At last, before you build you must export the path correctly so the gazebo fortress can locate the 3d-models
```
unset GZ_SIM_RESOURCE_PATH
nano ~/.bashrc
# add the export line at the end of the .bashrc
export GZ_SIM_RESOURCE_PATH=/root/ev_charge/install/ur_simulation_gz/share/ur_simulation_gz/models:/root/ev_charge/src/Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/models
source ~/.bashrc
echo $GZ_SIM_RESOURCE_PATH
```
* Alternatively, if you do not want to edit the .bashrc you can just export the paths
```
unset GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=/root/ev_charge/install/ur_simulation_gz/share/ur_simulation_gz/models:/root/ev_charge/src/Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/models
echo $GZ_SIM_RESOURCE_PATH
```

## Running the example
In order to run the example and test the procedure you must follow these 2 simple steps.<br>

* Launch the simulated enviroment
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```

* Launch the nodes (in a seperate terminal)
```
ros2 launch ur_simulation_gz EV_charge.launch.py
```

If you want to view the annotated image with the keypoints created by the model run this in a seperate terminal:
```
ros2 run yolo_model image_view
```
Note that the annotated images isn't in a constant feed.

## Important notes

* Always build and source before you launch the simulation or run the nodes
* You can edit the .bashrc with nano or vim. To install the just type:
```
sudo apt install nano
```
or
```
sudo apt install vim
```
* If you do not add the export line to the .bashrc and just export it, then if you reopen the pc or container you must do it again
* Wait until the simulated enviroment is correctly launched before you launch the nodes. One easy way to identify it is to wait until the logs stop.
