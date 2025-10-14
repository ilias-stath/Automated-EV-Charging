# Automated-EV-Charging
This project was created during my internship at Parity IKE and it is an automated electric vehicle charging procedure that charges vehicles having the ccs2 charging port. 
The robotic arm used is the ur16e from Universal Robotics outfitted with a camera and a ccs2-outlet.
It uses the camera to detect the ccs2-inlet port and find its 3d position and orientation. 
After that it takes the neccesairy steps to approach the inlet and begin the charging.
When the charging is complete it returns to its original location and the proccess can begin again.

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
  
