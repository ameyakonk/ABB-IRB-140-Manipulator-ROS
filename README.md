# ENPM662_Final_Project

### Project Overview and Description
In this project, we have designed an ABB IRB-140 robotic manipulator to automate the assembly process. The manipulator is used to pick PCB components
from component tray and place on the PCB. The manipulator is used for precise, error-free, and fast operation of component assembly. We have
attached a camera at the end effector of the robot making it versatile for various functionalities.

### Development Team
Ameya Konkar  UID: 118191058
M.Eng in Robotics, University of Maryland, College Park

Ninad Harishchandrakar  UID: 118150819
M.Eng in Robotics, University of Maryland, College Park

### External Dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntuu)
- [Gazebo](http://gazebosim.org/)
- [Opencv](https://github.com/opencv/opencv)

```
ros-noetic-gazebo-ros-control               
ros-noetic-joint-state-controller           
ros-noetic-joint-state-publisher-gui        
ros-noetic-controller-manager               
ros-noetic-joint-trajectory-controller      
ros-noetic-rqt-joint-trajectory-controller  
ros-noetic-moveit                           
ros-noetic-moveit-visual-tools              
```

### Installation instructions

```
cd <catkin workspace>/src
sudo apt-get install git
git clone --recursive https://github.com/ameyakonk/ENPM662_Final_Project.git
cd ..
catkin build
source ./devel/setup.bash
```
### Launching the Simulation
For launching the simulation demo run following command.
This will start a gazebo environment of a empty world having two tables, two objects, one wooden plate and a ABB IRB-140 manipulator.
The manipulator will pick the objects on first table and place it on the wooden plate present on the other table.
```
roslaunch decluttering_domestic_robot simulation.launch
```
