# ENPM662_Final_Project

### Project Overview and Description
In this project, we have designed an ABB IRB-140 robotic manipulator to automate the assembly process. The manipulator is used to pick PCB components
from component tray and place on the PCB. The manipulator is used for precise, error-free, and fast operation of component assembly. We have
attached a camera at the end effector of the robot making it versatile for various functionalities.

### Development Team
Ameya Konkar

### External Dependencies
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](http://gazebosim.org/)
- [Moveit](https://moveit.ros.org/)
- [Opencv](https://github.com/opencv/opencv)

### Installation instructions

```
cd <catkin workspace>/src
sudo apt-get install git
git clone --recursive https://github.com/HrushikeshBudhale/decluttering_domestic_robot.git
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
