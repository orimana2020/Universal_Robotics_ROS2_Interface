# ur_proj1
Universal Robotics ROS2 Interface



https://github.com/orimana2020/Universal_Robotics_ROS2_Interface/assets/69038641/f9e22aed-47af-4f02-8ce0-54e5cd0ad983



### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* Install ur driver ros2: (make sure to install ros2 humble)
* Install from Binary package!
* make sure to select humble branch
  ```sh
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main
  sudo apt-get install ros-${ROS_DISTRO}-ur
  sudo apt-get install python3-pip
  pip install setuptools==58.2.0
  ```
* Documentation:
  ```sh
  https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html
  ```
* Robot params:
  ```sh
  https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
  ```
### ROS2 Basics
  ```sh
  https://github.com/ros2/examples/tree/humble
  ```

### Build
```sh
colcon build
source install/setup.bash
 ```



### Run UR Interface

  
# Joint Trajectory Controller
* from workspace run:
open Terminal 1 and run

* Simulation:
```sh
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=true 
  ```

open Teminal 2 and navigate the workspace
```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```sh
ros2 run move_ur1 move_joint_trajectory 
```

* Real robot
```sh
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=false launch_rviz:=true
```
from the UR pendant run the ros program

open Teminal 2 and navigate the workspace
```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```sh
ros2 run move_ur1 move_joint_trajectory 
```



# Using MoveIt
```sh
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```
# and in another shell
```sh
ros2 launch move_ur1 ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true
```
