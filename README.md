# Implementation-of-Forward-and-Inverse-Kinematics-to-Perform-Pick-and-Place-on-UR5-Robot
1. Implementing inverse kinematics code to find the joint values required for moving to a given point in a UR5. 
2. Validation of the inverse kinematics using direct forward kinematics control of the robot to move to the said position. 
3. Pick and Place of random objects using the validated model.


Requirements 
1. UR5 Robot
2. Ubuntu 18
3. ROS Melodic
4. Robotiq Gripper

# Step 1 - Clone the following github repos in your src 
1. [Universal Robot](https://github.com/ros-industrial/universal_robot)
2. [Universal_Robot_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
3. [Robotiq](https://github.com/ros-industrial/robotiq)
4. Be sure to clone the melodic or melodic supported versions and build your environment using `catkin build`


# Step 2 - Networking Host Machine with UR5

1. Install the External_Control Program in UR5 Teach Pendant. Further details have been provided [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md)
2. Set the IP address of the robot as 192.168.0.100 in the teach pendant. 
3. Connect the UR5 controller to the host machine using an ethernet cable. 
4. Assign a static address, in our case, 192.168.0.77 . Assign the same along with port number as 500002, in the external control program in the teach pendant.
5. Run the external control program.
6. Run the following command, wherein tool_device_name refers to the robotiq gripper's name.  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 tool_device_name:=/tmp/ttyUR


# Step 3 - Inverse Kinematics Code
As the python file, ur5_inv_py has inverse kinematics for UR5 coded, in Newton Raphson Method, you would be required to add the home configurations, screw matrices, the end effector pose in the code. 

# Step 4 - Validation of inverse kinematics using Forward Kinematics
Using the topic that the robot subscribes to, /scaled_pos_joint_traj_controller/command trajectory_msgs/JointTrajectory, you can easily try to move the robot by publishing the joint values from the terminal. 
![ur5-2](https://user-images.githubusercontent.com/80807952/209034591-e896753a-8c30-4137-a2d5-a13aa5262a1a.png)
![ur5-3](https://user-images.githubusercontent.com/80807952/209034611-fc84a27e-628d-4c66-aa90-5433713a2dd7.png)

The above images shows successful validation of the inverse kinematics mode, by using the inverse kinematics solutions to make the robot do forward kinematics. In the above images, the robot is moved from the home position to the desired end effector position.

# Step 6 - Gripper Control

1. Activate gripper control using the command- rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
2. Open the gripper Ui using the command - rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

Given below are the images that showed point to point pick and place using Inverse Kinematics Solutions. 

![ur5-4](https://user-images.githubusercontent.com/80807952/209035131-6105c0c5-853b-4862-b86e-b5e2545ce733.png)

![ur5-5](https://user-images.githubusercontent.com/80807952/209035140-7d8f70e7-5f49-4cfa-9efb-974566733da6.png)
