If you robot uses steppers motors or smart servos (Dynamixel XL430-W250 or XL-320) then you can use cheap RAMPS 1.4 board or OpenCM_9.04 to control them from ROS.

Links to electronics parts:
https://reprap.org/wiki/RAMPS_1.4
http://emanual.robotis.com/docs/en/parts/controller/opencm904/
http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
http://emanual.robotis.com/docs/en/dxl/x/xl320/

Links to ROS:
http://wiki.ros.org/rostopic
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
http://wiki.ros.org/rosserial_python
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

In my setup the RAMPS 1.4 and OpenCM 9.04 connected directly to Raspberry Pi USB ports, but you may connect them to desktop computer instead of Raspberry Pi.

Here you can find the images with preinstalled ROS on Raspberry Pi and Virtual PC:
https://downloads.ubiquityrobotics.com/

As example, I will show you how to control Niryo robot (https://niryo.com/niryo-one/) with RAMPS 1.4 and OpenCM_9.04.
The Niryo robot uses:
- 3 stepper motors to move joint_0, joint_1 and joint_2
- 2 servos (Dynamixel XL430-W250) to move joint_3 and joint_4
- 2 servos (Dynamixel XL-320) to move joint_5 and gripper

The arduino programs for RAMPS 1.4 and OpenCM_9.04:
https://github.com/RoboLabHub/Niryo/tree/master/Controllers

Prerequisite commands:
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-kinetic-rosserial*

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src && catkin_init_workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
git clone --recursive https://github.com/RoboLabHub/robot_controller

cd ~/catkin_ws && catkin_make

To fix problem with USB ports on ubuntu you need to do these steps:
sudo cp ~/catkin_ws/src/robot_controller/99-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

After connecting and setup electronics run the following command to launch ROS serial:
roslaunch robot_controller rosserial.launch

The following ROS topic are used for controlling and monitoring the state of robot arm:
/enable_motors
/stepper_goal
/stepper_state
/servo_goal
/servo_state
/gripper_goal

Use the following ROS commands in separate terminal to control the robot arm motors:

To enable/disable motors (stepper motors and smart servo motors):
rostopic pub /enable_motors -1 std_msgs/Int16 1

To move stepper motors (for example, joint_0 to 105, joint_1 to 2000 and joint_3 to 350):
rostopic pub /stepper_goal std_msgs/Int16MultiArray -1 -- '[[], 0]' '[105, 2000, 350]'

To move smart servo motors (for example, joint_4 to 10, joint_5 to 500 and joint_6 to 400):
rostopic pub /servo_goal std_msgs/Int16MultiArray -1 -- '[[], 0]' '[10, 500, 400]'

To control gripper smart servo position (set position to 1023):
rostopic pub /gripper_goal -1 std_msgs/Int16 1023

You can also control your robot by python script to make continues movement:
chmod +x ~/catkin_ws/src/robot_controller/scripts/example.py
rosrun robot_controller example.py

But it you want that you robot do more complex moves, then you should use the MoveIt package.
Check this link for more info:
https://github.com/RoboLabHub/Niryo/tree/master/niryo_one_driver

Related video:


