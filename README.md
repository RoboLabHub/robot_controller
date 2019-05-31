If your robot uses steppers motors or smart servos (Dynamixel XL430-W250 or XL-320) then you can use cheap RAMPS 1.4 board or OpenCM_9.04 to control them from ROS.

Links to electronics parts:<br/>
https://reprap.org/wiki/RAMPS_1.4<br/>
http://emanual.robotis.com/docs/en/parts/controller/opencm904/<br/>
http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/<br/>
http://emanual.robotis.com/docs/en/dxl/x/xl320/<br/>

Links to ROS:<br/>
http://wiki.ros.org/rostopic<br/>
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics<br/>
http://wiki.ros.org/rosserial_python<br/>
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi<br/>

In my setup the RAMPS 1.4 and OpenCM 9.04 connected directly to Raspberry Pi USB ports, but you may connect them to desktop computer instead of Raspberry Pi.

Here you can find the images with preinstalled ROS on Raspberry Pi and Virtual PC:<br/>
https://downloads.ubiquityrobotics.com/

As example, I will show you how to control Niryo robot (https://niryo.com/niryo-one/) with RAMPS 1.4 and OpenCM_9.04.
The Niryo robot uses:
- 3 stepper motors to move joint_0, joint_1 and joint_2
- 2 servos (Dynamixel XL430-W250) to move joint_3 and joint_4
- 2 servos (Dynamixel XL-320) to move joint_5 and gripper

The arduino programs for RAMPS 1.4 and OpenCM_9.04:<br/>
https://github.com/RoboLabHub/Niryo/tree/master/Controllers

Prerequisite commands:<br/>
sudo apt-get update<br/>
sudo apt-get upgrade<br/>
sudo apt-get install ros-kinetic-rosserial*<br/>

mkdir -p ~/catkin_ws/src<br/>
cd ~/catkin_ws/src && catkin_init_workspace<br/>
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc<br/>
source ~/.bashrc<br/>
git clone --recursive https://github.com/RoboLabHub/robot_controller<br/>

cd ~/catkin_ws && catkin_make<br/>

To fix problem with USB ports on ubuntu you need to do these steps:<br/>
sudo cp ~/catkin_ws/src/robot_controller/99-robot.rules /etc/udev/rules.d/ <br/>
sudo udevadm control --reload-rules<br/>
sudo udevadm trigger<br/>

After connecting and setup electronics run the following command to launch ROS serial:<br/>
roslaunch robot_controller rosserial.launch

The following ROS topic are used for controlling and monitoring the state of robot arm:<br/>
/enable_motors<br/>
/stepper_goal<br/>
/stepper_state<br/>
/servo_goal<br/>
/servo_state<br/>
/gripper_goal<br/>

Use the following ROS commands in separate terminal to control the robot arm motors:

To enable/disable motors (stepper motors and smart servo motors):<br/>
rostopic pub /enable_motors -1 std_msgs/Int16 1<br/>

To move stepper motors (for example, joint_0 to 105, joint_1 to 2000 and joint_3 to 350):<br/>
rostopic pub /stepper_goal std_msgs/Int16MultiArray -1 -- '[[], 0]' '[105, 2000, 350]'<br/>

To move smart servo motors (for example, joint_4 to 10, joint_5 to 500 and joint_6 to 400):<br/>
rostopic pub /servo_goal std_msgs/Int16MultiArray -1 -- '[[], 0]' '[10, 500, 400]'<br/>

To control gripper smart servo position (set position to 1023):<br/>
rostopic pub /gripper_goal -1 std_msgs/Int16 1023<br/>

You can also control your robot by python script to make continues movement:<br/>
chmod +x ~/catkin_ws/src/robot_controller/scripts/example.py<br/>
rosrun robot_controller example.py<br/>

But if you want that you robot do more complex moves, then you should use the MoveIt package.<br/>
Check this link for more info:<br/>
https://github.com/RoboLabHub/Niryo/tree/master/niryo_one_driver<br/>

Related video:<br/>


