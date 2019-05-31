#!/usr/bin/env python

import roslib, rospy
from std_msgs.msg import Int16, Int16MultiArray, Int32MultiArray


class RobotArm(object):
    def __init__(self):
        self.enable_ = rospy.Publisher('/enable_motors', Int16, queue_size=1)

        self.steppers = rospy.Publisher('/stepper_goal',  Int16MultiArray, queue_size=1)
        self.servos   = rospy.Publisher('/servo_goal',    Int16MultiArray, queue_size=1)

        self.gripper  = rospy.Publisher('/gripper_goal',  Int16, queue_size=1)

        self.stepper_sub = rospy.Subscriber('/stepper_state', Int16MultiArray, self.stepper_callback)
        self.servo_sub   = rospy.Subscriber('/servo_state',   Int32MultiArray, self.servo_callback)

    def stepper_callback(self, msg):
        self.joint0     = msg.data[0]
        self.joint1     = msg.data[1]
        self.joint2     = msg.data[2]
        print("Stepper pos: {} {} {}".format(self.joint0, self.joint1, self.joint2))
        pass

    def servo_callback(self, msg):
        self.joint3     = msg.data[0]
        self.joint4     = msg.data[1]
        self.joint5     = msg.data[2]
        self.gripperPos = msg.data[3]
        print("Servo pos: {} {} {} Gripper pos: {}".format(self.joint3, self.joint4, self.joint5, self.gripperPos))

    def enable_motors(self, enable):
        rospy.sleep(0.1)
        self.enable_.publish(Int16(enable))

    def move_steppers(self, joint0, joint1, joint2):
        data = [joint0, joint1, joint2]
        self.steppers.publish(Int16MultiArray(None, data))

    def move_servos(self, joint3, joint4, joint5):
        rospy.sleep(0.1)
        data = [joint3, joint4, joint5]
        self.servos.publish(Int16MultiArray(None, data))        

    def set_gripper(self, position):
        rospy.sleep(0.1)
        self.gripper.publish(Int16(position))

if __name__ == '__main__':
    print("Running robot...")
    rospy.init_node('robot_controller', anonymous=True)
    robot = RobotArm()

    robot.enable_motors(1)
    robot.set_gripper(350)
    robot.move_steppers(100, 200, 300)
    robot.move_servos(100, 200, 300)
    rospy.sleep(1)
    robot.set_gripper(0)
    rospy.sleep(1)
    print("Exit")

