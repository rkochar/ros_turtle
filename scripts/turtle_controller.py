#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ros_turtle.msg import Move 
from geometry_msgs.msg import Twist
import math

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def move_forward():
    twist_message = Twist()
    twist_message.linear.x = 1
    pub.publish(twist_message)

def turn_left():
    twist_message = Twist()
    twist_message.angular.z = -math.pi/2
    pub.publish(twist_message)

def turn_and_move(): 
    move_forward()
    rospy.sleep(1)
    turn_left()
    rospy.sleep(1)

def make_square():
    turn_and_move()
    turn_and_move()
    turn_and_move()
    turn_and_move()

def callback(data):
    # pub = rospy.Publisher('/turtleass/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if data.type == "circle":
        rospy.loginfo("cirlce")
    elif data.type == "square":
        rospy.loginfo("square")
        make_square()
    elif data.type == "reset":
        rospy.loginfo("reset")
    else:
        rospy.loginfo("no such type")


def listener():
    rospy.init_node('turtle_controller', anonymous=True)

    rospy.Subscriber('move_turtle', Move, callback)
#    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
