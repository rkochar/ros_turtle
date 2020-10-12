#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ros_turtle.msg import Move
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import Empty

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def move_forward(range):
    twist_message = Twist()
    twist_message.linear.x = range
    pub.publish(twist_message)

def turn_left(rad):
    twist_message = Twist()
    twist_message.angular.z = rad
    pub.publish(twist_message)

def turn_and_move(range):
    move_forward(range)

    rospy.sleep(1)
    rad = -math.pi/2
    turn_left(rad)
    rospy.sleep(1)

def make_square(range):
    turn_and_move(range)
    turn_and_move(range)
    turn_and_move(range)
    turn_and_move(range)

def make_circle(radius):
    twist_message = Twist()

    twist_message.linear.x = radius
    twist_message.linear.y = 0.0
    twist_message.linear.z = 0.0

    twist_message.angular.x = 0.0
    twist_message.angular.y = 0.0
    twist_message.angular.z = -6.25

    #for i in range(10):
    pub.publish(twist_message)


def callback(data):
    # pub = rospy.Publisher('/turtleass/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if data.type == "circle":
        rospy.loginfo("circle")
        make_circle(data.range)
    elif data.type == "square":
        rospy.loginfo("square")
        make_square(data.range)
    elif data.type == "reset":
        rospy.loginfo("reset")
        rospy.ServiceProxy('clear', std_srvs.srv.Empty)
    else:
        rospy.loginfo("no such type")


def listener():
    rospy.init_node('turtle_controller', anonymous=True)

    try:
        rospy.Subscriber('move_turtle', Move, callback)
    except rospy.ServiceException as exp:
        print("ServiceException in turtle_controller.py", + str(exp))

        #    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
