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

def make_square(range):
    move_forward(range)
    rospy.sleep(1)
    
    rad = math.pi/2
    turn_left(rad)
    rospy.sleep(1)
    #twist_message.linear.x = 0

def square(range):
    # Move forward for 'range' steps.
    twist_message = Twist()
    twist_message.linear.x = range
    pub.publish(twist_message)
    rospy.sleep(1)
    twist_message.linear.x = 0

    # Turn turtle
    rad = math.pi / 2
    twist_message.angular.z = rad
    pub.publish(twist_message)
    rospy.sleep(1)
    twist_message.linear.x = 0
    pub.publish(twist_message)

def make_circle(radius):
    twist_message = Twist()
    speed = radius * 2 * math.pi

    twist_message.linear.x = speed # 2 * radius * math.pi
    #twist_message.linear.y = 3
    #twist_message.linear.z = 7

    #twist_message.angular.x = 3
    #twist_message.angular.y = 1
    twist_message.angular.z = speed / radius
    #for i in range(10):
    pub.publish(twist_message)
    # rospy.sleep(time)

def circle(radius):
    speed = 2
    time = radius * 2 * math.pi / speed 
    cond = True
    
    while time > 0:
        if time >= 0.5:
            make_circle(radius, speed)
            time -= 0.5
            rospy.sleep(0.5)
        else:
            make_circle(radius, speed)
            time = 0
            rospy.sleep(time)
        

def callback(data):
    # pub = rospy.Publisher('/turtleass/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if data.type == "circle":
        rospy.loginfo("circle")
        make_circle(data.range)
    elif data.type == "square":
        rospy.loginfo("square")
        for i in range(4):
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
