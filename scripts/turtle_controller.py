#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ros_turtle.msg import Move
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from ros_turtle.srv import SetSpeed, SetSpeedResponse

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
global speed
speed = 3

def move_forward(leng, speed):
    twist_message = Twist()
    twist_message.linear.x = speed
    pub.publish(twist_message)

def turn_left(rad, speed):
    twist_message = Twist()
    twist_message.linear.x = 0
    twist_message.angular.z = speed
    pub.publish(twist_message)

def make_square(leng):
    global speed
    time_to_wait = leng / speed
    move_forward(leng, speed)
    rospy.sleep(time_to_wait)
    pub.publish(Twist())

    rad = math.pi/2
    time_to_wait_turn = rad / speed
    turn_left(rad, speed)
    rospy.sleep(time_to_wait_turn)
    pub.publish(Twist())

def make_circle(radius):
    global speed
    linear_speed = speed
    circumference = 2 * math.pi * radius
    time_taken = circumference / linear_speed
    angular_speed = (2 * math.pi) / time_taken

    twist_message = Twist()
    twist_message.linear.x = linear_speed
    twist_message.angular.z = angular_speed
    while True:
        if (time_taken > 0.9):
            pub.publish(twist_message)
            rospy.sleep (0.9)
            time_taken -= 0.9
        else:
            pub.publish(twist_message)
            rospy.sleep(time_taken)
            pub.publish(Twist())
            break

#    global speed
#    twist_message = Twist()
#    s = radius * 2 * math.pi #* speed

#    twist_message.linear.x = s
#    twist_message.angular.z = s / radius
#    pub.publish(twist_message)

def callback(data):
    rate = rospy.Rate(10)

    if data.type == "circle":
        rospy.loginfo("circle")
        make_circle(data.range)

    elif data.type == "square":
        rospy.loginfo("Square side: '{0}'".format(
            str(data.range)))
        global speed
        rospy.loginfo("Speed '{0}' ".format(float(speed)))
        for i in range(4):
            make_square(data.range)

    elif data.type == "reset":
        rospy.loginfo("reset")
        rospy.ServiceProxy('reset', Empty)()

    elif data.type == "centre":
        rospy.loginfo("Centre")
        rospy.ServiceProxy('turtle1/teleport_absolute',
                TeleportAbsolute)(5.5, 5.5, 0)

    elif data.type == "move":
        rospy.loginfo("Move")
        make_square(data.range)

    elif data.type == 'speed':
        global speed
        speed *= data.range
        rospy.loginfo("Speed '{0}' ".format(float(speed)))

    else:
        rospy.loginfo("Invalid type: ", data.type)

def speed_callback(data):
#    rate = rospy.Rate(10)
#    if type(data) == float or type(data) == int:
#        #callback(data_object)
#        rospy.loginfo("Speed service: ", data)
        global speed
        speed *= data.speed
        return SetSpeedResponse()

def listener():
    rospy.init_node('turtle_controller', anonymous=True)

    try:
        rospy.Subscriber('move_turtle', Move, callback)
        rospy.Service('set_speed', SetSpeed, speed_callback)
    except rospy.ServiceException as exp:
        print("ServiceException in turtle_controller.py", + str(exp))

    rospy.spin()

if __name__ == '__main__':
    listener()
