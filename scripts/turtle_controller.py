#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ros_turtle.msg import Move 
from geometry_msgs.msg import Twist

pub = None

def callback(data):
    # pub = rospy.Publisher('/turtleass/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    if data.type == "circle":
        rospy.loginfo("cirlce")
    elif data.type == "square":
        rospy.loginfo("square")
        twist_message = Twist()
        twist_message.linear.x = 2
        pub.publish(twist_message)
    elif data.type == "reset":
        rospy.loginfo("reset")
    else:
        rospy.loginfo("no such type")


def listener():
    rospy.init_node('turtle_controller', anonymous=True)

    pub = rospy.Publisher('/turtleass/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('move_turtle', Move, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
