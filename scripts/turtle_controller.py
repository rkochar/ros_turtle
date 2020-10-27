#!/usr/bin/env python
import rospy
from ros_turtle.msg import Move
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from ros_turtle.srv import SetSpeed, SetSpeedResponse

# Make publisher to turtle and global variable speed with value 2.
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
speed = 2.0

def move_turtle_without_timeout(twist_message, duration):
    """
    Moves the turtle with the specified speeds for a specific amount of time.

    :param twist_message: the angular and linear speeds the turtle needs to move
    :param duration: the amount of time in seconds the turtle needs to move before it stops
    """
    time_left = duration
    while True:
        if time_left > 0.9: # 0.9 is less than the 1s timeout of the turtle.
            pub.publish(twist_message)
            rospy.sleep(0.9)
            time_left -= 0.9
        else:
            pub.publish(twist_message)
            rospy.sleep(time_left)
            pub.publish(Twist())
            break

def get_move_forward_message(velocity):
    """
    Moves turtle forward by specified distance.

    :param velocity: X component of linear velocity of turtle
    :return: a twist message that moves forward witht the specified linear velocity
    """
    twist_message = Twist()
    twist_message.linear.x = velocity
    return twist_message


def get_turn_left_message(angular_velocity):
    """
    Turns the turtle direction by specified radians.
    :param angular_velocity: Z component of angular velocity to turn turtle (in radians).
    :return: a twist message that turns counterclockwise with the specified angular velocity
    """
    twist_message = Twist()
    twist_message.linear.x = 0
    twist_message.angular.z = angular_velocity
    return twist_message


def make_square_side(distance):
    """
    Makes a square with the turtle of specified side.

    :param distance: Side of square.
    :return: None
    """
    # Move forward for a specific distance and speed.
    global speed
    time_to_wait = distance / speed
    twist_message = get_move_forward_message(speed)
    move_turtle_without_timeout(twist_message, time_to_wait)

    # Turn 90 degrees with a specific angular speed.
    rad = math.pi / 2
    time_to_wait_turn = rad / speed
    twist_message_turn = get_turn_left_message(speed)
    move_turtle_without_timeout(twist_message_turn, time_to_wait_turn)


def make_circle(radius):
    """
    Makes a circle of specified radius with the turtle.

    :param radius: Radius of circle.
    :return: None
    """
    global speed
    linear_speed = speed
    circumference = 2 * math.pi * radius
    time_taken = circumference / linear_speed
    angular_speed = (2 * math.pi) / time_taken

    twist_message = Twist()
    twist_message.linear.x = linear_speed
    twist_message.angular.z = angular_speed

    move_turtle_without_timeout(twist_message, time_taken)


def callback(data):
    """
    Callback method of topic "Move".

    :param data: Object of msg Move.
    :return: None
    """
    # Circle
    if data.type == "circle":
        rospy.loginfo("Circle")
        make_circle(data.range)

    # Square
    elif data.type == "square":
        rospy.loginfo("Square side: '{0}'".format(
            str(data.range)))
        # Make all 4 sides of the square.
        for i in range(4):
            make_square_side(data.range)

    # Reset turtle to centre and clean the screen.
    elif data.type == "reset":
        rospy.loginfo("Reset")
        rospy.ServiceProxy('reset', Empty)()

    # Centre the turtle.
    elif data.type == "center":
        rospy.loginfo("Center")
        rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)(5.5, 5.5, 0)

    # # Change speed.
    # elif data.type == 'speed':
    #     global speed
    #     speed *= data.range
    #     rospy.loginfo("Speed '{0}' ".format(float(speed)))

    # Default case.
    else:
        rospy.loginfo("Invalid type: ", data.type)


def speed_callback(data):
    """
    Callback of rosservice speed.

    :param data: Object of srv SetSpeed.
    :return: SetSpeedResponse
    """
    global speed
    speed *= data.speed
    return SetSpeedResponse()


def listener():
    """
    Listens to the topic "move_turtle" and service "set_speed" and calls the appropriate callback methods.
    :return: None
    """
    rospy.init_node('turtle_controller', anonymous=True)

    # Listen to topic and service. Throw exception when there is an unexpected error.
    try:
        rospy.Subscriber('move_turtle', Move, callback)
        rospy.Service('set_speed', SetSpeed, speed_callback)
    except rospy.ServiceException as exp:
        print("ServiceException in turtle_controller.py", + str(exp))

    # Spins forever.
    rospy.spin()


if __name__ == '__main__':
    """
    Main method.
    """
    listener()
