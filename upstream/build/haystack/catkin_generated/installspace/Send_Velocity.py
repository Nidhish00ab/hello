#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import signal


class InterruptManager:
    RUNNING = True

    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, *args):
        print("exiting", args)
        self.RUNNING = False


if __name__ == '__main__':
    rospy.init_node('joystick')
    rate = rospy.Rate(10.0)  # 10Hz
    status_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    velocity = Twist()
    interruptObj = InterruptManager()
    print("Entered Joystick ...")
    try:
        rospy.set_param("/haystack/joystick/linear_velocity", "0.10")
        rospy.set_param("/haystack/joystick/angular_velocity", "0.17")
        rospy.set_param("/haystack/joystick/current_linear_velocity", "0.0")
        rospy.set_param("/haystack/joystick/current_angular_velocity", "0.0")
        rospy.set_param("/haystack/joystick/state", "RELEASED")
    except Exception as e:
        print("Ros Error :", e)
    while interruptObj.RUNNING:
        state = rospy.get_param("/haystack/joystick/state")
        mode = rospy.get_param("/haystack/mode")
        while state == "PRESSED" and (mode == "MANUAL" or mode == "DISINFECT"):
            velocity.linear.x = float(rospy.get_param("/haystack/joystick/current_linear_velocity"))
            velocity.angular.z = float(rospy.get_param("/haystack/joystick/current_angular_velocity"))
            if velocity.linear.x != 0.0 and velocity.angular.z == 0.0:
                status_pub.publish(velocity)
            elif velocity.linear.x == 0.0 and velocity.angular.z != 0.0:
                status_pub.publish(velocity)
            rate.sleep()
            state = rospy.get_param("/haystack/joystick/state")
        rate.sleep()
    print("Exiting Joystick...")
