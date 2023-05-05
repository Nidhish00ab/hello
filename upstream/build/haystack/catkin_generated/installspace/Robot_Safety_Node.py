#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Imu, LaserScan, Image, Range
from ubiquity_motor.msg import MotorState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import signal
import threading
import configparser

X_AXIS_UPPER_THRESHOLD = -98
X_AXIS_LOWER_THRESHOLD = -118
Y_AXIS_UPPER_THRESHOLD = 10
Y_AXIS_LOWER_THRESHOLD = -10

CURRENT_THRESHOLD = 0.55
SENSOR_MSG_INITIAL_TIMEOUT = 20  # sec
SENSOR_MSG_TIMEOUT = 2

OVER_CURRENT_OCCURRENCE = 0
OVER_CURRENT_OCCURRENCE_LIMIT = 2

SENSOR_FAILURE_OCCURRENCE = 0
SENSOR_FAILURE_OCCURRENCE_LIMIT = 2
PUBLISH_VELOCITY = True
sensor_data_timer = None

Sensor_Status = {"motor": True, "lidar": True, "camera": True}
System_Status = {"battery": True, "arduino": True}

config = configparser.ConfigParser()
config.read("/haystack_disinfect_report/robot_config.ini")

if config.has_option("ROBOT", "MOTOR_CURRENT_THRESHOLD"):
    CURRENT_THRESHOLD = config["ROBOT"]["MOTOR_CURRENT_THRESHOLD"]


def imu_callback(data):
    # print(data.orientation)
    value = [round(math.degrees(i)) for i in
             euler_from_quaternion(
                 [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])]
    print("x = ", value[0], "   y = ", value[1], "  z = ", value[2])
    # -118 > -120 > -98
    if X_AXIS_LOWER_THRESHOLD > value[0] or value[0] > X_AXIS_UPPER_THRESHOLD or Y_AXIS_LOWER_THRESHOLD > value[1] \
            or value[1] > Y_AXIS_UPPER_THRESHOLD:
        robot_safety_publisher.publish(False)
        rospy.set_param("/haystack/mode", "IDLE")


def motor_state_callback(data):
    global OVER_CURRENT_OCCURRENCE, PUBLISH_VELOCITY
    Sensor_Status["motor"] = True
    if data.leftCurrent > CURRENT_THRESHOLD or data.rightCurrent > CURRENT_THRESHOLD:
        if OVER_CURRENT_OCCURRENCE > OVER_CURRENT_OCCURRENCE_LIMIT:
            try:
                mode = rospy.get_param("/haystack/mode")
            except:
                mode = "IDLE"
            if mode != "FOLLOW":
                print("Over Current Detected, Bumping Detected")
                rospy.set_param("/disinfect_status", "ROBOT_STRUCK")
                robot_safety_publisher.publish("ROBOT_STRUCK")
                rospy.set_param("/haystack/mode", "IDLE")
                PUBLISH_VELOCITY = False
                OVER_CURRENT_OCCURRENCE = 0
        OVER_CURRENT_OCCURRENCE += 1
    else:
        OVER_CURRENT_OCCURRENCE = 0
        PUBLISH_VELOCITY = True


def lidar_state_callback(data):
    Sensor_Status["lidar"] = True


def camera_state_callback(data):
    Sensor_Status["camera"] = True


def arduino_state_callback(data):
    System_Status["arduino"] = True


def battery_state_callback(data):
    Sensor_Status["battery"] = True


def sensor_data_timeout_timer():
    global sensor_data_timer, SENSOR_FAILURE_OCCURRENCE, Sensor_Status

    if sum(Sensor_Status.values()) != len(Sensor_Status):
        if SENSOR_FAILURE_OCCURRENCE < SENSOR_FAILURE_OCCURRENCE_LIMIT:
            SENSOR_FAILURE_OCCURRENCE += 1
        else:
            print("Sensor issue detected")
            rospy.set_param("/disinfect_status", "SENSOR_FAILURE")
            robot_safety_publisher.publish("SENSOR_FAILURE")
            rospy.set_param("/haystack/mode", "IDLE")
            SENSOR_FAILURE_OCCURRENC = 0

    for (sensor, data) in Sensor_Status.items():
        if not data:
            print(sensor)
            pass  # for finding the failed sensor

    Sensor_Status = dict.fromkeys(Sensor_Status, False)
    # restart the timer
    sensor_data_timer.cancel()
    sensor_data_timer = threading.Timer(SENSOR_MSG_TIMEOUT, sensor_data_timeout_timer)
    sensor_data_timer.start()


def velocity_callback(data):
    if PUBLISH_VELOCITY:
        safe_velocity_publisher.publish(data)


if __name__ == '__main__':
    print("Inside Robot Safety Node")
    rospy.init_node('Robot_Safety_Monitor', anonymous=True)
    robot_safety_publisher = rospy.Publisher('robot_safety', String, queue_size=10)
    # rospy.Subscriber("imu/data", Imu, imu_callback)
    rospy.Subscriber("motor_state", MotorState, motor_state_callback)
    rospy.Subscriber("robot_smooth_cmd_vel", Twist, velocity_callback)
    safe_velocity_publisher = rospy.Publisher("robot_safe_cmd_vel", Twist, queue_size=10)
    # rospy.Subscriber("scan", LaserScan, lidar_state_callback)
    # rospy.Subscriber("camera/color/image_raw", Image, camera_state_callback)
    # rospy.Subscriber("sonar_front_left", Range, arduino_state_callback)
    # rospy.Subscriber("haystack/battery/status", Bool, battery_state_callback)
    # sensor_data_timer = threading.Timer(SENSOR_MSG_INITIAL_TIMEOUT, sensor_data_timeout_timer)
    # sensor_data_timer.start()
    rospy.spin()  # simply keeps python from exiting until this node is stopped
    # sensor_data_timer.cancel()
