#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Range
import math
SONAR_FRONT_LEFT = "sonar_front_left"
SONAR_FRONT_MID = "sonar_front_mid"
SONAR_FRONT_RIGHT = "sonar_front_right"
SONAR_BACK_RIGHT = "sonar_back_right"
SONAR_BACK_MID = "sonar_back_mid"
SONAR_BACK_LEFT = "sonar_back_left"
CLIFF_REAR_LEFT = "cliff_rear_left"
CLIFF_FRONT_LEFT = "cliff_front_left"
CLIFF_FRONT_RIGHT = "cliff_front_right"
CLIFF_REAR_RIGHT = "cliff_rear_right"

SONAR_FRONT_LEFT_MEAN_DEGREE = 45
SONAR_FRONT_MID_MEAN_DEGREE = 0
SONAR_FRONT_RIGHT_MEAN_DEGREE = 315
SONAR_BACK_RIGHT_MEAN_DEGREE = 225
SONAR_BACK_MID_MEAN_DEGREE = 180
SONAR_BACK_LEFT_MEAN_DEGREE = 135

CLIFF_REAR_RIGHT_MEAN_DEGREE = 293
CLIFF_FRONT_RIGHT_MEAN_DEGREE = 338
CLIFF_FRONT_LEFT_MEAN_DEGREE = 23
CLIFF_REAR_LEFT_MEAN_DEGREE = 68

DIAGONAL_OFFSET = 0.35
MID_OFFSET = 0.30
CLIFF_MAX_THRESHOLD = 0.35
CLIFF_MIN_RANGE = 0.2
CLIFF_MAX_RANGE = 1.5
SONAR_MAX_RANGE = 0.55

SONAR_DISTANCE_RATIO = 0.5778
MAX_SONAR_FIELD_DEGREE = 20
SONAR_ANGLE_REDUCTION = 10

class LaserTest:

    def __init__(self):
        print("Entered")
        rospy.init_node('laser_scan')
        self.rate = rospy.Rate(10.0)
        self.status_pub = rospy.Publisher('/scan_from_range', LaserScan, queue_size=10)

        self.laser = LaserScan()
        self.laser.header.frame_id = "base_footprint"
        self.laser.header.stamp = rospy.Time.now()
        self.laser.scan_time = 0.0
        self.laser.time_increment = 0.0
        self.laser.angle_max = 6.28319
        self.laser.angle_min = 0.0
        self.laser.range_min = 0.0
        self.laser.range_max = 3.50
        self.laser.angle_increment = 0.0174533
        self.laser.ranges = []
        for i in range(0, 360):
            self.laser.ranges.append(float('inf'))
        rospy.Subscriber(SONAR_FRONT_LEFT, Range, self.sonar_front_left)
        rospy.Subscriber(SONAR_FRONT_MID, Range, self.sonar_front_mid)
        rospy.Subscriber(SONAR_FRONT_RIGHT, Range, self.sonar_front_right)
        rospy.Subscriber(SONAR_BACK_RIGHT, Range, self.sonar_back_right)
        rospy.Subscriber(SONAR_BACK_MID, Range, self.sonar_back_mid)
        rospy.Subscriber(SONAR_BACK_LEFT, Range, self.sonar_back_left)
        #rospy.Subscriber(CLIFF_REAR_LEFT, Range, self.cliff_rear_left)
        #rospy.Subscriber(CLIFF_FRONT_LEFT, Range, self.cliff_front_left)
        #rospy.Subscriber(CLIFF_FRONT_RIGHT, Range, self.cliff_front_right)
        #rospy.Subscriber(CLIFF_REAR_RIGHT, Range, self.cliff_rear_right)


    def sonar_front_right(self, data):
        mean_angle = SONAR_FRONT_RIGHT_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + DIAGONAL_OFFSET * 100))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + DIAGONAL_OFFSET

    def sonar_front_mid(self, data):
        mean_angle = SONAR_FRONT_MID_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + MID_OFFSET * 100))
            #print(int(math.degrees(angle_offset)))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + MID_OFFSET

    def sonar_front_left(self, data):
        mean_angle = SONAR_FRONT_LEFT_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + DIAGONAL_OFFSET * 100))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + DIAGONAL_OFFSET

    def sonar_back_left(self, data):
        mean_angle = SONAR_BACK_LEFT_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + DIAGONAL_OFFSET * 100))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + DIAGONAL_OFFSET

    def sonar_back_mid(self, data):
        mean_angle = SONAR_BACK_MID_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + MID_OFFSET * 100))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + MID_OFFSET

    def sonar_back_right(self, data):
        mean_angle = SONAR_BACK_RIGHT_MEAN_DEGREE
        range_value = data.range
        for i in range(mean_angle - MAX_SONAR_FIELD_DEGREE, mean_angle + MAX_SONAR_FIELD_DEGREE):
            self.laser.ranges[i] = float('inf')
        if range_value < SONAR_MAX_RANGE:
            angle_offset = math.atan(
                (range_value * 100 * SONAR_DISTANCE_RATIO) / (range_value * 100 + DIAGONAL_OFFSET * 100))
            for i in range(mean_angle - int(math.degrees(angle_offset)) + SONAR_ANGLE_REDUCTION, mean_angle + int(math.degrees(angle_offset)) - SONAR_ANGLE_REDUCTION):
                self.laser.ranges[i] = range_value + DIAGONAL_OFFSET

    def cliff_rear_right(self, data):
        range_value = data.range
        if CLIFF_MIN_RANGE < range_value < CLIFF_MAX_THRESHOLD:
            self.laser.ranges[CLIFF_REAR_RIGHT_MEAN_DEGREE] = float('inf')
        else:
            self.laser.ranges[CLIFF_REAR_RIGHT_MEAN_DEGREE] = CLIFF_MIN_RANGE + MID_OFFSET


    def cliff_front_right(self, data):
        range_value = data.range
        if CLIFF_MIN_RANGE < range_value < CLIFF_MAX_THRESHOLD:
            self.laser.ranges[CLIFF_FRONT_RIGHT_MEAN_DEGREE] = float('inf')
        else:
            self.laser.ranges[CLIFF_FRONT_RIGHT_MEAN_DEGREE] = CLIFF_MIN_RANGE + MID_OFFSET

    def cliff_front_left(self, data):
        range_value = data.range
        if CLIFF_MIN_RANGE < range_value < CLIFF_MAX_THRESHOLD:
            self.laser.ranges[CLIFF_FRONT_LEFT_MEAN_DEGREE] = float('inf')
        else:
            self.laser.ranges[CLIFF_FRONT_LEFT_MEAN_DEGREE] = CLIFF_MIN_RANGE + MID_OFFSET


    def cliff_rear_left(self, data):
        range_value = data.range
        if CLIFF_MIN_RANGE < range_value < CLIFF_MAX_THRESHOLD:
            self.laser.ranges[CLIFF_REAR_LEFT_MEAN_DEGREE] = float('inf')
        else:
            self.laser.ranges[CLIFF_REAR_LEFT_MEAN_DEGREE] = CLIFF_MIN_RANGE + MID_OFFSET


if __name__ == "__main__":

    laserObj = LaserTest()
    pre_scan = []
    ranges = []
    for i in range(0, 360):
        ranges.append(float('inf'))

    try:
        while not rospy.is_shutdown():
            laserObj.laser.header.stamp = rospy.Time.now()
            if pre_scan != laserObj.laser.ranges:
                laserObj.status_pub.publish(laserObj.laser)
                pre_scan = laserObj.laser.ranges
            else:
                laserObj.laser.ranges = ranges
                laserObj.status_pub.publish(laserObj.laser)
            laserObj.rate.sleep()
    except rospy.ROSInterruptException:
        pass


