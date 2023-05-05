#!/usr/bin/python


import roslaunch
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import signal
import sqlite3
import subprocess
import threading

MAPPING_LAUNCH_PATH = "/haystack_ws/src/haystack/launch/mapping.launch"
FOLLOW_NAVIGATION_LAUNCH_PATH = "/haystack_ws/src/haystack/launch/navigator_follow.launch"
DISINFECT_NAVIGATION_LAUNCH_PATH = "/haystack_ws/src/haystack/launch/navigator_sanitize.launch"
DISINFECT_DYNAMIC_PATH = "/haystack_ws/src/haystack/launch/sanitization.launch"
DISINFECT_STATIC_PATH = "/haystack_ws/src/haystack/launch/sanitization_static.launch"
CONTROLLER_LAUNCH_PATH = "/haystack_ws/src/haystack/launch/haystack_controller.launch"
STATIC_DISINFECT_START = False
DYNAMIC_DISINFECT_START = False
STATIC_PERCENTAGE_STATUS = False
DISINFECTION_MODE = "DYNAMIC"
ROBOT_SAFETY_STATUS = None
DISINFECT_REPORT_STATUS = None


class ModeManager:
    RUNNING = True

    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.rate = rospy.Rate(10.0)
        self.status_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/robot_safety", String, self.robot_monitor_callback)
        self.velocity = Twist()
        self.velocity.angular.z = 0.523599
        self.mapping_uuid = None
        self.mapping_launch = None
        self.follow_navigation_uuid = None
        self.follow_navigator_launch = None
        self.disinfect_navigation_uuid = None
        self.disinfect_navigator_launch = None
        self.disinfect_uuid = None
        self.disinfect_launch = None
        self.joystick_uuid = None
        self.joystick_launch = None
        self.rotation = None

    def signal_handler(self, *args):
        print("exiting", args)
        self.RUNNING = False

    def robot_monitor_callback(self, status):
        global ROBOT_SAFETY_STATUS
        print("Robot is in Unsafe situation!!!!")
        ROBOT_SAFETY_STATUS = status.data

    def percentage_update(self, timer):

        global STATIC_PERCENTAGE_STATUS
        print("Inside percentage update")
        percentage = 0
        print(timer)
        while STATIC_PERCENTAGE_STATUS:
            if percentage < timer:
                rospy.set_param("/coverage/percentage", int(percentage * 100 / timer))
                if DISINFECTION_MODE == "STATIC_ROTATION":
                    rospy.set_param("/haystack/joystick/state", "PRESSED")
            else:
                STATIC_PERCENTAGE_STATUS = False
                rospy.set_param("/haystack/mode", "IDLE")
            percentage += 1
            time.sleep(1)
        rospy.set_param("/haystack/joystick/state", "RELEASED")

    def dynamic_disinfect_flag_set(self):
        global DYNAMIC_DISINFECT_START
        DYNAMIC_DISINFECT_START = True

    def static_disinfect_flag_set(self):
        global STATIC_DISINFECT_START
        STATIC_DISINFECT_START = True

    def send_rotation(self, rotation_speed):
        rospy.set_param("/haystack/joystick/current_linear_velocity", '0.0')
        rospy.set_param("/haystack/joystick/current_angular_velocity", rotation_speed)
        time.sleep(0.1)
        rospy.set_param("/haystack/joystick/state", "PRESSED")

    def mapping_start(self):
        self.mapping_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.mapping_uuid)
        self.mapping_launch = roslaunch.parent.ROSLaunchParent(self.mapping_uuid, [MAPPING_LAUNCH_PATH])
        self.mapping_launch.start()

    def follow_mode_start(self):
        # self.mapping_start()
        self.follow_navigation_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.follow_navigation_uuid)
        self.follow_navigator_launch = roslaunch.parent.ROSLaunchParent(self.follow_navigation_uuid,
                                                                        [FOLLOW_NAVIGATION_LAUNCH_PATH])
        #self.follow_navigator_launch.start()

        # rospy.set_param("/person_3d_locator/person_3d_locate_disable", False)
        rospy.set_param("/person_follower/person_follower_set_enable", True)

    def dynamic_disinfect_start(self):
        rospy.set_param("/exploration_bridge/status", "INITIALIZING")
        rospy.set_param("/coverge/percentage", 0)
        # rospy.set_param("/coverage/state", "INITIALIZING")
        rospy.set_param("/coverage/image_name", "0")
        # rospy.set_param("/disinfect_room_number", "0")
        # self.mapping_start()
        self.disinfect_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.disinfect_uuid)
        self.disinfect_launch = roslaunch.parent.ROSLaunchParent(self.disinfect_uuid, [DISINFECT_DYNAMIC_PATH])
        self.disinfect_launch.start()

    def static_disinfect_start(self):
        rospy.set_param("/exploration_bridge/status", "INITIALIZING")
        rospy.set_param("/coverge/percentage", 0)
        self.disinfect_static_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.disinfect_static_uuid)
        self.disinfect_static_launch = roslaunch.parent.ROSLaunchParent(self.disinfect_static_uuid,
                                                                        [DISINFECT_STATIC_PATH])
        self.disinfect_static_launch.start()

    def disinfect_navigator_start(self):
        self.disinfect_navigation_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.disinfect_navigation_uuid)
        self.disinfect_navigator_launch = roslaunch.parent.ROSLaunchParent(self.disinfect_navigation_uuid,
                                                                           [DISINFECT_NAVIGATION_LAUNCH_PATH])
        self.disinfect_navigator_launch.start()

    def follow_mode_stop(self):
        try:
            # self.follow_navigator_launch.shutdown()
            # self.mapping_launch.shutdown()
            pass
        except:
            pass
        # rospy.set_param("/person_3d_locator/person_3d_locate_disable", True)
        rospy.set_param("/person_follower/person_follower_set_enable", False)

    def disinfect_mode_stop(self):
        global ROBOT_SAFETY_STATUS, DISINFECT_REPORT_STATUS
        try:
            print("Disinfection Mode = ", DISINFECTION_MODE)
            if DISINFECTION_MODE == "DYNAMIC":
                self.disinfect_navigator_launch.shutdown()
                self.disinfect_launch.shutdown()
            else:
                self.disinfect_static_launch.shutdown()
            # self.mapping_launch.shutdown()
        except Exception as e:
            print("Ros Error : ", e)
        rospy.set_param("/exploration_bridge/status", "STOPPED")
        rospy.set_param("/coverage/percentage", 0)
        rospy.set_param("/disinfect_type", 0)
        try:
            report_name = rospy.get_param("/coverage/image_name")
            room_no = rospy.get_param("/disinfect_room_number")
            coverage_state = rospy.get_param("/coverage/state")
        except Exception as e:
            print("Ros Error :", e)
            report_name = ""
            room_no = ""
            coverage_state = "COVERAGE_DONE"
        if DISINFECTION_MODE == "DYNAMIC":
            if len(report_name) > 1:
                print(report_name)
                report_value = report_name.split("_")
                database = sqlite3.connect('/haystack_disinfect_report/database/disinfect_status_report.db')
                cursor = database.execute("SELECT id from HAYSTACK_DISINFECT_REPORT")
                cursor = [x for x in cursor]
                print(cursor)
                if not cursor:
                    serial_no = 1
                else:
                    serial_no = cursor[-1][0] + 1
                print("room no : ", room_no)
                print("serial_no : ", serial_no)

                if coverage_state == "USER_CTRL_C" and DISINFECT_REPORT_STATUS is None:
                    if ROBOT_SAFETY_STATUS is not None:
                        DISINFECT_REPORT_STATUS = ROBOT_SAFETY_STATUS
                        # rospy.set_param("/disinfect_status", DISINFECT_REPORT_STATUS)
                        ROBOT_SAFETY_STATUS = None
                    else:
                        DISINFECT_REPORT_STATUS = "CANCELLED"
                        rospy.set_param("/disinfect_status", DISINFECT_REPORT_STATUS)

                print(("INSERT INTO HAYSTACK_DISINFECT_REPORT (ID,YEAR,MONTH,DAY,HOUR,MIN,ROOM,"
                                 "PERCENTAGE,DURATION,IMAGE,DATE,STATUS) VALUES(" +
                                 str(serial_no)
                                 + ", " + str(int(report_value[0]))
                                 + ", " + str(int(report_value[1]))
                                 + ", " + str(int(report_value[2]))
                                 + ", " + str(int(report_value[3]))
                                 + ", " + str(int(report_value[4]))
                                 + ", " + str(int(room_no))
                                 + ", " + str(int(report_value[6]))
                                 + ", " + str(int(report_value[5]))
                                 + ", '" + str(report_name) + ".png'"
                                 + ", '" + str(int(report_value[0])) + "_" + str(int(report_value[1])) + "_" +
                                 str(int(report_value[2]))
                                 + "', '" + str(DISINFECT_REPORT_STATUS) + "')"))

                database.execute("INSERT INTO HAYSTACK_DISINFECT_REPORT (ID,YEAR,MONTH,DAY,HOUR,MIN,ROOM,"
                                 "PERCENTAGE,DURATION,IMAGE,DATE,STATUS) VALUES(" +
                                 str(serial_no)
                                 + ", " + str(int(report_value[0]))
                                 + ", " + str(int(report_value[1]))
                                 + ", " + str(int(report_value[2]))
                                 + ", " + str(int(report_value[3]))
                                 + ", " + str(int(report_value[4]))
                                 + ", " + str(int(room_no))
                                 + ", " + str(int(report_value[6]))
                                 + ", " + str(int(report_value[5]))
                                 + ", '" + str(report_name) + ".png'"
                                 + ", '" + str(int(report_value[0])) + "_" + str(int(report_value[1])) + "_" +
                                 str(int(report_value[2]))
                                 + "', '" + str(DISINFECT_REPORT_STATUS) + "')")
                database.commit()
                database.close()
            DISINFECT_REPORT_STATUS = None

    def stop_rotation(self):
        try:
            rospy.set_param("/haystack/joystick/state", "RELEASED")
        except:
            pass

    def temperory_fix(self):
        try:
            rospy.set_param("/exploration_bridge/status", "RUNNING")
        except:
            pass


if __name__ == '__main__':
    rospy.init_node('Mode_Changer', anonymous=True)
    try:
        modeManagerObj = ModeManager()
        rospy.set_param("/haystack/mode", "IDLE")
        rospy.set_param("/haystack/battery_percentage", 100)
        rospy.set_param("/haystack/battery_status", "DISCHARGING")
        pre_mode = "IDLE"
        start_rotation = None
        stop_rotation = None
        print("MODE SWITCHER STARTED")

        report_database = sqlite3.connect('/haystack_disinfect_report/database/disinfect_status_report.db')

        print("Opened database successfully")
        try:

            report_database.execute('''CREATE TABLE HAYSTACK_DISINFECT_REPORT
                     (ID INT PRIMARY KEY     NOT NULL,
                     YEAR           INT    NOT NULL,
                     MONTH          INT     NOT NULL,
                     DAY          INT     NOT NULL,
                     HOUR          INT     NOT NULL,
                     MIN          INT     NOT NULL,
                     ROOM          INT     NOT NULL,
                     PERCENTAGE          INT     NOT NULL,
                     DURATION          INT     NOT NULL,
                     IMAGE        TEXT,
                     DATE        TEXT,
                     STATUS       TEXT);''')
            print("Table created successfully")
        except sqlite3.OperationalError:
            pass
        report_database.close()

        while modeManagerObj.RUNNING:
            try:
                mode = rospy.get_param("/haystack/mode")
                coverage_state = rospy.get_param("/coverage/state")
            except Exception as e:
                # print("Ros Error :", e)
                pass
            try:
                if mode == "DISINFECT":
                    if coverage_state == "PERSON_DETECTED":
                        DISINFECT_REPORT_STATUS = coverage_state
                        rospy.set_param("/disinfect_status", DISINFECT_REPORT_STATUS)
                        rospy.set_param("/coverage/state", "STOPPED")
                        rospy.set_param("/exploration_bridge/status", "STOPPED")
                    elif coverage_state == "INITIALIZATION_ERROR":
                        DISINFECT_REPORT_STATUS = coverage_state
                        rospy.set_param("/disinfect_status", DISINFECT_REPORT_STATUS)
                        rospy.set_param("/coverage/state", "STOPPED")
                        rospy.set_param("/haystack/mode", "IDLE")
                    elif coverage_state == "COVERAGE_DONE":
                        DISINFECT_REPORT_STATUS = coverage_state
                        rospy.set_param("/disinfect_status", DISINFECT_REPORT_STATUS)
                        rospy.set_param("/coverage/state", "STOPPED")
                        rospy.set_param("/haystack/mode", "IDLE")
                    
            except:
                pass

            if mode == "IDLE" and pre_mode != mode:
                if pre_mode == "FOLLOW":
                    print("EXITED FOLLOW MODE")
                    modeManagerObj.follow_mode_stop()
                if pre_mode == "DISINFECT":
                    print("EXITED DISINFECT MODE")
                    modeManagerObj.disinfect_mode_stop()
                    modeManagerObj.stop_rotation()
                    if DISINFECTION_MODE != "STATIC_STANDBY":
                        if start_rotation is not None:
                            if start_rotation.is_alive():
                                start_rotation.cancel()
                    if DISINFECTION_MODE == "DYNAMIC":
                        if stop_rotation is not None:
                            if stop_rotation.is_alive():
                                stop_rotation.cancel()
                    if disinfect_start_timer.is_alive():
                        disinfect_start_timer.cancel()
                    STATIC_PERCENTAGE_STATUS = False
                    if DISINFECTION_MODE == "STATIC_ROTATION" or DISINFECTION_MODE == "STATIC_STANDBY":
                        if percentage.is_alive():
                            percentage.cancel()
                if pre_mode == "MANUAL":
                    print("EXITED MANUAL MODE")
                    modeManagerObj.stop_rotation()
                pre_mode = "IDLE"
                print("ENTERED IDLE MODE")

            if mode == "FOLLOW" and pre_mode != mode:
                if pre_mode == "IDLE":
                    print("EXITED IDLE MODE")
                    pass
                if pre_mode == "DISINFECT":
                    modeManagerObj.disinfect_mode_stop()
                    modeManagerObj.stop_rotation()
                    if DISINFECTION_MODE != "STATIC_STANDBY":
                        if start_rotation is not None:
                            if start_rotation.is_alive():
                                start_rotation.cancel()
                    if DISINFECTION_MODE == "DYNAMIC":
                        if stop_rotation is not None:
                            if stop_rotation.is_alive():
                                stop_rotation.cancel()
                    if disinfect_start_timer.is_alive():
                        disinfect_start_timer.cancel()
                    STATIC_PERCENTAGE_STATUS = False
                    if DISINFECTION_MODE == "STATIC_ROTATION" or DISINFECTION_MODE == "STATIC_STANDBY":
                        if percentage.is_alive():
                            percentage.cancel()
                    print("EXITED DISINFECT MODE")
                if pre_mode == "MANUAL":
                    print("EXITED MANUAL MODE")
                modeManagerObj.follow_mode_start()
                pre_mode = "FOLLOW"
                print("ENTERED FOLLOW MODE")

            if mode == "DISINFECT" and pre_mode != mode:
                if pre_mode == "IDLE":
                    print("EXITED IDLE MODE")
                    pass
                if pre_mode == "FOLLOW":
                    modeManagerObj.follow_mode_stop()
                    print("EXITED FOLLOW MODE")
                if pre_mode == "MANUAL":
                    print("EXITED MANUAL MODE")
                if rospy.has_param('/disinfect_type'):
                    try:
                        disinfect_type = rospy.get_param('/disinfect_type')
                    except Exception as e:
                        print("Ros error: ", e)
                else:
                    disinfect_type = 0
                print("Disinfect Type = ", disinfect_type)
                if disinfect_type < 15:
                    DISINFECTION_MODE = "DYNAMIC"
                    # start_rotation = threading.Timer(30, modeManagerObj.send_rotation, ['0.65'])
                    # stop_rotation = threading.Timer(40, modeManagerObj.stop_rotation)
                    temporory_fix = threading.Timer(40,
                                                    modeManagerObj.temperory_fix)  # work around to run teh new algo with UI
                    disinfect_start_timer = threading.Timer(1, modeManagerObj.dynamic_disinfect_flag_set)
                    # start_rotation.start()
                    # stop_rotation.start()
                    disinfect_start_timer.start()
                    temporory_fix.start()
                    modeManagerObj.disinfect_navigator_start()

                elif disinfect_type < 31:
                    DISINFECTION_MODE = "STATIC_ROTATION"
                    STATIC_PERCENTAGE_STATUS = True
                    start_rotation = threading.Timer(30, modeManagerObj.send_rotation, ['0.40'])
                    if disinfect_type == 16:
                        # stop_rotation = threading.Timer(160, modeManagerObj.stop_rotation)
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [120])
                    elif disinfect_type == 17:
                        # stop_rotation = threading.Timer(340, modeManagerObj.stop_rotation)
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [300])
                    elif disinfect_type == 18:
                        # stop_rotation = threading.Timer(640, modeManagerObj.stop_rotation)
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [600])
                    elif disinfect_type == 19:
                        # stop_rotation = threading.Timer(940, modeManagerObj.stop_rotation)
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [900])
                    disinfect_start_timer = threading.Timer(28, modeManagerObj.static_disinfect_flag_set)
                    start_rotation.start()
                    # stop_rotation.start()
                    percentage.start()
                    disinfect_start_timer.start()
                elif disinfect_type < 47:
                    DISINFECTION_MODE = "STATIC_STANDBY"
                    STATIC_PERCENTAGE_STATUS = True
                    if disinfect_type == 32:
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [120])
                    elif disinfect_type == 33:
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [300])
                    elif disinfect_type == 34:
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [600])
                    elif disinfect_type == 35:
                        percentage = threading.Timer(40, modeManagerObj.percentage_update, [900])
                    disinfect_start_timer = threading.Timer(28, modeManagerObj.static_disinfect_flag_set)
                    percentage.start()
                    disinfect_start_timer.start()

                pre_mode = "DISINFECT"
                print("ENTERED DISINFECT MODE")

            if DYNAMIC_DISINFECT_START:
                modeManagerObj.dynamic_disinfect_start()
                DYNAMIC_DISINFECT_START = False

            if STATIC_DISINFECT_START:
                modeManagerObj.static_disinfect_start()
                STATIC_DISINFECT_START = False

            if mode == "MANUAL" and pre_mode != mode:
                if pre_mode == "FOLLOW":
                    modeManagerObj.follow_mode_stop()
                    print("EXITED FOLLOW MODE")
                if pre_mode == "DISINFECT":
                    modeManagerObj.disinfect_mode_stop()
                    modeManagerObj.stop_rotation()
                    if DISINFECTION_MODE != "STATIC_STANDBY":
                        if start_rotation is not None:
                            if start_rotation.is_alive():
                                start_rotation.cancel()
                    if DISINFECTION_MODE == "DYNAMIC":
                        if stop_rotation is not None:
                            if stop_rotation.is_alive():
                                stop_rotation.cancel()
                    if disinfect_start_timer.is_alive():
                        disinfect_start_timer.cancel()
                    STATIC_PERCENTAGE_STATUS = False
                    if DISINFECTION_MODE == "STATIC_ROTATION" or DISINFECTION_MODE == "STATIC_STANDBY":

                        if percentage.is_alive():
                            percentage.cancel()
                    print("EXITED DISINFECT MODE")
                if pre_mode == "IDLE":
                    print("EXITED IDLE MODE")
                    pass
                pre_mode = "MANUAL"
                print("ENTERED MANUAL MODE")
            time.sleep(0.5)
    finally:
        print("MODE SWITCHER ENDED")
        pass

