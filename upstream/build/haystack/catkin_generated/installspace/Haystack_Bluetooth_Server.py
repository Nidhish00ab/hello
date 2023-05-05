# !/usr/bin/python

from __future__ import absolute_import, print_function, unicode_literals
import math
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import time
import rospy
import struct
import sys
import threading
import sqlite3
from datetime import datetime, timedelta
import os

#
# try:
#     from gi.repository import GObject  # python3
# except ImportError:
#     import gobject as GObject  # python2

from gi.repository import GLib
import configparser

config = configparser.ConfigParser()
config.read("/haystack_disinfect_report/robot_config.ini")
# print(config["ROBOT"]["NAME"])
mainloop = None

BLUEZ_SERVICE_NAME = 'org.bluez'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'

LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE = 'org.bluez.GattDescriptor1'

RUNNING = True
ad_manager = None
haystack_advertisement = None

reports = []
send_image = None
bluetooth_connected_status = "DISCONNECTED"
wifi_name = None
wifi_password = None
time_zone = None

coverage_status = {"COVERAGE_DONE": 0, "PERSON_DETECTED": 1, "INITIALIZATION_ERROR": 2,
                   "CANCELLED": 3, "SENSOR_FAILURE": 4, "SYSTEM_FAILURE": 5, "ROBOT_STRUCK": 6, "None": 0}


class SendReport(threading.Thread):
    def __int__(self):
        super(self).__init__()

    def run(self):
        global reports
        print("Inside Send Report Thread")
        print(reports)
        for report in reports:
            print(report)
            id_byte = struct.unpack("BBBB", struct.pack("I", report[0]))
            year_byte = struct.unpack("BB", struct.pack("H", report[1]))
            room_byte = struct.unpack("BB", struct.pack("H", report[6]))
            crc_value = CRC16([3, id_byte[0], id_byte[1], id_byte[2],
                               id_byte[3], year_byte[0], year_byte[1],
                               report[2], report[3], report[4], report[5],
                               room_byte[0], room_byte[1], report[7], report[8], coverage_status[report[11]]])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.reportChObj.command_value = [dbus.Byte(3),
                                                                dbus.Byte(id_byte[0]), dbus.Byte(id_byte[1]),
                                                                dbus.Byte(id_byte[2]), dbus.Byte(id_byte[3]),
                                                                dbus.Byte(year_byte[0]), dbus.Byte(year_byte[1]),
                                                                dbus.Byte(report[2]), dbus.Byte(report[3]),
                                                                dbus.Byte(report[4]), dbus.Byte(report[5]),
                                                                dbus.Byte(room_byte[0]), dbus.Byte(room_byte[1]),
                                                                dbus.Byte(report[7]), dbus.Byte(report[8]),
                                                                dbus.Byte(coverage_status[report[11]]),
                                                                dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]

            app.haystackServiceObj.reportChObj.notify_command_change()
        reports = []


class SetWifi(threading.Thread):
    def __int__(self):
        super(self).__init__()

    def run(self):
        global wifi_password, wifi_name
        Wifi_Connected = False
        count = 0
        print("Connecting to the wifi")
        os.system("nmcli r wifi off")
        os.system("nmcli r wifi on")
        time.sleep(2)
        if wifi_name != None and wifi_password != None:
            while count < 30:
                try:
                    output = os.popen('nmcli d wifi connect ' + "'" + wifi_name + "'" +
                                      ' password ' + "'" + wifi_password + "'").read()

                    print(output)

                    if "successfully activated" in output:
                        print("Wifi Connected = ", output)
                        Wifi_Connected = True
                        temp = Wifi_Name = [ord(x) for x in wifi_name][::-1]
                        crc_value = CRC16([10, 1] + temp)
                        crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                        app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(10), dbus.Byte(1)] + \
                                                                            [dbus.Byte(x) for x in temp] + \
                                                                          [dbus.Byte(crc_bytes[0]),
                                                                          dbus.Byte(crc_bytes[1])]
                        print(app.haystackServiceObj.settingChObj.command_value)
                        app.haystackServiceObj.settingChObj.notify_command_change()
                        break
                except:
                    pass
                count += 1
                time.sleep(0.1)
            if not Wifi_Connected:
                crc_value = CRC16([10, 0])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(2), dbus.Byte(0),
                                                                  dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]

                app.haystackServiceObj.settingChObj.notify_command_change()
        else:
            crc_value = CRC16([10, 2])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(2), dbus.Byte(2),
                                                              dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]

            app.haystackServiceObj.settingChObj.notify_command_change()

        print("Wifi Setting Done")


class SetTime(threading.Thread):
    def __int__(self):
        super(self).__init__()

    def run(self):
        global time_zone
        print("Setting time zone to ", time_zone)
        # os.system("timedatectl set-timezone " + str(time_zone))
        os.system("cp /usr/share/zoneinfo/" + str(time_zone) +" /etc/localtime")


class SendImage(threading.Thread):

    def __int__(self):
        threading.Thread.__init__(self)
        self.data = None
        self.data_length = None
        self.remaining_bytes = None
        self.crc_remaining = None
        self.local_buffer = []
        self.crc_not_sent = True

    def run(self):
        self.local_buffer = []
        self.remaining_bytes = None
        with open("/haystack_disinfect_report/images/" + send_image, "rb") as image:
            self.data = image.read()
        self.data_length = len(self.data)
        self.data = [x for x in self.data]
        self.remaining_bytes = self.data_length
        length_byte = struct.unpack("BBBB", struct.pack("I", self.data_length))
        print("Total size = ", self.data_length)
        crc_value = CRC16(self.data)
        crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
        initial = 0

        starting_byte = 0
        self.crc_remaining = 2
        self.crc_not_sent = True
        while self.remaining_bytes or self.crc_remaining:
            print("Remaining Byte to send = ", self.remaining_bytes)
            if initial == 0:
                self.local_buffer.append(length_byte[0])
                self.local_buffer.append(length_byte[1])
                self.local_buffer.append(length_byte[2])
                self.local_buffer.append(length_byte[3])
                self.local_buffer = self.local_buffer + self.data[starting_byte:starting_byte + 16]
                starting_byte += 16
                self.remaining_bytes -= 16
                initial = 1

            elif self.remaining_bytes >= 20:
                self.local_buffer = []
                self.local_buffer = self.local_buffer + self.data[starting_byte:starting_byte + 20]
                starting_byte += 20
                self.remaining_bytes -= 20
            elif 0 < self.remaining_bytes < 19:
                self.local_buffer = []
                self.local_buffer = self.local_buffer + self.data[starting_byte:]
                self.local_buffer.append(crc_bytes[0])
                self.local_buffer.append(crc_bytes[1])
                self.remaining_bytes -= self.remaining_bytes
                self.crc_remaining -= 2
            elif self.remaining_bytes > 18:
                self.local_buffer = []
                self.local_buffer = self.local_buffer + self.data[starting_byte:starting_byte + 19]
                self.local_buffer.append(crc_bytes[0])
                self.remaining_bytes -= self.remaining_bytes
                self.crc_remaining -= 1
            elif self.crc_remaining == 2:
                self.local_buffer = []
                self.local_buffer.append(crc_bytes[0])
                self.local_buffer.append(crc_bytes[1])
                self.crc_remaining -= 2
            elif self.crc_remaining == 1:
                self.local_buffer = []
                self.local_buffer.append(crc_bytes[1])
                self.crc_remaining -= 1

            print(self.local_buffer)
            app.haystackServiceObj.sendImageChObj.command_value = [dbus.Byte(x) for x in self.local_buffer]

            app.haystackServiceObj.sendImageChObj.notify_command_change()

        print("File Transfer Completed")


def date_range(start, end):
    delta = end - start  # as timedelta
    days = [start + timedelta(days=i) for i in range(delta.days + 1)]
    # days = [[x.day, x.month] for x in days]
    return days


def CRC16(data):
    # LineFun = inspect.getframeinfo(inspect.currentframe())
    if data is None:
        return 0
    crc = 0x0000
    for i in data:
        crc ^= i << 8
        for j in range(0, 8):
            if (crc & 0x8000) > 0:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    # print(LineFun.function, LineFun.lineno, hex(crc & 0xFFFF))
    return crc & 0xFFFF


def report_manager(data):
    global send_image, reports
    print("Inside Report Manager")
    command_data = [int(value) for value in data]
    print(command_data)

    if command_data[0] == 1:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:9])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[9] == crc_bytes[0] and command_data[10] == crc_bytes[1]:
            print("CRC SUCCESS  in report query")
            report_database = sqlite3.connect('/haystack_disinfect_report/database/disinfect_status_report.db')
            # report_database = sqlite3.connect("test3.db")
            cursor = report_database.cursor()

            start_year = struct.unpack("H", struct.pack("BB", command_data[1], command_data[2]))[0]
            end_year = struct.unpack("H", struct.pack("BB", command_data[5], command_data[6]))[0]
            start_date = datetime(start_year, command_data[3], command_data[4])
            print(start_date)
            end_date = datetime(end_year, command_data[7], command_data[8])
            print(end_date)
            days = date_range(start_date, end_date)
            print(days)
            for x in days:
                date = str(x.year) + "_" + str(x.month) + "_" + str(x.day)
                print(date)
                cursor.execute("SELECT * from HAYSTACK_DISINFECT_REPORT WHERE DATE =  '" + date + "' ")
                reports = reports + cursor.fetchall()
            print(reports)
            report_database.close()
            report_length = len(reports)
            report_length_byte = struct.unpack("BBBB", struct.pack("I", report_length))
            crc_value = CRC16([2, report_length_byte[0], report_length_byte[1],
                               report_length_byte[2], report_length_byte[3]])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.reportChObj.command_value = [dbus.Byte(2), dbus.Byte(report_length_byte[0]),
                                                                dbus.Byte(report_length_byte[1]),
                                                                dbus.Byte(report_length_byte[2]),
                                                                dbus.Byte(report_length_byte[3]),
                                                                dbus.Byte(crc_bytes[0]),
                                                                dbus.Byte(crc_bytes[1])]

            app.haystackServiceObj.reportChObj.notify_command_change()
            if len(reports) != 0:
                sendReport = SendReport()
                sendReport.start()

        else:
            print("CRC ERROR in report query")

    if command_data[0] == 4:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:5])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[5] == crc_bytes[0] and command_data[6] == crc_bytes[1]:
            print("CRC SUCCESS  in report file request")
            report_database = sqlite3.connect('/haystack_disinfect_report/database/disinfect_status_report.db')
            # report_database = sqlite3.connect('test3.db')
            cursor = report_database.cursor()
            serial_id = struct.unpack("I", struct.pack("BBBB", command_data[1], command_data[2],
                                                       command_data[3], command_data[4]))
            print("Report ID = ", serial_id[0])
            cursor.execute("SELECT * from HAYSTACK_DISINFECT_REPORT WHERE ID =  " + str(serial_id[0]) + " ")
            report = cursor.fetchall()
            print(report)
            report_database.close()
            send_image = report[0][9]
            print("Image Name = ", send_image)
            sendImage = SendImage()
            sendImage.start()
        else:
            print("CRC ERROR  in report file request")


def setting_config(data):
    global wifi_name, wifi_password, time_zone
    print("Inside Wifi Setting")
    # command_data = struct.unpack("BBBBBBBH", data)
    print(data)
    command_data = [int(value) for value in data]
    print(command_data)

    if command_data[0] == 9:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS  in wifi name query")
        output = os.popen('iwgetid').read()
        Wifi_Conn = output.split('"')[1]
        print(Wifi_Conn)
        if len(Wifi_Conn) > 0:
            #Wifi_Name = [bytes(x, 'utf-8') for x in Wifi_Conn]
            Wifi_Name = [ord(x) for x in Wifi_Conn][::-1]
            print(Wifi_Name)
            crc_value = CRC16([10, 1] + Wifi_Name)
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(10), dbus.Byte(1)] + \
                                                             [dbus.Byte(x) for x in Wifi_Name] + \
                                                             [dbus.Byte(crc_bytes[0]),
                                                              dbus.Byte(crc_bytes[1])]
            print(app.haystackServiceObj.settingChObj.command_value)
            app.haystackServiceObj.settingChObj.notify_command_change()
        else:
            crc_value = CRC16([10, 0])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(3), dbus.Byte(0),
                                                              dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]

            app.haystackServiceObj.settingChObj.notify_command_change()

    if command_data[0] == 0:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS  in wifi name")
            wifi_name = None
            wifi_name = [str(value) for value in data[1:-2]]
            wifi_name = "".join(wifi_name)
            print(wifi_name)

    elif command_data[0] == 1:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS  in wifi password")
            wifi_password = None
            wifi_password = [str(value) for value in data[1:-2]]
            wifi_password = "".join(wifi_password)
            print(wifi_password)
            setWifi = SetWifi()
            setWifi.start()

    elif command_data[0] == 3:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS in time query")
            time_zone = None
            time_zone = [str(value) for value in data[1:-2]]
            time_zone = "".join(time_zone)
            print(time_zone)
            setTime = SetTime()
            setTime.start()

    elif command_data[0] == 5:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS in robot name query")
            robot_name = None
            robot_name = [str(value) for value in data[1:-2]]
            robot_name = "".join(robot_name)
            print(robot_name)
            config['ROBOT'] = {'NAME': robot_name}
            haystack_advertisement.add_local_name(robot_name)
            crc_value = CRC16([7, 1])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(7), dbus.Byte(1),
                                                              dbus.Byte(crc_bytes[0]),
                                                              dbus.Byte(crc_bytes[1])]

            app.haystackServiceObj.settingChObj.notify_command_change()

    elif command_data[0] == 7:
        crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:-2])))
        print(crc_bytes[0], crc_bytes[1])
        if command_data[-2] == crc_bytes[0] and command_data[-1] == crc_bytes[1]:
            print("CRC SUCCESS in robot firmware query")
            robot_firmware = os.getenv("ROBOT_VERSION", default=None)
            if not robot_firmware:
                robot_firmware = "0.0.0"
            print("robot_firmware = ", robot_firmware)
            report_length_byte = [int(i) for i in robot_firmware.split(".")]
            crc_value = CRC16([8, report_length_byte[0], report_length_byte[1], report_length_byte[2]])
            crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
            app.haystackServiceObj.settingChObj.command_value = [dbus.Byte(8), dbus.Byte(report_length_byte[0]),
                                                              dbus.Byte(report_length_byte[1]),
                                                              dbus.Byte(report_length_byte[2]),
                                                              dbus.Byte(crc_bytes[0]),
                                                              dbus.Byte(crc_bytes[1])]
            print(app.haystackServiceObj.settingChObj.command_value)
            app.haystackServiceObj.settingChObj.notify_command_change()


def mode_change(data):
    print("Inside Mode Changer")
    command_data = [int(value) for value in data]
    print(command_data)
    crc_bytes = struct.unpack("BB", struct.pack("H", CRC16(command_data[0:6])))
    print(crc_bytes[0], crc_bytes[1])
    if command_data[6] == crc_bytes[0] and command_data[7] == crc_bytes[1]:
        print("CRC SUCCESS")
        if command_data[0] == 0:
            rospy.set_param("/haystack/mode", "IDLE")
        elif command_data[0] == 1:
            rospy.set_param("/haystack/mode", "FOLLOW")
        elif command_data[0] == 2:
            disinfect_type = command_data[3]
            room_no = struct.unpack("H", struct.pack("BB", command_data[4], command_data[5]))
            print("Room No = ", room_no)
            print("Disinfect Type = ", disinfect_type)
            rospy.set_param("/disinfect_room_number", room_no[0])
            rospy.set_param("/disinfect_type", disinfect_type)
            time.sleep(0.1)
            rospy.set_param("/haystack/mode", "DISINFECT")
        elif command_data[0] == 3:
            linear_velocity = command_data[1]
            angular_velocity = command_data[2]
            if linear_velocity > 128:
                linear_velocity -= 256

            if angular_velocity > 128:
                angular_velocity -= 256
            linear_velocity = float(linear_velocity) / 100
            angular_velocity = -(float(math.radians(angular_velocity)))
            print("linear_velocity = ", linear_velocity)
            print("angular_velocity = ", angular_velocity)
            if -0.90 < linear_velocity < 0.90:
                print("linear_velocity = ", linear_velocity)
                if linear_velocity != 0:
                    rospy.set_param("/haystack/joystick/linear_velocity", abs(linear_velocity))
                rospy.set_param("/haystack/joystick/current_linear_velocity", linear_velocity)
            if -1.57 < angular_velocity < 1.57:
                print("angular_velocity = ", angular_velocity)
                if angular_velocity != 0:
                    rospy.set_param("/haystack/joystick/angular_velocity", abs(angular_velocity))
                rospy.set_param("/haystack/joystick/current_angular_velocity", angular_velocity)
            if command_data[3] == 0:
                rospy.set_param("/haystack/joystick/state", "RELEASED")
            elif command_data[3] == 1:
                rospy.set_param("/haystack/joystick/state", "PRESSED")
            rospy.set_param("/haystack/mode", "MANUAL")
        print("Completed")
    else:
        print("CRC ERROR")


class ModeMonitor(threading.Thread):
    def __init__(self):
        """

        :rtype: object
        """
        threading.Thread.__init__(self)
        rospy.set_param("/haystack/mode", "IDLE")
        self.current_mode = None
        self.current_battery_status = None
        self.current_battery_percentage = None
        self.current_command = None

    def run(self):

        while RUNNING:
            try:
                mode = rospy.get_param("/haystack/mode")
            except Exception as e:
                # print("Ros Error :", e)
                mode = "IDLE"
            try:
                coverage_state = rospy.get_param("/coverage/state")
            except Exception as e:
                # print("Ros Error :", e)
                coverage_state = "COVERAGE_DONE"

            try:
                battery_percentage = rospy.get_param("/haystack/battery_percentage")
                battery_status = rospy.get_param("/haystack/battery_status")
            except Exception as e:
                # print("Ros Error :", e)
                battery_percentage = 100
                battery_status = "DISCHARGING"

            if battery_status == "CHARGING":
                battery_status = 4
            else:
                battery_status = 2
            if mode == "IDLE":
                try:
                    disinfect_status = rospy.get_param("/disinfect_status")
                except Exception as e:
                    disinfect_status = "COVERAGE_DONE"
                crc_value = CRC16([0, 0, 0, 0, 0, coverage_status[disinfect_status]])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.commandChObj.command_value = [dbus.Byte(0), dbus.Byte(0), dbus.Byte(0),
                                                                     dbus.Byte(0), dbus.Byte(0),
                                                                     dbus.Byte(coverage_status[disinfect_status]),
                                                                     dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]

            if mode == "FOLLOW":
                crc_value = CRC16([1, 0, 0, 0, 0, 0])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.commandChObj.command_value = [dbus.Byte(1), dbus.Byte(0), dbus.Byte(0),
                                                                     dbus.Byte(0), dbus.Byte(0),
                                                                     dbus.Byte(0), dbus.Byte(crc_bytes[0]),
                                                                     dbus.Byte(crc_bytes[1])]

            if mode == "DISINFECT":
                try:
                    coverage_percentage = rospy.get_param("/coverage/percentage")
                except Exception as e:
                    # print("Ros Error :", e)
                    coverage_percentage = 0
                try:
                    disinfect_type = rospy.get_param("/disinfect_type")
                except Exception as e:
                    # print("Ros Error :", e)
                    disinfect_type = 0

                crc_value = CRC16([2, 0, int(coverage_percentage), int(disinfect_type), 0, 0])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.commandChObj.command_value = [dbus.Byte(2), dbus.Byte(0),
                                                                     dbus.Byte(int(coverage_percentage)),
                                                                     dbus.Byte(int(disinfect_type)), dbus.Byte(0),
                                                                     dbus.Byte(0), dbus.Byte(crc_bytes[0]),
                                                                     dbus.Byte(crc_bytes[1])]

            if mode == "MANUAL":
                try:
                    current_linear_velocity = rospy.get_param("/haystack/joystick/current_linear_velocity")
                    current_angular_velocity = rospy.get_param("/haystack/joystick/current_angular_velocity")
                    linear_velocity = rospy.get_param("/haystack/joystick/linear_velocity")
                    angular_velocity = rospy.get_param("/haystack/joystick/angular_velocity")
                    state = rospy.get_param("/haystack/joystick/state")
                except Exception as e:
                    print("Ros Error :", e)

                #if state == "PRESSED":
                #    state = 1
                #    current_linear_velocity = int(float(current_linear_velocity) * 100)
                #    if current_linear_velocity < 0:
                #        current_linear_velocity = 256 + current_linear_velocity
                #    current_angular_velocity = int(round(math.degrees(float(current_angular_velocity))))
                #    if current_angular_velocity < 0:
                #        current_angular_velocity = 256 + current_angular_velocity
                #    crc_value = CRC16([3, current_linear_velocity, current_angular_velocity, state, 0, 0])
                #    crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                #    app.haystackServiceObj.commandChObj.command_value = [dbus.Byte(3),
                #                                                         dbus.Byte(current_linear_velocity),
                #                                                         dbus.Byte(current_angular_velocity),
                #                                                         dbus.Byte(state), dbus.Byte(0), dbus.Byte(0),
                #                                                         dbus.Byte(crc_bytes[0]),
                #                                                         dbus.Byte(crc_bytes[1])]
                #else:
                state = 0
                linear_velocity = int(float(linear_velocity) * 100)
                if linear_velocity < 0:
                    linear_velocity = 256 + linear_velocity
                angular_velocity = int(round(math.degrees(float(angular_velocity))))
                if angular_velocity < 0:
                    angular_velocity = 256 + angular_velocity
                crc_value = CRC16([3, linear_velocity, angular_velocity, state, 0, 0])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.commandChObj.command_value = [dbus.Byte(3),
                                                                         dbus.Byte(linear_velocity),
                                                                         dbus.Byte(angular_velocity),
                                                                         dbus.Byte(state), dbus.Byte(0), dbus.Byte(0),
                                                                         dbus.Byte(crc_bytes[0]),
                                                                         dbus.Byte(crc_bytes[1])]

            if self.current_battery_percentage != battery_percentage or self.current_battery_status != battery_status:
                self.current_battery_status = battery_status
                self.current_battery_percentage = battery_percentage
                print(battery_percentage)
                crc_value = CRC16([battery_percentage, battery_status])
                crc_bytes = struct.unpack("BB", struct.pack("H", crc_value))
                app.haystackServiceObj.batteryChObj.command_value = [dbus.Byte(battery_percentage),
                                                                     dbus.Byte(battery_status),
                                                                     dbus.Byte(crc_bytes[0]), dbus.Byte(crc_bytes[1])]
                app.haystackServiceObj.batteryChObj.notify_command_change()

            if self.current_mode != mode or self.current_command != app.haystackServiceObj.commandChObj.command_value:
                self.current_mode = mode
                app.haystackServiceObj.commandChObj.notify_command_change()

            self.current_command = app.haystackServiceObj.commandChObj.command_value

            time.sleep(0.5)


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'


class NotPermittedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotPermitted'


class InvalidValueLengthException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.InvalidValueLength'


class FailedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.Failed'


class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = advertising_type
        self.service_uuids = None
        self.manufacturer_data = None
        self.solicit_uuids = None
        self.service_data = None
        self.local_name = None
        self.include_tx_power = False
        self.data = None
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        properties = dict()
        properties['Type'] = self.ad_type
        if self.service_uuids is not None:
            properties['ServiceUUIDs'] = dbus.Array(self.service_uuids,
                                                    signature='s')
        if self.solicit_uuids is not None:
            properties['SolicitUUIDs'] = dbus.Array(self.solicit_uuids,
                                                    signature='s')
        if self.manufacturer_data is not None:
            properties['ManufacturerData'] = dbus.Dictionary(
                self.manufacturer_data, signature='qv')
        if self.service_data is not None:
            properties['ServiceData'] = dbus.Dictionary(self.service_data,
                                                        signature='sv')
        if self.local_name is not None:
            properties['LocalName'] = dbus.String(self.local_name)
        if self.include_tx_power:
            properties['Includes'] = dbus.Array(["tx-power"], signature='s')

        if self.data is not None:
            properties['Data'] = dbus.Dictionary(
                self.data, signature='yv')
        return {LE_ADVERTISEMENT_IFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service_uuid(self, uuid):
        if not self.service_uuids:
            self.service_uuids = []
        self.service_uuids.append(uuid)

    def add_solicit_uuid(self, uuid):
        if not self.solicit_uuids:
            self.solicit_uuids = []
        self.solicit_uuids.append(uuid)

    def add_manufacturer_data(self, manuf_code, data):
        if not self.manufacturer_data:
            self.manufacturer_data = dbus.Dictionary({}, signature='qv')
        self.manufacturer_data[manuf_code] = dbus.Array(data, signature='y')

    def add_service_data(self, uuid, data):
        if not self.service_data:
            self.service_data = dbus.Dictionary({}, signature='sv')
        self.service_data[uuid] = dbus.Array(data, signature='y')

    def add_local_name(self, name):
        if not self.local_name:
            self.local_name = ""
        self.local_name = dbus.String(name)

    def add_data(self, ad_type, data):
        if not self.data:
            self.data = dbus.Dictionary({}, signature='yv')
        self.data[ad_type] = dbus.Array(data, signature='y')

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        print('GetAll')
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        print('returning props')
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE,
                         in_signature='',
                         out_signature='')
    def Release(self):
        print('%s: Released!' % self.path)


class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """

    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        self.haystackServiceObj = HaystackService(bus, 0)
        self.add_service(self.haystackServiceObj)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        print('GetManagedObjects')

        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response


class Service(dbus.service.Object):
    """
    org.bluez.GattService1 interface implementation
    """
    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.local_name = None
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Characteristics': dbus.Array(
                    self.get_characteristic_paths(),
                    signature='o'),
                'LocalName': dbus.String(self.local_name)
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_SERVICE_IFACE]

    def add_local_name(self, name):
        if not self.local_name:
            self.local_name = ""
        self.local_name = dbus.String(name)


class Characteristic(dbus.service.Object):
    """
    org.bluez.GattCharacteristic1 interface implementation
    """

    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.local_name = None
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'Descriptors': dbus.Array(
                    self.get_descriptor_paths(),
                    signature='o'),
                'LocalName': dbus.String(self.local_name)
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def add_local_name(self, name):
        if not self.local_name:
            self.local_name = ""
        self.local_name = dbus.String(name)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE,
                         in_signature='a{sv}',
                         out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        print('Default StartNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        print('Default StopNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE,
                         signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Descriptor(dbus.service.Object):
    """
    org.bluez.GattDescriptor1 interface implementation
    """

    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_DESC_IFACE: {
                'Characteristic': self.chrc.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE,
                         in_signature='a{sv}',
                         out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise NotSupportedException()


class HaystackService(Service):
    """
    Haystack service that provides characteristics and descriptors that
    exercise various API functionality.

    """
    HAYSTACK_SVC_UUID = 'c1d2f341-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index):
        Service.__init__(self, bus, index, self.HAYSTACK_SVC_UUID, True)
        self.batteryChObj = BatteryCharacteristic(bus, 0, self)
        self.commandChObj = CommandCharacteristic(bus, 1, self)
        self.diagnosticChObj = DiagnosticsCharacteristic(bus, 2, self)
        self.reportChObj = ReportCharacteristic(bus, 3, self)
        self.sendImageChObj = SendImageCharacteristic(bus, 4, self)
        self.settingChObj = SettingCharacteristic(bus, 5, self)
        self.add_characteristic(self.batteryChObj)
        self.add_characteristic(self.commandChObj)
        self.add_characteristic(self.diagnosticChObj)
        self.add_characteristic(self.reportChObj)
        self.add_characteristic(self.sendImageChObj)
        self.add_characteristic(self.settingChObj)


class BatteryCharacteristic(Characteristic):
    """
    Battery characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    BATTERY_CHRC_UUID = 'c1d2f342-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.BATTERY_CHRC_UUID,
            ['read', 'notify'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x46), dbus.Byte(0x02), dbus.Byte(0x28), dbus.Byte(0x87)]

    def notify_command_change(self):
        if not self.notifying:
            return
        print("Battery Notified", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying battery, nothing to do')
            return
        print("Command Notify Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying battery, nothing to do')
            return
        print("Battery Notify Disabled")
        self.notifying = False

    def ReadValue(self, options):
        print('BatteryCharacteristic Read: ' + repr(self.command_value))
        return self.command_value


class CommandCharacteristic(Characteristic):
    """
    Command characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    COMMAND_CHRC_UUID = 'c1d2f343-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.COMMAND_CHRC_UUID,
            ['write-without-response', 'notify', 'read'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x00)]
        # GObject.timeout_add(5000, self.drain_battery)

    def notify_command_change(self):
        if not self.notifying:
            return
        print("Command Notified ", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying command, nothing to do')
            return
        print("Command Notify Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying command, nothing to do')
            return
        print("Command Notify Disabled")
        self.notifying = False

    def ReadValue(self, options):
        print('CommandCharacteristic Read: ' + repr(self.command_value))
        return self.command_value

    def WriteValue(self, value, options):
        self.command_value = value
        print("Command Write : ", value)
        mode_change(self.command_value)


class ReportCharacteristic(Characteristic):
    """
    Command characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    REPORT_CHRC_UUID = 'c1d2f344-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.REPORT_CHRC_UUID,
            ['write-without-response', 'notify', 'read'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x00)]

    def notify_command_change(self):
        if not self.notifying:
            return
        print("Report Notified", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying report, nothing to do')
            return
        print("Report Notify Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying report, nothing to do')
            return
        print("Report Notify Disabled")
        self.notifying = False

    def WriteValue(self, value, options):
        self.command_value = value
        print("Report write: ", self.command_value)
        report_manager(self.command_value)

    def ReadValue(self, options):
        print('ReportCharacteristic Read: ' + repr(self.command_value))
        return self.command_value


class DiagnosticsCharacteristic(Characteristic):
    """
    Command characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    DIAGNOSTICS_CHRC_UUID = 'c1d2f345-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.DIAGNOSTICS_CHRC_UUID,
            ['write-without-response', 'notify', 'read'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x00)]
        # GObject.timeout_add(5000, self.drain_battery)

    def notify_command_change(self):
        if not self.notifying:
            return
        print("Diagonistic notified", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying diagonistic, nothing to do')
            return
        print("Diagonistic nofity Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying diagonistic, nothing to do')
            return
        self.notifying = False
        print("Diagonistic nofity Disabled")

    def ReadValue(self, options):
        print('DiagnosticsCharacteristic Read: ' + repr(self.command_value))
        return self.command_value

    def WriteValue(self, value, options):
        self.command_value = value
        print("Diagonistic write: ", self.command_value)
        mode_change(self.command_value)


class SettingCharacteristic(Characteristic):
    """
    Command characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    WIFI_CHRC_UUID = 'c1d2f347-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.WIFI_CHRC_UUID,
            ['write-without-response', 'notify', 'read'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x00)]
        # GObject.timeout_add(5000, self.drain_battery)

    def notify_command_change(self):
        if not self.notifying:
            return
        print("Setting Notified", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying setting, nothing to do')
            return
        print("Setting Notify Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying setting, nothing to do')
            return
        print("Setting Notify Disabled")
        self.notifying = False

    def ReadValue(self, options):
        print('SettingCharacteristic Read: ' + repr(self.command_value))
        return self.command_value

    def WriteValue(self, value, options):
        self.command_value = value
        print("Setting write: ", self.command_value)
        setting_config(self.command_value)


class SendImageCharacteristic(Characteristic):
    """
    Command characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.

    """
    SEND_IMAGE_CHRC_UUID = 'c1d2f346-bfb3-4559-be8d-3d7472cff8c9'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.SEND_IMAGE_CHRC_UUID,
            ['notify', 'write-without-response', 'read'],
            service)
        self.notifying = True
        self.command_value = [dbus.Byte(0x00)]

    def notify_command_change(self):
        if not self.notifying:
            return
        print("SendImage Notified", self.command_value)
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.command_value}, [])

    def StartNotify(self):
        if self.notifying:
            print('Already notifying image, nothing to do')
            return
        print("Image Notify Enabled")
        self.notifying = True
        #self.notify_command_change()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying image, nothing to do')
            return
        print("Image Notify Disabled")
        self.notifying = False

    def WriteValue(self, value, options):
        self.command_value = value
        print("Image write: ", self.command_value)
        report_manager(self.command_value)

    def ReadValue(self, options):
        print('ImageCharacteristic Read: ' + repr(self.command_value))
        return self.command_value


class HaystackBluetooth(Advertisement):

    def __init__(self, bus, index):
        Advertisement.__init__(self, bus, index, 'peripheral')

        self.add_service_uuid('5EEA')
        self.add_manufacturer_data(0x5EAA, [0x00, 0x01, 0x02, 0x03])
        self.add_service_data('5EE1', [0x00, 0x01, 0x02, 0x03, 0x04])
        self.add_local_name(config["ROBOT"]["NAME"])
        self.include_tx_power = True
        self.add_data(0x26, [0x01, 0x01, 0x00])


def register_ad_cb():
    print('Advertisement registered')


def register_ad_error_cb(error):
    print('Failed to register advertisement: ' + str(error))
    mainloop.quit()


def register_app_cb():
    print('GATT application registered')


def register_app_error_cb(error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()


def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()

    for o, props in objects.items():
        if LE_ADVERTISING_MANAGER_IFACE in props:
            return o

    return None


def shutdown(timeout):
    print('Advertising for {} seconds...'.format(timeout))
    time.sleep(timeout)
    mainloop.quit()


def properties_changed(interface, changed, invalidated, path):
    global timer, idle_timer
    if interface != "org.bluez.Device1":
        return
    # print(changed)
    if dbus.String(u'ServicesResolved') in changed:
        if changed[dbus.String(u'ServicesResolved')]:
            print("Connected")
            try:
                rospy.set_param("/haystack/bluetooth_status", "CONNECTED")
                stop_advertisement()
            except Exception as e:
                print("ROS Error : ", e)
        else:
            print("Disconnected")
            try:
                rospy.set_param("/haystack/bluetooth_status", "DISCONNECTED")
                rospy.set_param("/haystack/joystick/state", "RELEASED")
            except Exception as e:
                print("ROS Error : ", e)
            start_advertisement()


def start_advertisement():
    global ad_manager, haystack_advertisement
    print("Starting Advertisement")
    ad_manager.RegisterAdvertisement(haystack_advertisement.get_path(), {},
                                     reply_handler=register_ad_cb,
                                     error_handler=register_ad_error_cb)


def stop_advertisement():
    global ad_manager, haystack_advertisement
    print("Stop Advertisement")
    ad_manager.UnregisterAdvertisement(haystack_advertisement.get_path())


def disconnect_devices():
    mngr = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    mngd_objs = mngr.GetManagedObjects()
    for path in mngd_objs:
        con_state = mngd_objs[path].get('org.bluez.Device1', {}).get('Connected', False)
        if con_state:
            print(path)
            device = path
            dev_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, device),
                                   "org.bluez.Device1")
            dev_props.Disconnect()


if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    adapter = find_adapter(bus)
    if not adapter:
        print('LEAdvertisingManager1 interface not found')
        sys.exit()
    adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter),
                                   "org.freedesktop.DBus.Properties")

    adapter_props.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(1))

    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter),
                                LE_ADVERTISING_MANAGER_IFACE)

    haystack_advertisement = HaystackBluetooth(bus, 0)

    service_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter),
        GATT_MANAGER_IFACE)

    app = Application(bus)
    
    modeMonitorObj = ModeMonitor()
    modeMonitorObj.start()

    mainloop = GLib.MainLoop()

    start_advertisement()

    service_manager.RegisterApplication(app.get_path(), {},
                                        reply_handler=register_app_cb,
                                        error_handler=register_app_error_cb)

    bus.add_signal_receiver(properties_changed,
                            dbus_interface="org.freedesktop.DBus.Properties",
                            signal_name="PropertiesChanged",
                            arg0="org.bluez.Device1",
                            path_keyword="path")

    try:
        mainloop.run()
    except KeyboardInterrupt:
        mainloop.quit()

    RUNNING = False
    disconnect_devices()
    # ad_manager.UnregisterAdvertisement(haystack_advertisement)
    print('Advertisement unregistered')
    # dbus.service.Object.remove_from_connection(haystack_advertisement)
