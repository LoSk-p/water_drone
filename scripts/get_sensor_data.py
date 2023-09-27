#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
from mavros_msgs.msg import State, WaypointReached
import time
import os
import datetime
import glob
import json
from sensor_msgs.msg import NavSatFix
import getpass
import subprocess

from water_drone.msg import SensorData
from water_drone.srv import RunPump
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode

MEASURE_TIMEOUT = 300

class WaspmoteSensors:
    def __init__(self) -> None:
        rospy.init_node("waspmote_sensors", anonymous=True)
        self.data_msg = None
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.interval = self.config["general"]["interval"]  #how often to create new file
        self.last_time = 0
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        # self.create_folder()
        self.is_armed = False
        self.measure = False
        rospy.loginfo(f"Get sensors is ready. Current data {self.current_date}")
        rospy.Subscriber("/mavros/state", State, self.get_state)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.start_measure)
        self.start_measure_publisher = rospy.Publisher("write_measure_status", String, queue_size=10)
        # rospy.wait_for_service('/mavros/cmd/command')
        # self.mavros_cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.wait_for_service('run_pump')
        self.run_pump = rospy.ServiceProxy('run_pump', RunPump)
        self.lat = 0
        self.lon = 0
        self.timestamp = 0
    
    # def start_pause_mission(self, command: str):
    #     response = self.mavros_cmd(command=CommandCode.DO_PAUSE_CONTINUE, 
    #                                 param1=1 if command == "pause" else 0)

    def start_measure(self, data):
        rospy.loginfo(f"Waypoint reached: {data}")
        # self.start_pause_mission("pause")
        self.run_pump(main_pump=1, pump_in=1, number_of_pump=0)
        self.measure = True
        self.start_measure_publisher.publish("measure")
        time.sleep(MEASURE_TIMEOUT)
        self.measure = False
        self.start_measure_publisher.publish("dont measure")
        # self.start_pause_mission("start")
        self.run_pump(main_pump=1, pump_in=0, number_of_pump=0)

    def callback_gps(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.timestamp = data.header.stamp.secs

    def get_state(self, data):
        if self.is_armed != data.armed:
            rospy.loginfo(f"WASMPOTE_SENSORS NODE: Armed changed: {data.armed}")
        self.is_armed = data.armed

    def get_msg_data(self, data) -> None:
        self.data_msg = SensorData()
        self.data_msg.pH = data["pH"]
        self.data_msg.conductivity = data["conductivity"]
        self.data_msg.temperature = data["temperature"]
        self.data_msg.oxxygen = data["oxxygen"]
        self.data_msg.ORP = data["ORP"]
        self.data_msg.NO2 = data["NO2"]
        self.data_msg.NO3 = data["NO3"]
        self.data_msg.NH4 = data["NH4"]
        self.data_msg.Cl = data["Cl"]

    # def create_folder(self) -> None:
    #     if not (os.path.isdir(f"/home/{self.username}/data")):
    #         os.mkdir(f"/home/{self.username}/data")
    #         rospy.loginfo("Folder 'data' created")
    #     if not (os.path.isdir(f"/home/{self.username}/data/{self.current_date}")):
    #         os.mkdir(f"/home/{self.username}/data/{self.current_date}")
    #         os.mkdir(f"/home/{self.username}/data/{self.current_date}/sent")
    #         os.mkdir(f"/home/{self.username}/data/{self.current_date}/sensors_data")
    #         os.mkdir(f"/home/{self.username}/data/{self.current_date}/gps")
    #         rospy.loginfo("New data folders created")

    def find_usb_ports(self):
        ports_available = glob.glob('/dev/ttyUSB[0-9]')
        rospy.loginfo(f"USB ports: {ports_available}")
        fail_count = 0
        while len(ports_available) < 2:
            fail_count += 1
            if fail_count > 5:
                self.restart_usb()
                fail_count = 0
            time.sleep(5)
            ports_available = glob.glob('/dev/ttyUSB[0-9]')
            rospy.loginfo(f"Wait for USB ports. Now: {ports_available}")
        return ports_available

    def restart_usb(self):
        rospy.loginfo("Restart USB")
        subprocess.call(["uhubctl", "-l", "1-1", "-p", "3", "-a", "off"])
        time.sleep(1)
        subprocess.call(["uhubctl", "-l", "1-1", "-p", "3", "-a", "on"])
    
    def get_serial(self, ports):
        ser = serial.Serial(
            ports[0],
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        ser1 = serial.Serial(
            ports[1],
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        return ser, ser1

    def publish_data(self) -> None:
        ports = self.find_usb_ports()
        try:
            ser, ser1 = self.get_serial(ports)
        except Exception as e:
            rospy.logerr(f"Exception in open serial: {e}. Restarting USB..")
            self.restart_usb()
            ports = self.find_usb_ports()
            ser, ser1 = self.get_serial(ports)
        time.sleep(1)
        pub = rospy.Publisher("sensor_data", SensorData, queue_size=10)
        data = {
                "timestamp": "None",
                "lat": "None",
                "lon": "None",
                "temperature": "None",
                "pH": "None",
                "conductivity": "None",
                "oxxygen": "None",
                "ORP": "None",
                "NO2": "None",
                "NO3": "None",
                "NH4": "None",
                "Cl": "None"
            }
        ser1_fail_count = 0
        ser2_fail_count = 0
        while not rospy.is_shutdown():
            time.sleep(1)
            try:
                if self.is_armed and self.measure:
                    if ser.inWaiting() > 0 or ser1.inWaiting() > 0:
                        if ser.inWaiting() > 0:
                            data_read = str(ser.readline())
                            if "J#" in data_read:
                                ser1_fail_count += 1
                            rospy.loginfo(f"Sensors data: {data_read}")
                        if ser1.inWaiting() > 0:
                            data_read = str(ser1.readline())
                            if "J#" in data_read:
                                ser2_fail_count += 1
                            rospy.loginfo(f"Sensors data 1: {data_read}")
                        if time.time() - self.last_time > self.interval:
                            rospy.loginfo("in if new file get_sensor")
                            f = open(
                                f"/home/{self.username}/data/{self.current_date}/sensors_data/{time.time()}.json", "w"
                            )
                            self.last_time = time.time()
                        else:
                            list_of_files = glob.glob(
                                f"/home/{self.username}/data/{self.current_date}/sensors_data/*.json"
                            )
                            latest_file = max(list_of_files, key=os.path.getctime)
                            f = open(latest_file, "a")
                        data_prev = data_read[2:]
                        if data_prev[0] == "$":
                            data_prev = data_prev.split("|")
                            if data_prev[0] == "$w":
                                data["temperature"] = data_prev[1]
                                data["pH"] = data_prev[2]
                                data["conductivity"] = data_prev[3]
                                data["ORP"] = data_prev[5]
                                data["oxxygen"] = data_prev[4]
                            elif data_prev[0] == "$i":
                                # data["temperature"] = data_prev[1]
                                data["NH4"] = data_prev[2]
                                data["NO3"] = data_prev[3]
                                data["NO2"] = data_prev[4]
                                data["Cl"] = data_prev[5]
                            data["timestamp"] = self.timestamp
                            data["lat"] = self.lat
                            data["lon"] = self.lon
                            data_time = data.copy()
                            data_time["time"] = time.time()
                            rospy.loginfo(f"Write file data: {data_time}")
                            f.write(f"{data_time}\n")
                            f.close()
                            rospy.loginfo(ser1_fail_count)
                            rospy.loginfo(ser2_fail_count)
                            if (data["pH"] != "None" and data["NO2"] != "None") or (ser1_fail_count > 5) or (ser2_fail_count > 5):
                                ser1_fail_count = 0
                                ser2_fail_count = 0
                                rospy.loginfo(f"JSON sensor data: {data}")
                                self.get_msg_data(data)
                                pub.publish(self.data_msg)
                                data = {
                                    "timestamp": "None",
                                    "lat": "None",
                                    "lon": "None",
                                    "temperature": "None",
                                    "pH": "None",
                                    "conductivity": "None",
                                    "ORP": "None",
                                    "NO2": "None",
                                    "NO3": "None",
                                    "NH4": "None",
                                    "Cl": "None"
                                }                               
            except Exception as e:
                rospy.logerr(f"Exception in get sensors data: {e}")
                self.restart_usb()
                ports = self.find_usb_ports()
                ser, ser1 = self.get_serial(ports)

WaspmoteSensors().publish_data()
