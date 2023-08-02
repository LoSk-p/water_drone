#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
from mavros_msgs.msg import State
import time
import os
import datetime
import glob
import json
from sensor_msgs.msg import NavSatFix
import RPi.GPIO as GPIO
import getpass

from water_drone.msg import SensorData

MEASURE_TIMEOUT = 10

class WaspmoteSensors:
    def __init__(self) -> None:
        self.data_msg = None
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.interval = self.config["general"]["interval"]  #how often to create new file
        self.last_time = 0
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        self.create_folder()
        self.is_armed = True
        self.measure = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config["rpi_pins"]["sensor_1"], GPIO.IN)
        GPIO.setup(self.config["rpi_pins"]["sensor_2"], GPIO.IN)
        GPIO.setup(self.config["rpi_pins"]["pump_in"], GPIO.OUT)
        GPIO.setup(self.config["rpi_pins"]["pump_out"], GPIO.OUT)
        GPIO.output(self.config["rpi_pins"]["pump_in"], GPIO.HIGH)
        GPIO.output(self.config["rpi_pins"]["pump_out"], GPIO.HIGH)
        rospy.init_node("waspmote_sensors", anonymous=True)
        rospy.loginfo(f"Get sensors is ready. Current data {self.current_date}")
        rospy.Subscriber("/mavros/state", State, self.get_state)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
        rospy.Subscriber("/start_measure", String, self.start_measure)
    
    def pump_in(self):
        rospy.loginfo("Start pump in water")
        GPIO.output(self.config["rpi_pins"]["pump_in"], GPIO.LOW)
        while GPIO.input(self.config["rpi_pins"]["sensor_1"]):
            time.sleep(0.5)
        GPIO.output(self.config["rpi_pins"]["pump_in"], GPIO.HIGH)
        self.measure = True
        rospy.loginfo("Finished pump in water")
    
    def pump_out(self):
        rospy.loginfo("Start pump out water")
        self.measure = False
        GPIO.output(self.config["rpi_pins"]["pump_out"], GPIO.LOW)
        while not GPIO.input(self.config["rpi_pins"]["sensor_2"]):
            time.sleep(0.5)
        GPIO.output(self.config["rpi_pins"]["pump_out"], GPIO.HIGH)
        rospy.loginfo("Finished pump out water")

    def start_measure(self, data):
        rospy.loginfo(f"Measure: {data}")
        self.pump_in()
        time.sleep(MEASURE_TIMEOUT)
        self.pump_out()

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
        self.data_msg.ORP = data["ORP"]
        self.data_msg.NO2 = data["NO2"]
        self.data_msg.NO3 = data["NO3"]
        self.data_msg.NH4 = data["NH4"]

    def create_folder(self) -> None:
        if not (os.path.isdir(f"/home/{self.username}/data")):
            os.mkdir(f"/home/{self.username}/data")
            rospy.loginfo("Folder 'data' created")
        if not (os.path.isdir(f"/home/{self.username}/data/{self.current_date}")):
            os.mkdir(f"/home/{self.username}/data/{self.current_date}")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/sent")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/sensors_data")
            rospy.loginfo("New data folders created")

    def publish_data(self) -> None:
        # ser = serial.Serial(
        #     "/dev/ttyUSB0",
        #     baudrate=115200,
        #     parity=serial.PARITY_NONE,
        #     stopbits=serial.STOPBITS_ONE,
        #     bytesize=serial.EIGHTBITS,
        # )
        time.sleep(1)
        pub = rospy.Publisher("sensor_data", SensorData, queue_size=10)
        while not rospy.is_shutdown():
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
            }
            time.sleep(1)
            rospy.loginfo(f"sensor 1: {GPIO.input(self.config['rpi_pins']['sensor_1'])}, sensor 2: {GPIO.input(self.config['rpi_pins']['sensor_2'])}")
            if self.is_armed and self.measure:
                rospy.loginfo("Measure")
                # if ser.inWaiting() > 0:
                #     data_read = str(ser.readline())
                #     if time.time() - self.last_time > self.interval:
                #         rospy.loginfo("in if new file get_sensor")
                #         f = open(
                #             f"/home/{self.username}/data/{self.current_date}/sensors_data/{time.time()}.json", "w"
                #         )
                #         self.last_time = time.time()
                #     else:
                #         list_of_files = glob.glob(
                #             f"/home/{self.username}/data/{self.current_date}/sensors_data/*.json"
                #         )
                #         latest_file = max(list_of_files, key=os.path.getctime)
                #         f = open(latest_file, "a")
                #     data_prev = data_read[2:]
                #     rospy.loginfo(f"Sensors data: {data_prev}")
                #     if data_prev[0] == "<":
                #         data_prev = data_prev.split("#")
                #         if data_prev[2] == "WATER":
                #             data["temperature"] = data_prev[4].split(":")[1]
                #             data["pH"] = data_prev[5].split(":")[1]
                #             data["conductivity"] = data_prev[6].split(":")[1]
                #             data["ORP"] = data_prev[7].split(":")[1]
                #         else:
                #             data["temperature"] = data_prev[7].split(":")[1]
                #             data["NO2"] = data_prev[4].split(":")[1]
                #             data["NO3"] = "None"
                #             # data["NO3"] = data_prev[5].split(":")[1]
                #             data["NH4"] = data_prev[6].split(":")[1]
                #         data["timestamp"] = self.timestamp
                #         data["lat"] = self.lat
                #         data["lon"] = self.lon
                #         rospy.loginfo(f"JSON sensor data: {data}")
                #         data_time = data.copy()
                #         data_time["time"] = time.time()
                #         f.write(f"{data_time}\n")
                #         f.close()
                #         self.get_msg_data(data)
                #         pub.publish(self.data_msg)


WaspmoteSensors().publish_data()
