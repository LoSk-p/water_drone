#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
import time
import os
import datetime
import glob

from water_drone.msg import SensorData


class WaspmoteSensors:
    def __init__(self) -> None:
        self.data_msg = None
        self.interval = 10 * 60  # 10 min, how often to create new file
        self.last_time = 0
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        self.create_folder()
        rospy.init_node("waspmote_sensors", anonymous=True)

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
        if not (os.path.isdir(f"/home/pi/data")):
            os.mkdir(f"/home/pi/data")
        if not (os.path.isdir(f"/home/pi/data/{self.current_date}")):
            os.mkdir(f"/home/pi/data/{self.current_date}")
            os.mkdir(f"/home/pi/data/{self.current_date}/sent")
            os.mkdir(f"/home/pi/data/{self.current_date}/sensors_data")

    def publish_data(self) -> None:
        ser = serial.Serial(
            "/dev/ttyUSB0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        time.sleep(1)
        pub = rospy.Publisher("sensor_data", SensorData, queue_size=10)
        while not rospy.is_shutdown():
            data = {
                "temperature": "None",
                "pH": "None",
                "conductivity": "None",
                "ORP": "None",
                "NO2": "None",
                "NO3": "None",
                "NH4": "None",
            }
            if time.time() - self.last_time > self.interval:
                f = open(
                    f"/home/pi/data/{self.current_date}/sensors_data/{time.time()}", "w"
                )
            else:
                list_of_files = glob.glob(
                    f"/home/pi/data/{self.current_date}/sensors_data/*"
                )
                latest_file = max(list_of_files, key=os.path.getctime)
                self.last_time = time.time()
                f = open(latest_file, "a")
            if ser.inWaiting() > 0:
                data_read = str(ser.readline())
                data_prev = data_read[2:]
                print(data_prev)
                if data_prev[0] == "<":
                    data_prev = data_prev.split("#")
                    if data_prev[2] == "WATER":
                        data = {
                            "temperature": data_prev[4].split(":")[1],
                            "pH": data_prev[5].split(":")[1],
                            "conductivity": data_prev[6].split(":")[1],
                            "ORP": data_prev[7].split(":")[1],
                        }
                    else:
                        data = {
                            "temperature": data_prev[7].split(":")[1],
                            "NO2": data_prev[4].split(":")[1],
                            "NO3": data_prev[5].split(":")[1],
                            "NH4": data_prev[6].split(":")[1],
                        }
                    data_time = data.copy()
                    data_time["time"] = time.time()
                    f.write(f"{data_time}\n")
                    f.close()
                    self.get_msg_data(data)
                    pub.publish(self.data_msg)


WaspmoteSensors().publish_data()
