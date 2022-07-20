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

    def create_folder(self) -> None:
        if not (os.path.isdir(f"/home/pi/data/{self.current_date}")):
            os.mkdir(f"/home/pi/data/{self.current_date}")
            os.mkdir(f"/home/pi/data/{self.current_date}/sent")

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
            if time.time() - self.last_time > self.interval:
                f = open(f"/home/pi/data/{self.current_date}/{time.time()}", "w")
            else:
                list_of_files = glob.glob(f"/home/pi/data/{self.current_date}/*")
                latest_file = max(list_of_files, key=os.path.getctime)
                self.last_time = time.time()
                f = open(latest_file, "a")
            if ser.inWaiting() > 0:
                data = str(ser.readline())
                data_prev = data[2:]
                print(data_prev)
                if data_prev[0] == "<":
                    data_prev = data_prev.split("#")
                    data = {
                        "temperature": data_prev[4].split(":")[1],
                        "pH": data_prev[5].split(":")[1],
                        "conductivity": data_prev[7].split(":")[1],
                    }
                    data_time = {
                        "time": time.time(),
                        "temperature": data_prev[4].split(":")[1],
                        "pH": data_prev[5].split(":")[1],
                        "conductivity": data_prev[7].split(":")[1],
                    }
                    f.write(f"{data_time}\n")
                    f.flush()
                    f.close()
                    self.get_msg_data(data)
                    pub.publish(self.data_msg)


WaspmoteSensors().publish_data()
