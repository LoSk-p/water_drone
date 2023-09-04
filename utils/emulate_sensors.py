#!/usr/bin/env python3
import time
import rospy
from water_drone.msg import SensorData
import typing as tp
import json
import ast
import getpass
import datetime
import random

def get_msg_data(data: tp.Dict[str, float]) -> SensorData:
    data_msg = SensorData()
    data_msg.pH = data["pH"]
    data_msg.conductivity = data["conductivity"]
    data_msg.temperature = data["temperature"]
    data_msg.ORP = data["ORP"]
    data_msg.NH4 = data["NH4"]
    data_msg.NO2 = data["NO2"]
    data_msg.NO3 = data["NO3"]
    return data_msg


def read_data() -> None:
    rospy.init_node("reader_sensor_data", anonymous=True)
    pub_sensor = rospy.Publisher("sensor_data", SensorData, queue_size=10)
    while not rospy.is_shutdown():
        time.sleep(0.4)
        data = {
            "temperature": str(random.randint(1, 100)),
            "pH": str(random.randint(1, 100)),
            "conductivity": str(random.randint(1, 100)),
            "ORP": str(random.randint(1, 100)),
            "NH4": str(random.randint(1, 100)),
            "NO2": str(random.randint(1, 100)),
            "NO3": str(random.randint(1, 100))
        }
        print(data)
        data_msg = get_msg_data(data)
        pub_sensor.publish(data_msg)
        time.sleep(5)

read_data()
