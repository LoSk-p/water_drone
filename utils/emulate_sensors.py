#!/usr/bin/env python3
import time
import rospy
from water_drone.msg import SensorData
import typing as tp
import json
import ast

def get_msg_data(data: tp.Dict[str, float]) -> SensorData:
    data_msg = SensorData()
    data_msg.pH = data["pH"]
    data_msg.conductivity = data["conductivity"]
    data_msg.temperature = data["temperature"]
    data_msg.ORP = data["ORP"]
    return data_msg


def read_data() -> None:
    rospy.init_node("reader_sensor_data", anonymous=True)
    pub_sensor = rospy.Publisher("sensor_data", SensorData, queue_size=10)
    data_dict = {}
    while not rospy.is_shutdown():
        time.sleep(0.4)
        with open("/home/pi/data/2022_08_15/sensors_data/1660577237.9745145.json", "r") as f:
            for l in f:
                data = ast.literal_eval(l)
                data = {
                    "temperature": data["temperature"],
                    "pH": data["pH"],
                    "conductivity": data["conductivity"],
                    "ORP": data["ORP"]
                }
                print(data)
                data_msg = get_msg_data(data)
                pub_sensor.publish(data_msg)
                time.sleep(5)


            # data_prev = line[2:]
            # if data_prev[0] == "<":
            #     data_prev = data_prev.split("#")
            #     data = {
            #         "temperature": float(data_prev[4].split(":")[1]),
            #         "pH": float(data_prev[5].split(":")[1]),
            #         "conductivity": float(data_prev[7].split(":")[1]),
            #     }
            #     data_msg = get_msg_data(data)
            #     pub_sensor.publish(data_msg)
            #     time.sleep(2)
                # print(data_prev)


read_data()
