#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from water_drone.msg import SensorData
from std_msgs.msg import String
from mavros_msgs.msg import State
import json
import datetime
import time
import glob
import os
from uuid import getnode as get_mac
import hashlib
from rospy_message_converter import message_converter
import getpass


def _generate_pubkey(id: str) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)

def median(measurements, med_length=3):
    # Creating buffer
    meas = {}
    for key in measurements.keys():
        if measurements[key] != "None":
            meas[key] = float(measurements[key])

    if not hasattr(median, "buffer"):
        median.buffer = {}
        for key in meas.keys():
            median.buffer[key] = [float(meas[key])] * med_length

    # Move buffer to actually values ( [0, 1, 2] -> [1, 2, 3] )
    sorted_buffer = {}
    median_values = {}
    for key in median.buffer.keys():
        median.buffer[key] = median.buffer[key][1:]
        median.buffer[key].append(float(meas[key]))

        # Calculation median
        sorted_buffer[key] = median.buffer[key].copy()
        sorted_buffer[key].sort()

        median_values[key] = sorted_buffer[key][med_length // 2]

    return median_values


def easy_mean(measurements, s_k=0.4):
    meas = {}
    for key in measurements.keys():
        if measurements[key] != "None":
            meas[key] = measurements[key]
    # Creating static variable
    if not hasattr(easy_mean, "fit"):
        easy_mean.fit = meas.copy()

    # Calculation easy mean
    for key in meas.keys():
        easy_mean.fit[key] += (float(meas[key]) - float(easy_mean.fit[key])) * s_k

    return easy_mean.fit

MODEL = 3

class GetSensors:
    def __init__(self) -> None:
        rospy.init_node("write_file", anonymous=True)
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.interval = self.config["general"]["interval"]
        self.last_time = 0
        self.lat = 0
        self.lon = 0
        self.data_json = {}
        self.measurement = {}  # single measurement
        self.is_armed = False
        rospy.loginfo(f"Write file is ready. Current data {self.current_date}")
        self.pub = rospy.Publisher("new_file", String, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.get_state)

    def get_state(self, data):
        if self.is_armed != data.armed:
            rospy.loginfo(f"WRITE FILE NODE: Armed changed: {data.armed}")
            self.mac = f"{get_mac()}_{time.time()}"
            rospy.loginfo(f" New mac: {self.mac}")
        self.is_armed = data.armed

    def callback_gps(self, data: NavSatFix) -> None:
        self.data_json["timestamp"] = data.header.stamp.secs
        self.data_json["Lat"] = data.latitude
        self.data_json["Lon"] = data.longitude

    def callback_sensors(self, data: SensorData) -> None:
        if self.is_armed:
            data_dict = message_converter.convert_ros_message_to_dictionary(data) 
            self.measurement = easy_mean(median(data_dict))
            rospy.loginfo(f"Filtered: {self.measurement}, not filtered: {data_dict}")
            if data.pH == "None":
                file_prefix = "ions"
            else:
                file_prefix = "water"

            if time.time() - self.last_time > self.interval:
                filename = f"/home/{self.username}/data/{self.current_date}/{file_prefix}_{time.time()}.json"
                rospy.loginfo(f"Creating new file: {filename}")
                f = open(filename, "w")
                self.pub.publish(f"New file {filename}")
                self.last_time = time.time()
                dict_from_file = {}
            else:
                list_of_files = glob.glob(f"/home/{self.username}/data/{self.current_date}/*.json")
                latest_file = max(list_of_files, key=os.path.getctime)
                with open(latest_file) as f:
                    try:
                        dict_from_file = json.load(f)
                    except json.decoder.JSONDecodeError as e:
                        rospy.loginfo(e)
                        dict_from_file = {}
                f = open(latest_file, "w")

            if "timestamp" in self.data_json:
                public_address = _generate_pubkey(str(self.mac))
                self.measurement["timestamp"] = self.data_json["timestamp"]
                self.measurement["geo"] = (self.data_json["Lat"], self.data_json["Lon"])
                measurmnet_format_data = {
                    public_address: {
                        "model": MODEL,
                        "measurements": [self.measurement],
                    }
                }
                if public_address in dict_from_file:
                    dict_from_file[public_address]["measurements"].append(
                        self.measurement
                    )
                else:
                    dict_from_file.update(measurmnet_format_data)
                json.dump(dict_from_file, f)
                f.write("\n")
                f.close()
            else:
                return

    def write_file(self) -> None:
        rospy.Subscriber("/sensor_data", SensorData, self.callback_sensors)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    GetSensors().write_file()
