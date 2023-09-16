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
from copy import deepcopy


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
        self.create_folder()
        self.interval = self.config["general"]["interval"]
        self.last_time = 0
        self.lat = 0
        self.lon = 0
        self.data_json = {}
        self.measurement = {}  # single measurement
        self.is_armed = False 
        self.data_filename = ""
        self.measure = False
        self.mac = f"{get_mac()}_{time.time()}"
        rospy.loginfo(f"Write file is ready. Current data {self.current_date}")
        self.publisher = rospy.Publisher("new_file", String, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.get_state)
        rospy.Subscriber("/write_measure_status", String, self.get_measure_status)
        self.gps_filename = f"/home/{self.username}/data/{self.current_date}/gps/gps-{round(time.time())}"
        self.gps_counter = 0

    def create_folder(self) -> None:
        if not (os.path.isdir(f"/home/{self.username}/data")):
            os.mkdir(f"/home/{self.username}/data")
            rospy.loginfo("Folder 'data' created")
        if not (os.path.isdir(f"/home/{self.username}/data/{self.current_date}")):
            os.mkdir(f"/home/{self.username}/data/{self.current_date}")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/sent")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/sensors_data")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/gps")
            os.mkdir(f"/home/{self.username}/data/{self.current_date}/pumps")
            rospy.loginfo("New data folders created")

    def get_measure_status(self, data):
        if data.data == "measure":
            self.data_filename = f"/home/{self.username}/data/{self.current_date}/{round(time.time())}.json"
            with open(self.data_filename, "w") as f:
                pass
            self.measure = True
        elif data.data == "dont measure":
            self.measure = False
            self.publisher.publish(self.data_filename)

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
        if self.gps_counter > 30:
            self.gps_counter = 0
            with open(self.gps_filename, "a") as f:
                f.write(f"{data.header.stamp.secs},{data.latitude},{data.longitude}\n")
        else:
            self.gps_counter += 1

    def callback_sensors(self, data: SensorData) -> None:
        rospy.loginfo(f"armed: {self.is_armed}, measure: {self.measure}")
        if self.is_armed and self.measure:
            data_dict = message_converter.convert_ros_message_to_dictionary(data) 
            self.measurement = easy_mean(median(data_dict))
            rospy.loginfo(f"Filtered: {self.measurement}, not filtered: {data_dict}")

            with open(self.data_filename) as f:
                try:
                    dict_from_file = json.load(f)
                except json.decoder.JSONDecodeError as e:
                    rospy.loginfo(f"Error in json load: {e}")
                    dict_from_file = {}
            f = open(self.data_filename, "w")

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
                values_for_mean = dict_from_file[public_address]["measurements"][len(dict_from_file[public_address]["measurements"])//2:]
                mean_values = deepcopy(values_for_mean[0])
                if "geo" in mean_values:
                    mean_values.pop("geo", None)
                if "timestamp" in mean_values:
                    mean_values.pop("timestamp", None)
                number_of_meas = len(values_for_mean)
                for measurement in values_for_mean:
                    for key in mean_values.keys():
                        mean_values[key] += measurement[key]
                for key in mean_values.keys():
                    mean_values[key] = mean_values[key]/number_of_meas
                if "geo" in values_for_mean[0]:
                    mean_values["geo"] = values_for_mean[0]["geo"]
                if "timestamp" in values_for_mean[-1]:
                    mean_values["timestamp"] = values_for_mean[-1]["timestamp"]
                dict_from_file[public_address]["mean_values"] = mean_values
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
