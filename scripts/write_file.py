#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from water_drone.msg import SensorData
import json
import datetime
import time
import glob
import os

class GetSensors:
	def __init__(self) -> None:
		rospy.init_node("write_file", anonymous=True)
		self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
		self.interval = 10 * 60  # 10 min, how often to create new file
		self.last_time = 0
		self.lat = 0
		self.lon = 0

	def callback_gps(self, data: NavSatFix) -> None:
		self.data_json["time"] = data.header.stamp.secs
		self.data_json["Lat"] = data.latitude
		self.data_json["Lon"] = data.longitude

	def callback_sensors(self, data: SensorData) -> None:
		self.data_json = {}
		if data.pH != "None":
			ions_data = False
			self.data_json["temperatura"] = data.temperature
			self.data_json["pH"] = data.pH
			self.data_json["conductivity"] = data.conductivity
			self.data_json["ORP"] = data.ORP
		else:
			iona_data = True
			self.data_json["temperatura"] = data.temperature
			self.data_json["NO2"] = data.NO2
			self.data_json["NO3"] = data.NO3
			self.data_json["NH4"] = data.NH4
		if ions_data:
			file_prefix = "ions"
		else:
			file_prefix = "water"
		if time.time() - self.last_time > self.interval:
			f = open(f"/home/pi/data/{self.current_date}/{file_prefix}_{time.time()}", "w")
		else:
			list_of_files = glob.glob(f"/home/pi/data/{self.current_date}/*")
			latest_file = max(list_of_files, key=os.path.getctime)
			self.last_time = time.time()
			f = open(latest_file, "a")
		json.dump(self.data_json, f)
		f.write("\n")
		f.close()

	def write_file(self) -> None:
		rospy.Subscriber("/sensor_data", SensorData, self.callback_sensors)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
		rospy.spin()

if __name__ == "__main__":
    GetSensors().write_file()
