#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from water_drone.msg import SensorData

class GetSensors:
	def __init__(self):
		rospy.init_node("write_file", anonymous=True)
		self.data_gps = None
		self.data_sensors = None
	def callback_gps(self, data):
		#print(data.timestamp)
		self.data_gps = "time: " + str(data.header.stamp.secs) + "; Lat: " + str(data.latitude) + "; Lon: " + str(data.longitude)
	def callback_sensors(self, data):
		self.data_sensors = "; Temperature: " + str(data.temperature) + "; pH: " + str(data.pH) + "; conductivity: " + str(data.conductivity) + ";\n"
		with open("gps+sensors", "a") as file:
			file.write(self.data_gps + self.data_sensors)
	def write_file(self):
		rospy.Subscriber("/sensor_data", SensorData, self.callback_sensors)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
		rospy.spin()
if __name__ == '__main__':
	GetSensors().write_file()	
