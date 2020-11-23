
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from water_drone.msg import SensorData
import json

class GetSensors:
	def __init__(self):
		rospy.init_node("write_file", anonymous=True)
		self.data_json = {'time': 0, 'Lat': 0, 'Lon': 0, 'temp': 0, 'pH': 0, 'cond': 0}
		self.data_gps = None
		self.data_sensors = None
		self.lat = 59.0
		self.lon = 30.0
	def callback_gps(self, data):
		#print(data.timestamp)
		self.data_json['time'] = data.header.stamp.secs
		if data.latitude == 0:
			self.lat += 0.01
			self.data_json['Lat'] = self.lat
		else:
			self.data_json['Lat'] = data.latitude
		if data.longitude == 0:
			self.lon += 0.01
			self.data_json['Lon'] = self.lon
		else:
			self.data_json['Lon'] = data.longitude
		self.data_gps = "time: " + str(data.header.stamp.secs) + "; Lat: " + str(data.latitude) + "; Lon: " + str(data.longitude)
	def callback_sensors(self, data):
		self.data_json['temp'] = data.temperature
		self.data_json['pH'] = data.pH
		self.data_json['cond'] = data.conductivity
		self.data_sensors = "; Temperature: " + str(data.temperature) + "; pH: " + str(data.pH) + "; conductivity: " + str(data.conductivity) + ";\n"
		with open("/home/ubuntu/data/gps-sensors.json", "a") as file:
			#file.write(self.data_gps + self.data_sensors)
			json.dump(self.data_json, file)
			file.write("\n")
			print(self.data_json)
	def write_file(self):
		rospy.Subscriber("/sensor_data", SensorData, self.callback_sensors)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
		rospy.spin()
if __name__ == '__main__':
	GetSensors().write_file()	
