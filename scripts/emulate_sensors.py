#!/usr/bin/env python3
import time
import rospy
from water_drone.msg import SensorData

def get_msg_data(data):
	data_msg = SensorData()
	data_msg.pH = data['pH']
	data_msg.conductivity = data['conductivity']
	data_msg.temperature = data['temperature']
	return(data_msg)

def read_data():
	rospy.init_node("reader_sensor_data", anonymous=True)
	pub_sensor = rospy.Publisher("sensor_data", SensorData, queue_size=10)
	while not rospy.is_shutdown():
		time.sleep(0.4)
		file = open("sensor_data_saved", "r")
		print("hell")
		for line in file:
			data_prev = line[2:]
			if data_prev[0] == "<":
				data_prev = data_prev.split('#')
				data = {'temperature': float(data_prev[4].split(':')[1]), 'pH': float(data_prev[5].split(':')[1]), 'conductivity': float(data_prev[7].split(':')[1])}
				data_msg = get_msg_data(data)
				pub_sensor.publish(data_msg)
				time.sleep(2)
			#print(data_prev)
		file.close()

read_data()
