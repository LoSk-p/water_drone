#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
import time
from water_drone.msg import SensorData

class WaspmoteSensors:
	def __init__(self):
		self.data_msg = None
		rospy.init_node("waspmote_sensors", anonymous = True)
	def get_msg_data(self, data):
		self.data_msg = SensorData()
		self.data_msg.pH = data['pH']
		self.data_msg.conductivity = data['conductivity']
		self.data_msg.temperature = data['temperature']

	def publish_data(self):
		ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
		time.sleep(1)
		pub = rospy.Publisher("sensor_data", SensorData, queue_size = 10)
		while not rospy.is_shutdown():
			with open('/home/ubuntu/sensor_data_with_time', 'a') as f:
				if ser.inWaiting() > 0:
					data = str(ser.readline())
					data_prev = data[2:]
					print(data_prev)
					if data_prev[0] == "<":
						data_prev = data_prev.split('#')
						data = {'temperature': data_prev[4].split(':')[1], 
							'pH': data_prev[5].split(':')[1],
							'conductivity': data_prev[7].split(':')[1]}
						data_time = {'time': time.time(),'temperature': data_prev[4].split(':')[1], 
							'pH': data_prev[5].split(':')[1],
							'conductivity': data_prev[7].split(':')[1]}
						f.write(f'{data_time}\n')
						self.get_msg_data(data)
						pub.publish(self.data_msg)

WaspmoteSensors().publish_data()
