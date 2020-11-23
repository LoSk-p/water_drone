
#!/usr/bin/env python3 

from config.default import drone
import json
import requests
import subprocess
import rospy
import time


class Sender:
	def __init__(self):
		self.timestamp = 0
		try:
			with open("/home/ubuntu/catkin_ws/src/water_drone/config/last_date", "r") as file:
				for line in file:
					line = json.loads(line)
					self.timestamp = line["time"]
					print(self.timestamp)
		except Exception as e:
			print(e)
			print("inside except")
			pass

	def _parse(self):
		#print(self.timestamp)

		try:
			with open("/home/ubuntu/data/gps-sensors.json", "r") as file:
				for data in file:
					data = json.loads(data)
					key = drone()['SECRET']
					dir = drone()['DIR']

					if "time" in data.keys():
						if int(data["time"]) > self.timestamp:
							program = f"echo '{data}' | {dir}robonomics io write datalog --remote wss://substrate.ipci.io -s {key}"
							#program = "echo " + str(data) + " | " + dir + "robonomics io write datalog -s " + key
							process_robonomics = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
							output = process_robonomics.stdout.readline()
							self.timestamp = int(data["time"])
							with open("/home/ubuntu/catkin_ws/src/water_drone/config/last_date", "w") as cash:
								cash.write(str(json.dumps(data)))
							print(f'Data sent to DAO IPCI {output.strip()}')
					elif int(data["timestamp"]) > self.timestamp:
						print(f'fromdate {data["timestamp"]}')
						print(f'old time {self.timestamp}')
						self.timestamp = int(data["timestamp"])
						date = data["date"]
						program = f"echo '{date}' | {dir}robonomics io write datalog --remote wss://substrate.ipci.io -s {key}"
						process_robonomics = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)

		except Exception as e:
			rospy.logerr(f'error: {e}')
			rospy.loginfo('after e')
			time.sleep(2)

	def _check_connection(self):
		while True:
			process = subprocess.Popen("iwconfig", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			networks = str(process.communicate())
			essid = networks.find("ESSID") + 6
			essid_end = networks.find("Mode") - 14
			if networks[essid:essid_end] != 'off/any':
				self._parse()

sender = Sender()
sender._check_connection()

