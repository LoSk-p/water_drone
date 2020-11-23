
#!/usr/bin/env python3 

from config.default import drone
import json
import requests
import subprocess
from dataclasses import dataclass
import rospy
import time

@dataclass()
class Sender:
	timestamp: float  = 0
	def _parse(self):
		
		header = {"Content-type": "application/json"}
		try:
			with open("/home/ubuntu/data/gps-sensors.json", "r") as file:
				for data in file:
					data = json.loads(data)
					if int(data["time"]) > self.timestamp:
						key = drone()['SECRET']
						dir = drone()['DIR']
						program = f"echo '{data}' | {dir}robonomics io write datalog --remote wss://substrate.ipci.io -s {key}"
						#program = "echo " + str(data) + " | " + dir + "robonomics io write datalog -s " + key
						process_robonomics = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
						output = process_robonomics.stdout.readline()
						self.timestamp = int(data["time"])
						print(f'Data sent to DAO IPCI {output.strip()}')
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

Sender()._check_connection()

