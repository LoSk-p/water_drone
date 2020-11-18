#!/usr/bin/env python3 

import json
import requests
from wifi import Cell
import subprocess
from dataclasses import dataclass


@dataclass()
class Sender:
	timestamp: float  = 0
	def parse(self):
		header = {"Content-type": "application/json"}
		with open("/home/ubuntu/data/gps-sensors.json", "r") as f:
			for data in f:
				data = json.loads(data)
				if int(data["time"]) > self.timestamp:
					r = requests.post('http://connectivity.robonomics.network:8001/', data=json.dumps(data), headers=header)
					self.timestamp = int(data["time"])
					print(data)


	def check_connection(self):
		while True:
			process = subprocess.Popen("iwconfig", stdout=subprocess.PIPE)
			networks = str(process.communicate())
			essid = networks.find("ESSID") + 6
			essid_end = networks.find("Mode") - 14
			if networks[essid:essid_end] != 'off/any':
				self.parse()

Sender().check_connection()
