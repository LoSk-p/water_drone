#!/usr/bin/env python

import requests
import rospy
import time
import datetime
import glob
from pinatapy import PinataPy
from robonomicsinterface import Account, Datalog
import os
from std_msgs.msg import String
from mavros_msgs.msg import State
from uuid import getnode as get_mac
import json
import getpass
import hashlib
from copy import deepcopy
from water_drone.msg import NewPump

USE_IPFS = False

def _generate_pubkey(id: str) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)

class Sender:
    def __init__(self) -> None:
        rospy.init_node("sender", anonymous=True)
        self.timestamp = 0
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        rospy.loginfo(f"Sender is ready. Current date {self.current_date}")
        rospy.Subscriber("/new_file", String, self._parse)
        rospy.Subscriber("/new_pump", NewPump, self._parse_new_pump)
        while not rospy.is_shutdown():
            pass

    def pin_file_to_pinata(self, file_path: str) -> str:
        pinata_api = self.config["pinata"]["api"]
        pinata_secret = self.config["pinata"]["secret"]
        if pinata_secret:
            try:
                rospy.loginfo(f"Pinning file {file_path} to Pinata")
                pinata = PinataPy(pinata_api, pinata_secret)
                pinata.pin_file_to_ipfs(file_path, save_absolute_paths=False)
                ipfs_hash = pinata.pin_list()["rows"][0]["ipfs_pin_hash"]
                rospy.loginfo(f"File sent to pinata. Hash is {ipfs_hash}")
                return ipfs_hash
            except requests.exceptions.ConnectionError as e:
                rospy.logerr(f"Failed while pining file to Pinata. Error: {e}")
                return "No internet"
            except Exception as e:
                rospy.logerr(f"Unexpected exception in pinning file {e}")
                return

    def create_datalog(self, data_for_datalog: str):
        account = Account(seed=self.config["robonomics"]["seed"])
        try:
            datalog = Datalog(account)
            rospy.loginfo(f"Start creating datalog witn {data_for_datalog}")
            transaction_hash = datalog.record(data_for_datalog)
            rospy.loginfo(
                f"Ipfs hash sent to Robonomics Parachain. Transaction hash is: {transaction_hash}"
            )

        except Exception as e:
            rospy.logerr(f"Failed to send hash to Robonomics with: {e}")
            time.sleep(10)

    def _parse_new_pump(self, data):
        mac = f"{get_mac()}_{time.time()}"
        public_key = _generate_pubkey(str(mac))
        json_for_datalog = {public_key: {"model": 2, "geo": f"{data.lat},{data.lon}", "measurements": [{"timestamp": data.timestamp, "pump_number": data.pump_number}]}}
        self.create_datalog(json.dumps(json_for_datalog))

    def _parse(self, from_topic) -> None:
        list_of_files = glob.glob(f"/home/{self.username}/data/{self.current_date}/*.json")
        latest_file = max(list_of_files, key=os.path.getctime)
        if latest_file != from_topic.data:
            list_of_files.remove(latest_file)
        for file_path in list_of_files:
            if USE_IPFS:
                ipfs_hash = self.pin_file_to_pinata(file_path)
                if ipfs_hash and (ipfs_hash != "No internet"):
                    data_for_datalog = ipfs_hash
                else:
                    return
            else:
                with open(file_path) as f:
                    try:
                        dict_from_file = json.load(f)
                        public_key = list(dict_from_file.keys())[0]
                        mean_values = deepcopy(dict_from_file[public_key]["mean_values"])
                        mean_values.pop("geo")

                        #######################################
                        if float(mean_values["NO3"]) > 1000 or float(mean_values["NO3"]) < 0:
                            mean_values.pop("NO3")
                        if float(mean_values["NO2"]) > 1000 or float(mean_values["NO2"]) < 0:
                            mean_values.pop("NO2")
                        if float(mean_values["Cl"]) > 1000 or float(mean_values["Cl"]) < 0:
                            mean_values.pop("Cl")
                        #######################################

                        geo = dict_from_file[public_key]["mean_values"]["geo"]
                        json_for_datalog = {public_key: {"model": 3, "geo": f"{geo[0]},{geo[1]}", "measurements": [mean_values]}}
                        data_for_datalog = json.dumps(json_for_datalog)
                    except json.decoder.JSONDecodeError as e:
                        rospy.loginfo(e)
                        dict_from_file = {}
                        data_for_datalog = ""
            self.create_datalog(data_for_datalog)
            os.replace(
                file_path,
                f"/home/{self.username}/data/{self.current_date}/sent/{file_path.split('/')[-1]}",
            )


sender = Sender()
