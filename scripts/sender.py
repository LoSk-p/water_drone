#!/usr/bin/env python3

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
import json
import getpass

class Sender:
    def __init__(self) -> None:
        self.timestamp = 0
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        rospy.loginfo("here")
        self.is_armed = False
        rospy.loginfo(f"Sender is ready. Current date {self.current_date}")
        rospy.init_node("sender", anonymous=True)
        rospy.Subscriber("/new_file", String, self._parse)
        rospy.Subscriber("/mavros/state", State, self.get_state)
        while not rospy.is_shutdown():
            pass

    def get_state(self, data):
        if self.is_armed != data.armed:
            rospy.loginfo(f"DATA_SENDER NODE: Armed changed: {data.armed}")
        self.is_armed = data.armed

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

    def _parse(self, from_topic) -> None:
        # if self.is_armed:
        list_of_files = glob.glob(f"/home/{self.username}/data/{self.current_date}/*.json")
        latest_file = max(list_of_files, key=os.path.getctime)
        if latest_file != from_topic.data:
            list_of_files.remove(latest_file)
        account = Account(seed=self.config["robonomics"]["seed"])
        for file_path in list_of_files:
            ipfs_hash = self.pin_file_to_pinata(file_path)
            if ipfs_hash and (ipfs_hash != "No internet"):
                try:
                    datalog = Datalog(account)
                    transaction_hash = datalog.record(ipfs_hash)
                    rospy.loginfo(
                        f"Ipfs hash sent to Robonomics Parachain. Transaction hash is: {transaction_hash}"
                    )
                    os.replace(
                        file_path,
                        f"/home/{self.username}/data/{self.current_date}/sent/{file_path.split('/')[-1]}",
                    )

                except Exception as e:
                    rospy.logerr(f"Failed to send hash to Robonomics with: {e}")
                    time.sleep(10)
            else:
                return

sender = Sender()
