#!/usr/bin/env python3

import requests
import rospy
import time
import datetime
import glob
from pinatapy import PinataPy
import robonomicsinterface as RI
import os


class Sender:
    def __init__(self) -> None:
        self.timestamp = 0
        self.config = open("/home/pi/catkin_ws/src/water_dron/config/config.json")
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))

    def pin_file_to_pinata(self, file_path: str) -> str:
        pinata_api = self.config["pinata"]["api"]
        pinata_secret = self.config["pinata"]["secret"]
        if pinata_secret:
            try:
                rospy.loginfo("Pinning file to Pinata")
                pinata = PinataPy(pinata_api, pinata_secret)
                pinata.pin_file_to_ipfs(file_path)
                hash = pinata.pin_list()["rows"][0]["ipfs_pin_hash"]
                rospy.loginfo(f"File sent to pinata. Hash is {hash}")
                return hash
            except requests.exceptions.ConnectionError as e:
                rospy.logwarn(f"Failed while pining file to Pinata. Error: {e}")
                return
            except Exception as e:
                return

    def _parse(self) -> None:
        list_of_files = glob.glob(f"/home/pi/data/{self.current_date}/*")
        latest_file = max(list_of_files, key=os.path.getctime)
        account = RI.Account(seed=self.config["robonomics"]["seed"])
        for file_path in list_of_files:
            if file_path != latest_file:  # the latest can be modified right now
                hash = self.pin_file_to_pinata(file_path)
                if hash:
                    try:
                        datalog = RI.Datalog(account)
                        transaction_hash = datalog.record(hash)
                        rospy.loginfo(
                            f"Ipfs hash sent to Robonomics Parachain and included in block {transaction_hash}"
                        )
                        os.replace(
                            file_path,
                            f"/home/pi/data/{self.current_date}/sent/{file_path.split('/')[-1]}",
                        )

                    except Exception as e:
                        rospy.logwarn(f"Failed to send hash to Robonomics with: {e}")
                        time.sleep(10)


sender = Sender()
