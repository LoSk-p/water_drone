#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
import time
import json
import getpass

from water_drone.srv import RunPump
from water_drone.msg import WaterLevelSensorsData

PUMP_IN_DELAY = 35

class Pumps:
    def __init__(self) -> None:
        self.measure = False
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        rospy.init_node("pumps_control", anonymous=True)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config["rpi_pins"]["sensor_up"], GPIO.IN)
        GPIO.setup(self.config["rpi_pins"]["sensor_down"], GPIO.IN)
        GPIO.setup(self.config["rpi_pins"]["pump_in_main"], GPIO.OUT)
        GPIO.setup(self.config["rpi_pins"]["pump_out_main"], GPIO.OUT)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.HIGH)
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
        if self.config["extra_pumps"]:
            GPIO.setup(self.config["rpi_pins"]["pump1"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump1"], GPIO.HIGH)
            GPIO.setup(self.config["rpi_pins"]["pump2"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump2"], GPIO.HIGH)
            GPIO.setup(self.config["rpi_pins"]["pump3"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump3"], GPIO.HIGH)
            GPIO.setup(self.config["rpi_pins"]["pump4"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump4"], GPIO.HIGH)
            GPIO.setup(self.config["rpi_pins"]["pump5"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump5"], GPIO.HIGH)
            GPIO.setup(self.config["rpi_pins"]["pump6"], GPIO.OUT)
            GPIO.output(self.config["rpi_pins"]["pump6"], GPIO.HIGH)
        s = rospy.Service('run_pump', RunPump, self.handle_run_pump)
        self.publisher = rospy.Publisher("water_level_sensors", WaterLevelSensorsData, queue_size=10)

    def handle_run_pump(self, req):
        if req.main_pump:
            if req.pump_in:
                self.pump_in_main()
            else:
                self.pump_out_main()
        else:
            self.pump_in(req.number_of_pump)
        return True
    
    def pump_in_main(self):
        rospy.loginfo("Start pump in water")
        # time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.LOW)
        while GPIO.input(self.config["rpi_pins"]["sensor_up"]):
            time.sleep(0.5)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.HIGH)
        rospy.loginfo("Finished pump in water")
    
    def pump_out_main(self):
        rospy.loginfo("Start pump out water")
        # time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.LOW)
        while not GPIO.input(self.config["rpi_pins"]["sensor_down"]):
            time.sleep(0.5)
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
        rospy.loginfo("Finished pump out water")

    def pump_in(self, number_of_pump: int):
        if not self.config["extra_pumps"]:
            rospy.logerr("The node was initialised without extra pumps")
            return
        rospy.loginfo(f"Start pump in water in pump {number_of_pump}")
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.LOW)
        time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.HIGH)
        rospy.loginfo("Finished pump in water")

    def send_sensors_data(self):
        while not rospy.is_shutdown():
            data = WaterLevelSensorsData()
            data.low_sensor = GPIO.input(self.config["rpi_pins"]["sensor_down"])
            data.up_sensor = GPIO.input(self.config["rpi_pins"]["sensor_up"])
            self.publisher.publish(data)
            time.sleep(0.5)

Pumps().send_sensors_data()