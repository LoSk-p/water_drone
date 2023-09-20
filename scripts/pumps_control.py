#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
import time
import json
import getpass
from sensor_msgs.msg import NavSatFix
import datetime
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode

from water_drone.srv import RunPump
from water_drone.msg import WaterLevelSensorsData, NewPump

PUMP_IN_DELAY = 30
MAIN_PUMP_IN_DELAY = 20 # Delay after water in up sensor
MAIN_PUMP_OUT_DELAY = 15 # Delay after no water in low sensor

class Pumps:
    def __init__(self) -> None:
        self.username = getpass.getuser()
        self.lat = None
        self.lon = None
        self.timestamp = None
        self.extra_pump_work = False
        self.main_pump_out_work = False
        self.main_pump_in_work = False
        self.measure = False
        self.mission_paused = False
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.pumps_filename = f"/home/{self.username}/data/{self.current_date}/pumps/pumps-{time.time()}.json"
        rospy.init_node("pumps_control", anonymous=True)
        self.low_sensor_values = []
        self.up_sensor_values = []
        self.full_pumps = []
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
        self.new_pump_pub = rospy.Publisher("new_pump", NewPump, queue_size=10)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.callback_gps)
        rospy.wait_for_service('/mavros/cmd/command')
        self.mavros_cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.loginfo("Pumps control is ready")

    def callback_gps(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.timestamp = data.header.stamp.secs

    def start_pause_mission(self, command: str):
        if (self.main_pump_in_work or self.measure or self.extra_pump_work) and command == "start":
            rospy.loginfo(f"Don't start mission while: main pump in - {self.main_pump_in_work}, measure - {self.measure}, extra pump - {self.extra_pump_work}")
            return
        if (command == "pause") == self.mission_paused:
            rospy.loginfo(f"Mission is already {command}")
            return
        rospy.loginfo(f"Mission {command}")
        response = self.mavros_cmd(command=CommandCode.DO_PAUSE_CONTINUE, 
                                    param1=1 if command == "pause" else 0)
        self.mission_paused = (command == "pause")

    def handle_run_pump(self, req):
        if req.main_pump:
            if req.pump_in:
                self.start_pause_mission("pause")
                self.pump_in_main()
                self.measure = True
            else:
                self.measure = False
                self.start_pause_mission("start")
                self.pump_out_main()
        else:
            self.pump_in(req.number_of_pump)
        return True

    def check_low_sensor(self):
        """Return True if there is no water in low sensor"""
        return all(self.low_sensor_values)

    def check_up_sensor(self) -> bool:
        """Return True if there is water in up sensor"""
        return all([not val for val in self.up_sensor_values])
    
    def pump_in_main(self):
        if self.check_up_sensor():
            rospy.loginfo("Water is full")
            return
        self.main_pump_in_work = True
        rospy.loginfo("Start pump in water")
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.LOW)
        while not self.check_up_sensor():
            time.sleep(0.5)   
        time.sleep(MAIN_PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.HIGH)
        self.main_pump_in_work = False
        rospy.loginfo("Finished pump in water")
    
    def pump_out_main(self):
        if self.main_pump_in_work or self.measure or self.extra_pump_work:
            rospy.loginfo(f"Not pump out water because of: main pump in - {self.main_pump_in_work}, measure - {self.measure}, extra pump - {self.extra_pump_work}")
            return
        self.main_pump_out_work = True
        rospy.loginfo("Start pump out water")
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.LOW)
        while not self.check_low_sensor():
            if self.main_pump_in_work:
                rospy.loginfo("Stop pump out water")
                self.main_pump_out_work = False
                GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
                return
            time.sleep(0.5)
        start_time = time.time()
        while (time.time() - start_time) < MAIN_PUMP_OUT_DELAY:
            if self.main_pump_in_work:
                rospy.loginfo("Stop pump out water")
                self.main_pump_out_work = False
                GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
                return
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
        self.main_pump_out_work = False
        rospy.loginfo("Finished pump out water")

    def _create_new_pump_message(self, pump_number: int) -> NewPump:
        message = NewPump()
        message.lat = str(self.lat)
        message.lon = str(self.lon)
        message.timestamp = str(self.timestamp)
        message.pump_number = str(pump_number)
        return message

    def pump_in(self, number_of_pump: int):
        if not self.config["extra_pumps"]:
            rospy.logerr("The node was initialised without extra pumps")
            return
        if number_of_pump in self.full_pumps:
            rospy.loginfo(f"Bottle number {number_of_pump} as already full")
            return
        rospy.loginfo(f"Request to pump in water in pump {number_of_pump}")
        self.full_pumps.append(number_of_pump)
        self.start_pause_mission("pause")
        with open(self.pumps_filename, "a") as f:
            json_data = json.dumps({"Lat": self.lat, "Lon": self.lon, "timestamp": self.timestamp, "pump_number": number_of_pump})
            f.write(f"{json_data}\n")
        self.new_pump_pub.publish(self._create_new_pump_message(number_of_pump))
        self.pump_in_main()
        self.extra_pump_work = True
        rospy.loginfo(f"Start pump in water in bottle {number_of_pump}")
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.LOW)
        time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.HIGH)
        rospy.loginfo(f"Finished pump in water in bottle {number_of_pump}")
        self.extra_pump_work = False
        self.start_pause_mission("start")
        self.pump_out_main()
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.LOW)
        time.sleep(1.5)
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.HIGH)
        rospy.loginfo(f"Finished pump in water in pump {number_of_pump}")

    def send_sensors_data(self):
        time_to_pump_in_mes = 0
        while not rospy.is_shutdown():
            data = WaterLevelSensorsData()
            data.low_sensor = GPIO.input(self.config["rpi_pins"]["sensor_down"])
            data.up_sensor = GPIO.input(self.config["rpi_pins"]["sensor_up"])
            self.publisher.publish(data)
            self.up_sensor_values.append(GPIO.input(self.config["rpi_pins"]["sensor_up"]))
            if len(self.up_sensor_values) >= 5:
                self.up_sensor_values.pop(0)
            self.low_sensor_values.append(GPIO.input(self.config["rpi_pins"]["sensor_down"]))
            if len(self.low_sensor_values) >= 5:
                self.low_sensor_values.pop(0)
            if self.measure:
                if data.up_sensor:
                    time_to_pump_in_mes = time.time()
                    rospy.loginfo("Pump in water while measure")
                    GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.LOW)
                if time_to_pump_in_mes != 0 and (time.time() - time_to_pump_in_mes) > MAIN_PUMP_IN_DELAY:
                    time_to_pump_in_mes = 0
                    GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.HIGH)
            time.sleep(0.5)

Pumps().send_sensors_data()