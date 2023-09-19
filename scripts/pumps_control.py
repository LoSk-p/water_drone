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

class Pumps:
    def __init__(self) -> None:
        self.measure = False
        self.username = getpass.getuser()
        self.lat = None
        self.lon = None
        self.timestamp = None
        self.current_date = str(datetime.datetime.now().strftime("%Y_%m_%d"))
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/config.json", "r") as f:
            self.config = json.load(f)
        self.pumps_filename = f"/home/{self.username}/data/{self.current_date}/pumps/pumps-{time.time()}.json"
        rospy.init_node("pumps_control", anonymous=True)
        self.low_sensor_values = []
        self.up_sensor_values = []
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
        response = self.mavros_cmd(command=CommandCode.DO_PAUSE_CONTINUE, 
                                    param1=1 if command == "pause" else 0)

    def handle_run_pump(self, req):
        if req.main_pump:
            if req.pump_in:
                self.pump_in_main()
            else:
                self.pump_out_main()
        else:
            self.pump_in(req.number_of_pump)
        return True

    def check_low_sensor(self):
        """Return True if there is no water in low sensor"""
        self.low_sensor_values.append(GPIO.input(self.config["rpi_pins"]["sensor_down"]))
        if len(self.low_sensor_values) >= 5:
            self.low_sensor_values.pop(0)
        return all(self.low_sensor_values)

    def check_up_sensor(self) -> bool:
        """Return True if there is water in up sensor"""
        self.up_sensor_values.append(GPIO.input(self.config["rpi_pins"]["sensor_up"]))
        if len(self.up_sensor_values) >= 5:
            self.up_sensor_values.pop(0)
        return all([not val for val in self.up_sensor_values])
    
    def pump_in_main(self):
        rospy.loginfo("Start pump in water")
        # time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.LOW)
        while not self.check_up_sensor():
            time.sleep(0.5)
        GPIO.output(self.config["rpi_pins"]["pump_in_main"], GPIO.HIGH)
        rospy.loginfo("Finished pump in water")
    
    def pump_out_main(self):
        rospy.loginfo("Start pump out water")
        # time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.LOW)
        while not self.check_low_sensor():
            time.sleep(0.5)
        time.sleep(15)
        GPIO.output(self.config["rpi_pins"]["pump_out_main"], GPIO.HIGH)
        rospy.loginfo("Finished pump out water")

    def _create_new_pump_message(self, pump_number: int) -> NewPump:
        message = NewPump()
        message.lat = self.lat
        message.lon = self.lon
        message.timestamp = self.timestamp
        message.pump_number = pump_number
        return message

    def pump_in(self, number_of_pump: int):
        if not self.config["extra_pumps"]:
            rospy.logerr("The node was initialised without extra pumps")
            return
        rospy.loginfo(f"Start pump in water in pump {number_of_pump}")
        self.start_pause_mission("pause")
        with open(self.pumps_filename, "a") as f:
            json_data = json.dumps({"Lat": self.lat, "Lon": self.lon, "timestamp": self.timestamp, "pump_number": number_of_pump})
            f.write(f"{json_data}\n")
        self.new_pump_pub.publish(self._create_new_pump_message(number_of_pump))
        self.pump_in_main()
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.LOW)
        time.sleep(PUMP_IN_DELAY)
        GPIO.output(self.config["rpi_pins"][f"pump{number_of_pump}"], GPIO.HIGH)
        self.start_pause_mission("start")
        self.pump_out_main()
        rospy.loginfo("Finished pump in water")

    def send_sensors_data(self):
        while not rospy.is_shutdown():
            data = WaterLevelSensorsData()
            data.low_sensor = GPIO.input(self.config["rpi_pins"]["sensor_down"])
            data.up_sensor = GPIO.input(self.config["rpi_pins"]["sensor_up"])
            self.publisher.publish(data)
            time.sleep(0.5)

Pumps().send_sensors_data()