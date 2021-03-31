#!/usr/bin/env python3 

import rospy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import RCIn

def RCOut_callback(data):
	with open("imu_rc", "a") as f:
		f.write("RCOut data:\n")
		f.write(str(data))
		f.write("\n")

def RCIn_callback(data):
	with open("imu_rc", "a") as f:
		f.write("RCIn data:\n")
		f.write(str(data))
		f.write("\n")

def Imu_callback(data):
	with open("imu_rc", "a") as f:
		f.write("Imu data:\n")
		f.write(str(data))
		f.write("\n")

def ImuRaw_callback(data):
	with open("imu_rc", "a") as f:
		f.write("Imu_raw data:\n")
		f.write(str(data))
		f.write("\n")

if __name__ == '__main__':
	rospy.init_node("write_rc")
	rospy.Subscriber("/mavros/imu/data_raw", Imu, ImuRaw_callback)
	rospy.Subscriber("/mavros/imu/data", Imu, Imu_callback)
	rospy.Subscriber("/mavros/rc/in", RCIn, RCIn_callback)
	rospy.Subscriber("/mavros/rc/out", RCOut, RCOut_callback)
	rospy.spin()
