#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

def callback(data):
	gps_data = "latitude: " + str(data.latitude) + "; longitude: " + str(data.longitude) + "\n"
	print(gps_data)
	file.write(gps_data)
def listener():
	rospy.init_node("get_data", anonymous=True)
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback)
	rospy.spin()
if __name__ == '__main__':
	file = open("data", 'w')
	listener()
