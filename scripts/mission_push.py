import rospy

import json
import getpass
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList

from water_drone.srv import PushMission

class WaypointPushNode():
    def __init__(self):
        rospy.init_node("mission_push", anonymous=True)
        rospy.wait_for_service('/mavros/mission/push')
        self.push_mission_mavros = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        rospy.Service('push_mission', PushMission, self.push_mission)
        self.username = getpass.getuser()
        with open(f"/home/{self.username}/catkin_ws/src/water_drone/config/mission.json", "r") as f:
            self.mission = json.load(f)
        while not rospy.is_shutdown():
            pass

    def push_mission(self, req):
        # See /mavros/mavros_msgs/msg/Waypoint.msg and /mavros/mavros_msgs/msg/WaypointList.msg
        waypoints = [Waypoint() for i in range(len(self.mission[str(req.mission_number)])+2)]
        waypoint_number = 0
        for waipoint in waypoints:
            if waypoint_number == 0 or waypoint_number == (len(waypoints) - 1):
                waipoint.frame = 0
            else:
                waipoint.frame = 3 # frame 6 = Global relative altitude (to home position)
            if waypoint_number == (len(waypoints) - 1):
                waipoint.command = 20
            else:
                waipoint.command = 16 # command 16 = MAV_CMD_NAV_WAYPOINT -> go to waypoint
            waipoint.is_current = False
            waipoint.autocontinue = True
            # Hold time (how long does the vehicle stay at the waypoint in seconds):
            waipoint.param1 = float(0)
            # Accept Radius - how close you need to be to mark waypoint as reached in meters:
            waipoint.param2 = float(0)
            # 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for
            # counter-clockwise orbit. Allows trajectory control:
            waipoint.param3 = float(0)
            # Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode
            # (e.g. yaw towards next waypoint, yaw to home, etc.):
            waipoint.param4 = float(0)
            # Latitude:
            if waypoint_number == 0 or waypoint_number == (len(waypoints) - 1):
                waipoint.x_lat = float(0)
                waipoint.y_long = float(0)
            else:
                waipoint.x_lat = self.mission[str(req.mission_number)][waypoint_number - 1][0]
                waipoint.y_long = self.mission[str(req.mission_number)][waypoint_number - 1][1]
            # Altitude in meters:
            waipoint.z_alt = float(0)

            waypoint_number += 1

        response = self.push_mission_mavros(0, waypoints)
        rospy.loginfo(str(response))
        rospy.loginfo('Waypoint sent: %s .' %('successfully' if response.success else 'not successfully'))
        return response.success

WaypointPushNode()