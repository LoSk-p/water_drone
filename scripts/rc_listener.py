import rospy
from water_drone.srv import RunPump
from std_srvs.srv import Empty
from mavros_msgs.msg import RCIn

CHANNEL_START_MEASURE = 4
CHANNEL_PUMP_IN_BOTTLE = 7
START_MEASURE_PWM = 2000
PUMP_IN_BOTTLE_PWM = 1500

class RCListener:
    def __init__(self) -> None:
        rospy.init_node("rc_listener", anonymous=True)
        rospy.wait_for_service('run_pump')
        self.run_pump_service = rospy.ServiceProxy('run_pump', RunPump)
        rospy.wait_for_service('start_measure')
        self.start_measure_service = rospy.ServiceProxy('start_measure', Empty)
        self.pumps_queue = [1, 4, 2, 5, 3, 6]
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_handle)
        self.rc_start_measure_state = 0
        self.rc_pump_in_bottle_state = 0
        rospy.loginfo("RC Listener is ready")
        while not rospy.is_shutdown():
            pass

    def rc_in_handle(self, data):
        if len(data.channels) > 0:
            if data.channels[CHANNEL_START_MEASURE] != self.rc_start_measure_state:
                self.rc_start_measure_state = data.channels[CHANNEL_START_MEASURE]
                if self.rc_start_measure_state == START_MEASURE_PWM:
                    rospy.loginfo(f"Start measure on RC signal, channels: {data.channels}")
                    self.start_measure_service()
            if data.channels[CHANNEL_PUMP_IN_BOTTLE] != self.rc_pump_in_bottle_state:
                self.rc_pump_in_bottle_state = data.channels[CHANNEL_PUMP_IN_BOTTLE]
                if self.rc_pump_in_bottle_state == PUMP_IN_BOTTLE_PWM:
                    rospy.loginfo(f"Start choose bottle on RC signal, channels: {data.channels}")
                    res = False
                    while not res:
                        rospy.loginfo(f"Start pump in water in bottle {self.pumps_queue[0]}")
                        res = self.run_pump_service(main_pump=0, pump_in=0, number_of_pump=self.pumps_queue[0])
                        rospy.loginfo(f"Water in bottle {self.pumps_queue[0]} pumped in with res: {res}")
                        self.pumps_queue.pop(0)

RCListener()