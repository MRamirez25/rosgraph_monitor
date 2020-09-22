from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from math import sqrt
import rospy


class SafetyObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/boxer_velocity_controller/odom", Odometry), ("/front/scan", LaserScan)]     # list of pairs
        self._a_max = 1
        
        self._robot_w = 0.55
        self._robot_l = 0.75

        self._rate = 10

        super(SafetyObserver, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Forward velocity
        vel_x = msgs[0].twist.twist.linear.x
        # Braking distance, using the maximum acceleration
        d_brake = vel_x ** 2 / (2*self._a_max)

        # Find closest obstacle
        for n in range(len(msgs[1].ranges)):
            theta = msgs[1].angle_min + n*msgs[1].angle_increment
            d_robot = min(abs((self._robot_w/2)*sin(theta)),abs((self._robot_l/2)*cos(theta)))
            d_obstacles = msgs[1].ranges[n] - d_robot
        d_obstacle = min(d_obstacles)

        # Determine safety level
        safety = 1.0
        if (2*d_brake > d_obstacle):
            safety = d_obstacle/(2*d_brake)

        #print ("d_break: {0}".format(d_break))
        #print("disntace:{0}".format(msgs[2].data))
        print("safety:{0}".format(safety))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("safety", str(safety)))
        status_msg.message = "QA status"

        return status_msg

