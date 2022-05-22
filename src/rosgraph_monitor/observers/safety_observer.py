from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from math import sqrt, sin, cos, pi, atan
import rospy
import numpy as np


class SafetyObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/mobile_base_controller/odom", Odometry), ("/scan_filtered", LaserScan)]     # list of pairs
        self._a_max = 0.5

        self._rate = 10

        self._robot_y = 0.275
        self._robot_x = 0.20

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
        d_obstacles=[]
        thetas = []
        ds_robot = []
        for n in range(len(msgs[1].ranges)):
            theta = msgs[1].angle_min + n*msgs[1].angle_increment
            thetas.append(theta)
            if theta > -atan(self._robot_y/self._robot_x) and theta < atan(self._robot_y/self._robot_x):
                d_robot = abs((self._robot_x)*cos(theta))
            else:
                d_robot = abs((self._robot_y)*sin(theta))
            ds_robot.append(d_robot)
            d_obstacles.append(msgs[1].ranges[n] - d_robot)
        d_obstacle = min(d_obstacles)

        indx = np.argmin(d_obstacles)
        print(d_obstacle)

        # Determine safety level
        safety = 0.0
        if (d_brake > d_obstacle):
            safety = d_obstacle/(d_brake)
            safety = 1 - safety
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

