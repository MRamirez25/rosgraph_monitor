from rosgraph_monitor.observer import TopicObserver
from safety_observer import SafetyObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from math import sqrt, pi, sin, cos, atan
import rospy
import numpy as np


class SafetyObserverTrain(TopicObserver):
    def __init__(self, name):
        self._pub_metric0 = rospy.Publisher('/metrics/safety', Float64, queue_size=10)
        self._pub_metric1 = rospy.Publisher('/metrics/safety_old1', Float64, queue_size=10)
        self._pub_metric2 = rospy.Publisher('/metrics/safety_old2', Float64, queue_size=10)

        topics = [("/boxer_velocity_controller/odom", Odometry), ("/front/scan", LaserScan)]     # list of pairs
        self._a_max = 0.5

        self._rate = 10

        self._robot_y = 0.275
        self._robot_x = 0.20

        super(SafetyObserverTrain, self).__init__(
            name, self._rate, topics)

    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric0 = status_msgs[0].values[0].value
            metric1 = status_msgs[0].values[1].value
            metric2 = status_msgs[0].values[2].value

            self._pub_metric0.publish(float(metric0))
            self._pub_metric1.publish(float(metric1))
            self._pub_metric2.publish(float(metric2))

            self._seq += 1
            self._rate.sleep()

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
        safety = 1.0
        if (d_brake > d_obstacle):
            safety = d_obstacle/(d_brake)

        # Determine safety_old level
        d_obs = min(msgs[1].ranges)
        safety_old1 = 1.0
        if (d_brake > d_obs):
            safety_old1 = d_obs/(d_brake)
        safety_old2 = 1.0
        if (2*d_brake > d_obs):
            safety_old2 = d_obs/(2*d_brake)
        

        #print ("d_break: {0}".format(d_break))
        #print("disntace:{0}".format(msgs[2].data))
        print("safety:{0}".format(safety))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("safety", str(safety)))
        status_msg.values.append(
            KeyValue("safety_old1", str(safety_old1)))
        status_msg.values.append(
            KeyValue("safety_old2", str(safety_old2)))
        status_msg.message = "QA status"

        return status_msg