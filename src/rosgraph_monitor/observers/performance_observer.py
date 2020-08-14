from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from math import sqrt
import numpy as np


class PerformanceObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/odom", Odometry), ("/move_base/NavfnROS/plan", Path)]     # list of pairs
        self._v_max = 1 #m/s
        self._tc = 2 #seconds
        self._rate = 10 #Hz

        self._n = int(self._tc*self._rate)
        self._global_path_dis = np.zeros(self._n)
        self._x_path_prev = 0
        self._y_path_prev = 0

        super(PerformanceObserver, self).__init__(
            name, self._rate, topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        x_robot = msgs[0].pose.pose.position.x
        y_robot = msgs[0].pose.pose.position.y
        path_poses = msgs[1].poses


        # Search in path poses for the closest pose
        path_distance = 10 #init with a large value
        x_path = path_poses[0].pose.position.x
        y_path = path_poses[0].pose.position.y
        for pose in path_poses:
            path_pose_distance = sqrt((pose.pose.position.x - x_robot)**2 + (pose.pose.position.y-y_robot)**2)
            if path_pose_distance < path_distance:
                path_distance = path_pose_distance
                x_path = pose.pose.position.x
                y_path = pose.pose.position.y
            if path_pose_distance > (path_distance + 0.5):
                break # This means that the rest of the poses are further away, so stop searching

        # Calculate distance to previous point
        d_actual_path = sqrt((self._x_path_prev-x_path)**2 + (self._y_path_prev-y_path)**2 )
        d_global_path = sqrt((self._x_path_prev-x_path)**2 + (self._y_path_prev-y_path)**2 )

        # Reset if d > 1. This means that the position of the robot is reset
        if d_actual_path > 1.0:
            self._global_path_dis = np.zeros(self._n)
        # Update global_path_dis array with new distance, and remove the first distance
        else:
            self._global_path_dis = self._global_path_dis - self._global_path_dis[0]
            self._global_path_dis = np.delete(self._global_path_dis, 0)
            self._global_path_dis = np.append(self._global_path_dis,self._global_path_dis[-1]+d_global_path)

        # Update previous robot position
        self._x_path_prev = x_path
        self._y_path_prev = y_path
        
        # Time to completion for global path
        t_path = self._global_path_dis[-1] / self._v_max

        performance = 0.6 * (t_path/self._tc)**3


        print("performance:{0}".format(performance))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("performance", str(performance)))
        status_msg.message = "QA status"

        return status_msg

class PerformanceObserverTEB(PerformanceObserver):
    def __init__(self, name):
        super(PerformanceObserverTEB, self).__init__(name)

        # Only override the topics attribute
        self._topics = [("/odom", Odometry), ("/move_base/TebLocalPlannerROS/global_plan", Path)]

class PerformanceObserverDWA(PerformanceObserver):
    def __init__(self, name):
        super(PerformanceObserverDWA, self).__init__(name)

        # Only override the topics attribute
        self._topics = [("/odom", Odometry), ("/move_base/DWAPlannerROS/global_plan", Path)]