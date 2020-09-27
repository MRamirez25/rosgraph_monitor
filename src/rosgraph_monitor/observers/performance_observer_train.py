from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from math import sqrt
import numpy as np
import rospy


class PerformanceObserverTrain(TopicObserver):
    def __init__(self, name):
        topics = [("/boxer_velocity_controller/odom", Odometry), ("/move_base/NavfnROS/plan", Path)]     # list of pairs
        self._v_max = 1 #m/s
        self._tc = 2 #seconds
        self._rate = 10 #Hz

        self._n = int(self._tc*self._rate)
        self._global_path_dis = np.zeros(self._n)
        self._direct_path_dis = np.zeros(self._n)
        self._x_path_prev = 0
        self._y_path_prev = 0
        self._x_robot_prev = 0

        self._pub_metric1 = rospy.Publisher('/metrics/performance2', Float64, queue_size=10)
        self._pub_metric2 = rospy.Publisher('/metrics/performance31', Float64, queue_size=10)
        self._pub_metric3 = rospy.Publisher('/metrics/performance32', Float64, queue_size=10)

        super(PerformanceObserverTrain, self).__init__(
            name, self._rate, topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        x_robot = msgs[0].pose.pose.position.x
        y_robot = msgs[0].pose.pose.position.y
        v_x = msgs[0].twist.twist.linear.x
        path_poses = msgs[1].poses


        # Search in path poses for the closest pose
        path_distance = 10 #init with a large value
        x_path = path_poses[0].pose.position.x
        y_path = path_poses[0].pose.position.y
        print("amount of poses:%s"%len(path_poses))
        for pose in path_poses:
            path_pose_distance = sqrt((pose.pose.position.x - x_robot)**2 + (pose.pose.position.y - y_robot)**2)
            if path_pose_distance < path_distance:
                path_distance = path_pose_distance
                x_path = pose.pose.position.x
                y_path = pose.pose.position.y
            # if path_pose_distance > (path_distance + 0.5):
            #     break # This means that the rest of the poses are further away, so stop searching

        # Calculate distance to previous point
        d_actual_path = sqrt((self._x_path_prev-x_path)**2 + (self._y_path_prev-y_path)**2 )
        d_global_path = sqrt((self._x_path_prev-x_path)**2 + (self._y_path_prev-y_path)**2 )
        d_direct_path = sqrt((self._x_robot_prev-x_robot)**2)

        # Reset if d > 1. This means that the position of the robot is reset
        if d_actual_path > 1.0:
            self._global_path_dis = np.zeros(self._n)
            self._direct_path_dis = np.zeros(self._n)
        # Update global_path_dis array with new distance, and remove the first distance
        else:
            self._global_path_dis = self._global_path_dis - self._global_path_dis[0]
            self._global_path_dis = np.delete(self._global_path_dis, 0)
            self._global_path_dis = np.append(self._global_path_dis,self._global_path_dis[-1]+d_global_path)

            self._direct_path_dis = self._direct_path_dis - self._direct_path_dis[0]
            self._direct_path_dis = np.delete(self._direct_path_dis, 0)
            self._direct_path_dis = np.append(self._direct_path_dis,self._direct_path_dis[-1]+d_direct_path)

        # Update previous robot position
        self._x_path_prev = x_path
        self._y_path_prev = y_path
        self._x_robot_prev = x_robot
        
        # Time to completion for global path
        t_path = self._global_path_dis[-1] / self._v_max
        t_path_dir = self._direct_path_dis[-1] / self._v_max

        # performance = 0.6 * (t_path/self._tc)**3
        performance = t_path/self._tc
        performance_dir = t_path_dir/self._tc

        performance2 = v_x/self._v_max

        print("performance:{0}".format(performance))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("performance2", str(performance2)))
        status_msg.values.append(
            KeyValue("performance31", str(performance)))
        status_msg.values.append(
            KeyValue("performance32", str(performance_dir)))
        status_msg.message = "QA status"

        return status_msg

    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric0 = status_msgs[0].values[0].value
            metric1 = status_msgs[0].values[1].value
            metric2 = status_msgs[0].values[2].value

            self._pub_metric1.publish(float(metric0))
            self._pub_metric2.publish(float(metric1))
            self._pub_metric3.publish(float(metric2))

            self._seq += 1
            self._rate.sleep()

class PerformanceObserverTEBTrain(PerformanceObserverTrain):
    def __init__(self, name):
        super(PerformanceObserverTEBTrain, self).__init__(name)

        # Only override the topics attribute
        self._topics = [("/boxer_velocity_controller/odom", Odometry), ("/move_base/TebLocalPlannerROS/global_plan", Path)]

class PerformanceObserverDWATrain(PerformanceObserverTrain):
    def __init__(self, name):
        super(PerformanceObserverDWATrain, self).__init__(name)

        # Only override the topics attribute
        self._topics = [("/boxer_velocity_controller/odom", Odometry), ("/move_base/DWAPlannerROS/global_plan", Path)]