from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from math import sqrt, pi, sin, cos
import rospy
import rospkg
import pickle
import numpy as np
from tf.transformations import euler_from_quaternion
# import timeit

class ObstacleDensityObserverTrain(TopicObserver):
    def __init__(self, name):
        # Topics to subscribe to
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid), ("/boxer_velocity_controller/odom", Odometry)]     # list of pairs

        rospack = rospkg.RosPack()
        path = rospack.get_path('rosgraph_monitor')
        filename = path + '/scripts/OD_windows'
        # print(filename)
        with open(filename) as file:
            self._ODw_i, self._yaws = pickle.load(file)
        
        # Update rate
        self._rate = 10
        # Topic to publish to, instead of /diagnostics
        # self._pub_metric1 = rospy.Publisher('/metrics/obstacle_density11', Float64, queue_size=10)
        # self._pub_metric2 = rospy.Publisher('/metrics/obstacle_density12', Float64, queue_size=10)
        # self._pub_metric3 = rospy.Publisher('/metrics/obstacle_density13', Float64, queue_size=10)
        self._pub_metric1 = rospy.Publisher('/metrics/obstacle_density21', Float64, queue_size=10)
        # self._pub_metric5 = rospy.Publisher('/metrics/obstacle_density22', Float64, queue_size=10)

        # Inherit __init__ from parent class
        super(ObstacleDensityObserverTrain, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()


        # Determine yaw angle of the robot
        quaternion = msgs[1].pose.pose.orientation
        yaw_odom = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
        yaw = self._yaws[np.argmin(np.absolute(np.subtract(self._yaws,yaw_odom)))]

        # Init
        obstacle_density = [0.0,0.0,0.0,0.0,0.0]
        
        for i in range(len(self._ODw_i[yaw])):
            length = len(self._ODw_i[yaw][i])
            for n in self._ODw_i[yaw][i]:
                # print(n)
                # Check if cell is occupied
                if msgs[0].data[n] != 0:
                    obstacle_density[i] += 1.0/length
            # print(obstacle_density[i])

        print("obstacle_density:{0}".format(obstacle_density))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        for i in range(len(self._ODw_i[yaw])):
            status_msg.values.append(
                KeyValue("obstacle_density%s"%i, str(obstacle_density[i])))
        status_msg.message = "QA status"

        # toc=timeit.default_timer()
        # print(toc-tic)

        return status_msg
        
    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric1 = status_msgs[0].values[0].value
            # metric2 = status_msgs[0].values[1].value
            # metric3 = status_msgs[0].values[2].value
            # metric4 = status_msgs[0].values[3].value
            # metric5 = status_msgs[0].values[4].value
            # metric6 = status_msgs[0].values[5].value

            self._pub_metric1.publish(float(metric1))
            # self._pub_metric2.publish(float(metric2))
            # self._pub_metric3.publish(float(metric3))
            # self._pub_metric4.publish(float(metric4))
            # self._pub_metric5.publish(float(metric5))
            # self._pub_metric6.publish(float(metric6))

            self._seq += 1
            self._rate.sleep()