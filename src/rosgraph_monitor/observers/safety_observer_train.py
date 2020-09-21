from rosgraph_monitor.observer import TopicObserver
from safety_observer import SafetyObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from math import sqrt
import rospy


class SafetyObserverTrain(TopicObserver):
    def __init__(self, name):
        self._pub_metric0 = rospy.Publisher('/metrics/safety', Float64, queue_size=10)
        self._pub_metric1 = rospy.Publisher('/metrics/safety_wl', Float64, queue_size=10)

        topics = [("/boxer_velocity_controller/odom", Odometry), ("/front/scan", LaserScan)]     # list of pairs
        self._a_max = 1

        self._rate = 10

        self._robot_w = .55
        self._robot_l = 0.75

        super(SafetyObserverTrain, self).__init__(
            name, self._rate, topics)

    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric0 = status_msgs[0].values[0].value
            metric1 = status_msgs[0].values[1].value

            self._pub_metric0.publish(float(metric0))
            self._pub_metric1.publish(float(metric1))

            self._seq += 1
            self._rate.sleep()

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Forward velocity
        vel_x = msgs[0].twist.twist.linear.x
        # Braking distance, using the maximum acceleration
        d_brake = vel_x ** 2 / (2*self._a_max)

        # Find closest obstacle
        d_obstacle = min(msgs[1].ranges)
        # d_obs_n = np.argmin(msgs[1].ranges)

        # Find the angle and the x and y distances and subtract the robot dimensions
        # theta = msgs[1].angle_min + d_obs_n*msgs[1].angle_increment
        # d_obs_x = abs(d_obs*math.cos(theta)) - 0.5 * self._robot_l
        # d_obs_y = abs(d_obs*math.sin(theta)) - 0.5 * self._robot_w

        # d_obstacle = math.sqrt(d_obs_x**2 + d_obs_y**2)

        # Determine safety level
        safety = 1.0
        if (2*d_brake > d_obstacle):
            safety = d_obstacle/(2*d_brake)

        safety_wl = d_obstacle/(2*d_brake)

        #print ("d_break: {0}".format(d_break))
        #print("disntace:{0}".format(msgs[2].data))
        print("safety:{0}".format(safety))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("safety", str(safety)))
        status_msg.values.append(
            KeyValue("safety_wl", str(safety_wl)))
        status_msg.message = "QA status"

        return status_msg