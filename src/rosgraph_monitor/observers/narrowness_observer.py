from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
from math import sqrt, pi
import rospy

class NarrownessObserver(TopicObserver):
    def __init__(self, name):# Topics to subscribe to
        topics = [("/scan_raw", LaserScan)]     # list of pairs

        # Pars
        self._d_max = 1.0
        theta_range = 720
        step = (1.5 * pi)/theta_range
        self._nl = int(0.25 * pi / step)
        self._nr = int(1.25 * pi / step)
        self._r_width = 0.55

        # Update rate
        self._rate = 10

        # Inherit __init__ from parent class
        print(2)
        super(NarrownessObserver, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Spaces
        right_space = min(msgs[0].ranges[self._nr],self._d_max)
        left_space  = min(msgs[0].ranges[self._nl],self._d_max)
        
        # Narrowness
        narrowness = (left_space + right_space) / (self._r_width)

        print("narrowness:{0}".format(narrowness))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("narrowness", str(narrowness)))
        status_msg.message = "EM status"

        return status_msg
