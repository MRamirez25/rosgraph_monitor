from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
from math import sqrt, pi
import rospy

class NarrownessObserverTrain(TopicObserver):
    def __init__(self, name):# Topics to subscribe to
        topics = [("/front/scan", LaserScan)]     # list of pairs

        # Pars
        self._d_max = [1.0,2.0,3.0,4.0]
        theta_range = 720
        step = (1.5 * pi)/theta_range
        self._nl = int(0.25 * pi / step)
        self._nr = int(1.25 * pi / step)
        self._r_width = 0.55

        # Update rate
        self._rate = 10

        self._pub_metric1 = rospy.Publisher('/metrics/narrowness1', Float64, queue_size=10)
        self._pub_metric2 = rospy.Publisher('/metrics/narrowness2', Float64, queue_size=10)
        self._pub_metric3 = rospy.Publisher('/metrics/narrowness3', Float64, queue_size=10)
        self._pub_metric4 = rospy.Publisher('/metrics/narrowness4', Float64, queue_size=10)

        # Inherit __init__ from parent class
        super(NarrownessObserverTrain, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Spaces
        right_space = min(msgs[0].ranges[self._nr],self._d_max)
        left_space  = min(msgs[0].ranges[self._nl],self._d_max)
        
        # Narrowness
        narrowness = []
        for i in range(len(self._d_max)):
            right_space = min(msgs[0].ranges[self._nr],self._d_max[i])
            left_space  = min(msgs[0].ranges[self._nl],self._d_max[i])
            narrowness.append((left_space + right_space) / (self._r_width))

        print("narrowness:{0}".format(narrowness[3]))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        for i in range(len(narrowness)):
            status_msg.values.append(
                KeyValue("narrowness%s"%i, str(narrowness[i])))
        status_msg.message = "QA status"

        return status_msg

    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric1 = status_msgs[0].values[0].value
            metric2 = status_msgs[0].values[1].value
            metric3 = status_msgs[0].values[2].value
            metric4 = status_msgs[0].values[3].value
            # metric5 = status_msgs[0].values[4].value
            # metric5 = status_msgs[0].values[5].value

            self._pub_metric1.publish(float(metric1))
            self._pub_metric2.publish(float(metric2))
            self._pub_metric3.publish(float(metric3))
            self._pub_metric4.publish(float(metric4))
            # self._pub_metric5.publish(float(metric5))
            # self._pub_metric6.publish(float(metric6))

            self._seq += 1
            self._rate.sleep()
