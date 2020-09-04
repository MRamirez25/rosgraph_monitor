from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import sqrt
import pickle


class SafetyPredictionObserver(TopicObserver):
    def __init__(self, name, config):
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid), ("/front/scan", LaserScan)]  
           # list of pairs

        # Pars for Obstacle Density
        self._w = 1
        map_height = rospy.get_param("/move_base/local_costmap/height")
        map_width = rospy.get_param("/move_base/local_costmap/width")
        map_resolution = rospy.get_param("/move_base/local_costmap/resolution")
        map_n = int((map_width/map_resolution)*(map_height/map_resolution))
        n1 = int(0.5*(map_width-self._w)/map_resolution)
        map_n_width = int((map_width/map_resolution))
        n2 = map_n_width - n1
        self._map_i = []
        for i in range(n1,n2,1):
            for j in range(n1,n2,1):
                self._map_i.append(i*map_n_width + j)

        # Pars for Narrowness
        self._d_max = 1.0
        theta_range = 720
        step = (1.5 * pi)/theta_range
        self._nl = int(0.25 * pi / step)
        self._nr = int(1.25 * pi / step)

        # Update rate
        self._rate = 10

        self.config = config

        self.model = load_safety_model(self.config)

        super(SafetyPredictionObserver, self).__init__(
            name, self._rate, topics)

    def load_safety_model(config):
        pkl_filename = dir_models + config
        with open(pkl_filename, 'rb') as file:
            model = pickle.load(file)
        return model


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Determine Obstacle Density
        obstacle_density = 0.0
        for i in self._map_i:
            if msgs[0].data[i] != 0:
                obstacle_density += 1.0/len(self._map_i)

        # Spaces
        right_space = min(msgs[0].ranges[self._nr],self._d_max)
        left_space  = min(msgs[0].ranges[self._nl],self._d_max)
        
        # Narrowness
        narrowness = (left_space + right_space) / (2 * self._d_max)

        d_obstacle_density = obstacle_density
        d_narrowness = narrowness

        safety = self.model.predict([narrowness, d_narrowness, obstacle_density, d_obstacle_density])

        print("safety:{0}".format(safety))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("safety", str(safety)))
        status_msg.message = "QA update"

        return status_msg
