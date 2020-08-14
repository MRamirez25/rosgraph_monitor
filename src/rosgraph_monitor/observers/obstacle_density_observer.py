from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from math import sqrt, pi
import rospy


class ObstacleDensityObserver(TopicObserver):
    def __init__(self, name):
        # Topics to subscribe to
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid)]     # list of pairs
        
        # Pars
        self._w = 1
        print(0)
        map_height = rospy.get_param("/move_base/local_costmap/height")
        print(1)
        map_width = rospy.get_param("/move_base/local_costmap/width")
        print(2)
        map_resolution = rospy.get_param("/move_base/local_costmap/resolution")
        print(3)
        map_n = int((map_width/map_resolution)*(map_height/map_resolution))
        print(4)
        n1 = int(0.5*(map_width-self._w)/map_resolution)
        print(5)
        map_n_width = int((map_width/map_resolution))
        print(6)
        n2 = map_n_width - n1
        print(7)
        self._map_i = []
        print(8)
        for i in range(n1,n2,1):
            for j in range(n1,n2,1):
                self._map_i.append(i*map_n_width + j)
        print(2)
        # Update rate
        self._rate = 10

        # Inherit __init__ from parent class
        super(ObstacleDensityObserver, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()
        
        # Determine Obstacle Density
        obstacle_density = 0.0
        for i in self._map_i:
            if msgs[0].data[i] != 0:
                obstacle_density += 1.0/len(self._map_i)

        print("obstacle_density:{0}".format(obstacle_density))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("obstacle_density", str(obstacle_density)))
        status_msg.message = "QA status"

        return status_msg
