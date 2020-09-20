from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from math import sqrt, pi
import rospy


class ObstacleDensityObserverTrain(TopicObserver):
    def __init__(self, name):
        # Topics to subscribe to
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid)]     # list of pairs
        
        # Pars
        self._w = 1
        # map_height = rospy.get_param("/move_base/local_costmap/height")
        # map_width = rospy.get_param("/move_base/local_costmap/width")
        # map_resolution = rospy.get_param("/move_base/local_costmap/resolution")

        # To determine the costmap cells for the OD window
        map_height = 5          # Default in ROS
        map_width = 5           # Default in ROS
        map_resolution = 0.01   # Default in ROS
        map_n_width = int((map_width/map_resolution))

        # Start and end elements
        n1_y = int(0.5*(map_width-self._w)/map_resolution)
        n2_y = map_n_width - n1_y

        n1_x = int(0.5*map_width/map_resolution)
        n2_x = int(n1_x+(self._w/map_resolution))

        self._map_i = []
        for i in range(n1_y,n2_y,1):
            for j in range(n1_x,n2_x,1):
                self._map_i.append(i*map_n_width + j)

        # Update rate
        self._rate = 10
        # Publish topic
        self._pub_metric = rospy.Publisher('/metrics/obstacle_density', Float64, queue_size=10)

        # Inherit __init__ from parent class
        super(ObstacleDensityObserverTrain, self).__init__(
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
        
    # Override this function to publish on a separate topic
    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            status_msgs = self.generate_diagnostics()
            
            # print(status_msgs)
            metric = status_msgs[0].values[0].value

            self._pub_metric.publish(float(metric))

            self._seq += 1
            self._rate.sleep()