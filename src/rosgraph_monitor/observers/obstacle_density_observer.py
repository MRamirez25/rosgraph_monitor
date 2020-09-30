from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from math import sqrt, pi
import rospy


class ObstacleDensityObserver(TopicObserver):
    def __init__(self, name):
        # Topics to subscribe to
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid), ("/odom", Odometry)]     # list of pairs

        rospack = rospkg.RosPack()
        path = rospack.get_path('rosgraph_monitor')
        filename = path + '/scripts/OD_windows'
        # print(filename)
        with open(filename) as file:
            self._ODw_n, self._yaws = pickle.load(file)
                
        # Update rate
        self._rate = 10

        # Inherit __init__ from parent class
        super(ObstacleDensityObserver, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        # Determine yaw angle of the robot
        quaternion = msgs[1].pose.pose.orientation
        yaw_odom = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
        yaw = self._yaws[np.argmin(np.absolute(np.subtract(self._yaws,yaw_odom)))]

        # Init
        obstacle_density = 0.0
        
        length = len(self._ODw_n[yaw])
        for n in self._ODw_n[yaw]:
            # Check if cell is occupied
            if msgs[0].data[n] != 0:
                obstacle_density[i] += 1.0/length

        print("obstacle_density:{0}".format(obstacle_density))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("obstacle_density", str(obstacle_density)))
        status_msg.message = "QA status"

        return status_msg
