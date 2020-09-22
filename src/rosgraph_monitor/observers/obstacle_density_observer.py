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
        topics = [("/move_base/local_costmap/costmap", OccupancyGrid), ("/boxer_velocity_controller/odom", Odometry)]     # list of pairs
        
        # Pars ODwindow
        self._ODw_center = [0.5, 0.0]
        self._ODw_width = 1.0

        # Pars costmap (Hardcoded for now)
        self._costmap_width = 5.0
        self._costmap_res = 0.01
        self._costmap_width_n = int((self._costmap_width/self._costmap_res))
                
        # Update rate
        self._rate = 10

        # Inherit __init__ from parent class
        super(ObstacleDensityObserver, self).__init__(
            name, self._rate, topics)


    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()
        
        # Determine yaw angle of the robot
        quaternion = msgs[1].pose.pose.orientation
        yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
        
        # Init
        obstacle_density = 0.0
        nc=0 # number of cells in OD window

        # Loop over all the local costmap grid cells
        for n in range(len(msgs[0].data)):
            # Determine x and y coordinates of this cell
            x = (n%self._costmap_width_n)*self._costmap_res - self._costmap_width/2
            y = int(n/self._costmap_width_n)*self._costmap_res - self._costmap_width/2
            # Rotate to robot local coordinate system
            xa = x*cos(yaw)+y*sin(yaw)
            ya = -x*sin(yaw)+y*cos(yaw)
            # Check if this cell is in the OD window
            if xa < self._ODw_center[0]+(self._ODw_width/2) and xa > self._ODw_center[0]+(-self._ODw_width/2) and ya < self._ODw_center[1]+(self._ODw_width/2) and ya > self._ODw_center[1]-(self._ODw_width/2):
                # Check if cell is occupied
                if msgs[0].data[n] != 0:
                    obstacle_density += 1.0
                nc += 1
        # Normalize OD with the number of cells in the OD window
        if nc > 0:
            obstacle_density = obstacle_density/nc

        print("obstacle_density:{0}".format(obstacle_density))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("obstacle_density", str(obstacle_density)))
        status_msg.message = "QA status"

        return status_msg
