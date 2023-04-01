import py_trees
import rospy
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class IsMobileAtPose(py_trees.behaviour.Behaviour):

    def __init__(self, name,location):

        super(IsMobileAtPose,self).__init__(name)
        self.location = location
        self.current_pos = None

    def initialise(self):

        pass
    
    def callback(self,msg_pose):

        self.current_pos = msg_pose.pose.pose
    
    def update(self):

        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        
        if self.current_pos and self.compare_pose(self.location,self.current_pos):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
    def compare_pose(self,p1,p2):

        loc = self.location
        pos = p2.position
        if self.isClose(loc[0], pos.x) and self.isClose(loc[0], pos.y) and self.isClose(0, pos.z):
            o1 = loc[2]
            o2 = p2.orientation
            r2, p2, y2 = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
            if self.isClose(0, r2) and self.isClose(0, p2) and self.isClose(o1, y2):
                return True
        return False

    def isClose(self,v1,v2,tol = 0.1):

        return abs(v1-v2) <= tol
    
    def terminate(self,new_status):
        
        self.logger.info(f"Terminated check Mobile Pose : {new_status}")
