import py_trees
import rospy
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class IsMobileAtPose(py_trees.behaviour.Behaviour):

    def __init__(self, name,location):

        super(IsMobileAtPose,self).__init__(name)
        self.location = location
        self.timeout_duration = rospy.Duration(3.0)

    def initialise(self):
        self.current_pos = None
    
    def callback(self,msg_pose):

        self.current_pos = msg_pose.pose.pose
    
    def update(self):

        start_time = rospy.Time.now()
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        start_time = rospy.Time.now()

        while self.current_pos is None and (rospy.Time.now()-start_time < self.timeout_duration):
            rospy.sleep(0.05)

        if self.current_pos is not None and self.compare_pose(self.current_pos):
            self.logger.info("Robot at Desired Pose")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
    def compare_pose(self,p2):

        loc = self.location
        pos = p2.position
        if self.isClose(loc[0], pos.x) and self.isClose(loc[1], pos.y) and self.isClose(loc[2], pos.z):
            return True
        return False

    def isClose(self,v1,v2,tol = 0.5):

        return abs(v1-v2) <= tol
    
    def terminate(self,new_status):
        
        self.logger.info(f"Terminated check Mobile Pose : {new_status}")