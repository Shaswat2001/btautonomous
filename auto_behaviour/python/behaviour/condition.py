import py_trees
from nav_msgs.msg import Odometry

class IsMobileAtPose(py_trees.behaviour.Behaviour):

    def __init__(self, name,pose):

        super(IsMobileAtPose,self).__init__(name)
        self.pose = pose

    def setup(self):
        pass

    def initialise(self):
        return super().initialise()
    
    def update(self):
        return super().update()
    
    def terminate(self):
        return 0
