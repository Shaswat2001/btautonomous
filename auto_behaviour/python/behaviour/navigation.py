import py_trees
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class GetMobileToPose(py_trees.behaviour.Behaviour):

    def __init__(self, name,location):
        super(GetMobileToPose,self).__init__(name)

        self.location = location
        self.blackboard = py_trees.blackboard.Blackboard()

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

    def initialise(self):

        target = self.blackboard.get("target_location")

        if target is not None:
            self.location = target

        goal = self.convert_location_to_move_base(self.location)
        self.client.send_goal(goal)
        rospy.sleep(0.5)
    
    def update(self):
        
        status = self.client.get_state()

        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        
        elif status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        
        return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):
        
        self.logger.info(f"Terminated with status {new_status}")
        self.blackboard.set("target_pose", None)

    def convert_location_to_move_base(self,location):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location[0]
        goal.target_pose.pose.position.y = location[1]
        goal.target_pose.pose.position.z = location[2]

        quaternion = quaternion_from_euler(0,0,location[3])

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal

class PickObject(py_trees.behaviour.Behaviour):

    def __init__(self, name, arm, gripper,pose):
        super(PickObject,self).__init__(name)

        self.arm = arm
        self.gripper = gripper
        self.pose = pose

    def setup(self):
        pass

    def initialise(self):
        return super().initialise()
    
    def open_gripper(self):
        pass

    def update(self):
        return super().update()
    
    def terminate(self,new_status):
        self.logger.info(f"Terminated with status {new_status}")
    
class PlaceObject(py_trees.behaviour.Behaviour):

    pass