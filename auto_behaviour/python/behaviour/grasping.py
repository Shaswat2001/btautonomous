import py_trees

class GoToPose(py_trees.behaviour.Behaviour):

    def __init__(self, name, arm_group,pose):
        super(GoToPose,self).__init__(name)

        self.arm_group = arm_group
        self.pose = pose
        self.status = "RUNNING"

    def initialise(self):

        self.status = self.arm_group.go(self.pose, wait=True)

    def update(self):

        if self.status == "RUNNING":
            return py_trees.common.Status.RUNNING
        elif self.status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):

        self.arm_group.stop()
        self.logger.info(f"Terminated with status {new_status}")

class PickObject(py_trees.behaviour.Behaviour):

    def __init__(self, name, arm, gripper,pose):
        super(PickObject,self).__init__(name)

        self.arm = arm
        self.gripper = gripper
        self.pose = pose
        

    def initialise(self):

        return super().initialise()

    def update(self):
        return super().update()
    
    def terminate(self,new_status):
        self.logger.info(f"Terminated with status {new_status}")
    
class PlaceObject(py_trees.behaviour.Behaviour):

    def __init__(self, name, arm, gripper,pose):
        super(PlaceObject,self).__init__(name)

        self.arm = arm
        self.gripper = gripper
        self.pose = pose

    def initialise(self):
        return super().initialise()

    def update(self):
        return super().update()
    
    def terminate(self,new_status):
        self.logger.info(f"Terminated with status {new_status}")

class MoveGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name, hand_group,finger_val):
        super(MoveGripper,self).__init__(name)

        self.hand_group = hand_group
        self.finger_val = finger_val
        self.status = "RUNNING"

    def initialise(self):

        joint_goal = self.hand_group.get_current_joint_values()
        joint_goal[0] = self.finger_val

        self.status = self.hand_group.go(joint_goal, wait=True)

    def update(self):

        if self.status == "RUNNING":
            return py_trees.common.Status.RUNNING
        elif self.status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):

        self.hand_group.stop()
        self.logger.info(f"Terminated with status {new_status}")
        