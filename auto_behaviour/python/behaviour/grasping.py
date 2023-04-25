import py_trees

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

class OpenGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name, hand_group,finger_val):
        super(OpenGripper,self).__init__(name)

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
        self.logger.info(f"Terminated with status {new_status}")

class CloseGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name, arm, gripper,pose):
        super(CloseGripper,self).__init__(name)

        self.arm = arm
        self.gripper = gripper
        self.pose = pose

    def initialise(self):
        return super().initialise()

    def update(self):
        return super().update()
    
    def terminate(self,new_status):
        self.logger.info(f"Terminated with status {new_status}")

        