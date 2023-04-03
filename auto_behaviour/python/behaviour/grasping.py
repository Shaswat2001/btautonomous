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

    def __init__(self, name, arm, gripper,pose):
        super(OpenGripper,self).__init__(name)

        self.arm = arm
        self.gripper = gripper
        self.pose = pose

    def initialise(self):
        return super().initialise()

    def update(self):
        return super().update()
    
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

        