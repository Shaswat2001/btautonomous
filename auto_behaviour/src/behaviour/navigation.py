import py_trees

class GetMobileToPose(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super().__init__(name)

    def setup(self):
        pass

    def initialise(self):
        return super().initialise()
    
    def update(self):
        return super().update()
    
    def terminate(self):
        return 0

class IsMobileAtPose(py_trees.behaviour.Behaviour):

    pass