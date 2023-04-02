#!/usr/bin/env python3

from xml.dom import minidom
import rospy
import py_trees
import py_trees_ros
import yaml
from yaml.loader import SafeLoader
from behaviour import navigation
from behaviour import condition
from behaviour import vision

def create_behaviour_tree_from_xml(xml_file):

    pass

if __name__ == "__main__":

    rospy.init_node('automate_behaviour')
    # behaviour_location = rospy.get_param('bt_location')
    # file = minidom.parse(behaviour_location)

    block_location = rospy.get_param('block_locations')

    with open(block_location) as f:
        locations = yaml.load(f, Loader=SafeLoader)

    root = py_trees.composites.Sequence(name = "rootSequence")
    child = py_trees.composites.Selector(name="child")

    child.add_children([condition.IsMobileAtPose(name = "CheckMobilePose",location =locations["location2"]),
                        navigation.GetMobileToPose(name = "GetToPose",location=locations["location2"])])
    
    root.add_children([child,vision.LookForColoredObject(name="computer_vision",color="green")])

    ros_py_tree = py_trees_ros.trees.BehaviourTree(root)

    done = False
    while not rospy.is_shutdown() and not done:

        ros_py_tree.tick()

        if ros_py_tree.root.status == py_trees.common.Status.SUCCESS:
            print("Behavior tree succeeded")
            done = True
        elif ros_py_tree.root.status == py_trees.common.Status.FAILURE:
            print("Behavior tree failed.")
            done = True
        rospy.sleep(0.5)
    rospy.spin()


    # bt_tree = create_behaviour_tree_from_xml(xml_file = file)