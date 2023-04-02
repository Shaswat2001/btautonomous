#!/usr/bin/env python3

import rospy
import random
import yaml
import math
import moveit_commander
from tf.transformations import quaternion_from_euler

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from yaml.loader import SafeLoader

block_pos_offset = 0.6
FRAME_ID = 'map'
dimensions = {
    "table":[0.2,0.4,0.4],
    "block":[0.05,0.05,0.05]
}
def create_pos_from_location(bl_location):

    p = Pose()
    p.position.x = bl_location[0]
    p.position.y = bl_location[1] 
    p.position.z = bl_location[2]
    quaternion = quaternion_from_euler(0,0,bl_location[3])

    p.orientation.x = quaternion[0]
    p.orientation.y = quaternion[1]
    p.orientation.z = quaternion[2]
    p.orientation.w = quaternion[3]

    return p

def create_collision_object(id, dimensions, location):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object.primitive_poses = [create_pos_from_location(location)]
    object.operation = object.ADD
    return object

if __name__ == "__main__":

    rospy.init_node('spawn_model',anonymous=True)
    SCENE = moveit_commander.PlanningSceneInterface()
    objects = rospy.get_param("objects")
    object_location = rospy.get_param('object_locations')
    FRAME_ID = rospy.get_param('frame_id')

    with open(object_location) as f:
        obj_locations = yaml.load(f, Loader=SafeLoader)


    if "table" in objects:

        locations = obj_locations["table"]
        for num,location in enumerate(locations.keys()):
            id = "table"+str(num)
            object = create_collision_object(id, dimensions["table"], locations[location])
            SCENE.add_object(object)

    if "blocks" in objects:

        colors = ['red','green','blue']
        
        locations = obj_locations["block"]
        for num,location in enumerate(locations.keys()):
            id = "block"+str(num)
            color = random.choice(colors)
            object = create_collision_object(id, dimensions["block"], locations[location])
            SCENE.add_object(object)

