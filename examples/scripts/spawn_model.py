#!/usr/bin/env python3

import rospy
import random
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from yaml.loader import SafeLoader

def create_pos_from_location(bl_location):

    p = Pose()
    p.position.x = bl_location[0]
    p.position.y = bl_location[1]
    p.position.z = 0

    return p

if __name__ == "__main__":

    colors = ['red','green','blue']
    rospy.init_node('spawn_model',anonymous=True)

    block_location = rospy.get_param('block_locations')

    with open(block_location) as f:
        locations = yaml.load(f, Loader=SafeLoader)

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for num,location in enumerate(locations.keys()):
        color = random.choice(colors)
        spawn_model_client(
            model_name='block_'+str(num),
            model_xml=open('/home/shaswat/btautonomous/src/examples/model/blocks/'+color+'_block.sdf', 'r').read(),
            robot_namespace='/',
            initial_pose=create_pos_from_location(locations[location]),
            reference_frame='map'
        )

