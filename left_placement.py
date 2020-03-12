import argparse
import struct
import sys
import copy

import rospy
import rospkg

import tf

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface
import csv
import listener
import bezier_interpolation
import domino
import subprocess
import right_placement
import time
import simulation
import math

def main():

    mode_sim = True

    rospy.init_node("left_arm")
    left_pnp = right_placement.PickAndPlace('left')

    left = [tuple([float(x) for x in y.split(',')]) for y in str(sys.argv[1])[2:-2].split('), (')]

    #Setup variables
    table_height = 0.24
    hover = 0.1

    #Setup variables for height adjustment
    table_length = 160 #160cm
    table_reach = table_length/2 #Each arm only uses half the table
    relaive_table_length = 0.6 #Maximum y-distance travelled in gazebo by arm
    scale_factor = table_reach/relaive_table_length #Scaling factor
    scale_difference = 317.4 #Conversion between gazebo and millimetres
    #Table Heights
    raised_height = 79 #End with bricks under
    lowered_height = 72
    height_difference = raised_height - lowered_height
    incline_angle = float(math.tan(height_difference/table_length))

    #Variable for loading bricks
    movement_count = -1

    for coord in left:
        movement_count +=2
        print(coord[0],coord[1])
        adjusted_z = table_height + ((float(coord[0])+0.6)*scale_factor*incline_angle)/scale_difference
        domino.pickandplace('l',left_pnp,coord[1],coord[0],adjusted_z,0,math.pi,coord[2]+math.pi/2,hover,movement_count,mode_sim)

if __name__ == '__main__':
    sys.exit(main())

