#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('trajectory_msgs')

import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *

rospy.init_node('lookat', anonymous=True)
client = actionlib.SimpleActionClient('/head_traj_controller/joint_trajectory_action', JointTrajectoryAction)
client.wait_for_server()

head_positions = (0,0)

def panTilt(positions):
    g = JointTrajectoryGoal()
    g.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
    g.trajectory.points = [JointTrajectoryPoint()]
    g.trajectory.points[0].positions = positions;
    g.trajectory.points[0].velocities = [0, 0];
    # pan(-:right, +:left), tilt(neg:look up, pos:look down)
    g.trajectory.points[0].time_from_start = rospy.Duration.from_sec(0.8)
    client.send_goal(g)
    #client.wait_for_result()

from math import atan2, pi
def pos_callback(data):
    azim = atan2(data.y, data.x)/pi*180
    if azim > 100:
        azim = 100 + (azim - 100) * 0.2
    if azim < -100:
        azim = -100 - (azim + 100) * 0.2
    azim = azim/180*pi
    pos = list(head_positions)
    pos[0] = azim
    pos[1] = 0.1
    panTilt(pos)

def recv_joint_states(data):
    global head_positions
    head_positions = data.actual.positions

rospy.Subscriber("/tracker/nearest", Point, pos_callback)
rospy.Subscriber('/head_traj_controller/state', JointTrajectoryControllerState, recv_joint_states)
rospy.spin()
