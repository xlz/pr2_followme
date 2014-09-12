#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('std_srvs')

import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *
from std_srvs.srv import Empty
from std_msgs.msg import String

import sys
rospy.init_node('follow', anonymous=True)

focus = rospy.ServiceProxy('/tracker/focus', Empty)
defocus = rospy.ServiceProxy('/tracker/defocus', Empty)
rospy.wait_for_service('/tracker/focus')
rospy.wait_for_service('/tracker/defocus')

defocus()

dictate = rospy.ServiceProxy('/pcsdk/recog/dictate', Empty)
print >>sys.stderr, 'waiting for pcsdk'
rospy.wait_for_service('/pcsdk/recog/dictate')
dictate()
print >>sys.stderr, 'pcsdk dictating'

frame = rospy.get_param('~frame', '/odom_combined')
moveBaseTopic = '/move_base_local'
if frame == '/map':
    moveBaseTopic = '/move_base'
print >>sys.stderr, 'frame is', frame
clientHead = actionlib.SimpleActionClient('/head_traj_controller/joint_trajectory_action', JointTrajectoryAction)
clientLook = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
clientMove = actionlib.SimpleActionClient(moveBaseTopic, MoveBaseAction)
clientHead.wait_for_server()
clientLook.wait_for_server()
print >>sys.stderr, 'move base topic is', moveBaseTopic
clientMove.wait_for_server()
print >>sys.stderr, 'simple action clients ok'

pubTarget = rospy.Publisher('/follow/target', PointStamped)
pubGoal = rospy.Publisher('/follow/goal', PointStamped)
pubRobot = rospy.Publisher('/follow/robot', PointStamped)
pubBaseCommand = rospy.Publisher('base_controller/command', Twist)
pubVoice = rospy.Publisher('/pcsdk/synth', String)

from random import randrange
def say(*text):
    data = String()
    data.data = text[randrange(len(text))]
    pubVoice.publish(data)

def clamp(x, x_min, x_max):
    return min(x_max, max(x_min, x))

reset = True
state = 'waiting'

#head_positions = (0,0.1)
robot_pos = None
robot_yaw = None

points_seq = 0
def createPoint(x,y,z=0):
    global points_seq
    points_seq += 1
    point = PointStamped()
    point.header.seq = points_seq
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = frame
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point

def panTilt(positions):
    g = JointTrajectoryGoal()
    g.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
    g.trajectory.points = [JointTrajectoryPoint()]
    g.trajectory.points[0].positions = positions;
    g.trajectory.points[0].velocities = [0, 0];
    # pan(-:right, +:left), tilt(neg:look up, pos:look down)
    g.trajectory.points[0].time_from_start = rospy.Duration.from_sec(0.6)
    clientHead.send_goal(g)
    clientHead.wait_for_result()

def lookAt(pos):
    g = PointHeadGoal()
    g.target.header.frame_id = frame
    g.target.point.x = pos[0]
    g.target.point.y = pos[1]
    g.target.point.z = pos[2]
    g.pointing_axis.x = 0
    g.pointing_axis.y = 0
    g.pointing_axis.z = 1
    g.pointing_frame = '/head_mount_kinect_rgb_optical_frame'
    g.min_duration = rospy.Duration.from_sec(0.6)
    g.max_velocity = 0.8
    clientLook.send_goal(g)

from math import tan, atan2, pi, sqrt
lastMoveGoal = rospy.Time.now().to_sec()
def dist2(a,b):
    d = (a[0]-b[0], a[1]-b[1])
    return sqrt(d[0]*d[0]+d[1]*d[1])

lastPosition = None
def moveTo(pos):
    global lastMoveGoal, lastPosition
    #if rospy.Time.now().to_sec() - lastMoveGoal < 1:
    if lastPosition is not None and dist2(lastPosition, pos) < 0.3:
        return False
    #lastMoveGoal = rospy.Time.now().to_sec()
    lastPosition = pos
 
    #print 'move to %.2f,%.2f' % (pos[0], pos[1])
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = frame
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = pos[0]
    g.target_pose.pose.position.y = pos[1]
    g.target_pose.pose.position.z = 0
    g.target_pose.pose.orientation.x = 0
    g.target_pose.pose.orientation.y = 0
    g.target_pose.pose.orientation.z = 0
    g.target_pose.pose.orientation.w = 1
    #clientMove.cancel_all_goals()
    pubGoal.publish(createPoint(pos[0], pos[1]))
    clientMove.send_goal(g)
    return True


def faceTo(pos):
    movement = Twist()
    azim = atan2(pos[1], pos[0])
    if azim - robot_yaw > pi:
        turn = (azim - robot_yaw) - 2*pi
    elif azim - robot_yaw < -pi:
        turn = (azim - robot_yaw) + 2*pi
    else:
        turn = azim - robot_yaw
    movement.angular.z = clamp(turn, -0.4, 0.4)
    print 'azim %.0f yaw %.0f z %.0f' % (azim/pi*180, robot_yaw/pi*180, turn/pi*180)
    pubBaseCommand.publish(movement)

def target_callback(data):
    if robot_pos is None:
        return
    if reset:
        return
    look_pos = [data.x, data.y, 0]
    distance = dist2(robot_pos, look_pos)
    look_pos[2] = 1.35 - distance * tan(0.1) # tilt down at 0.1 rad
    if distance > 0.3:
        lookAt(look_pos)
    pubTarget.publish(createPoint(data.x, data.y))

    if state != 'following':
        return

    if distance < 1.0:
        clientMove.cancel_all_goals()
        diff = [look_pos[0]-robot_pos[0], look_pos[1]-robot_pos[1]]
        faceTo(diff)
        return

    SAFE_DISTANCE = 0.5
    if distance > SAFE_DISTANCE:
        diff = [robot_pos[0]-look_pos[0], robot_pos[1]-look_pos[1]]
        goal_pos = [look_pos[0]+diff[0]/distance*SAFE_DISTANCE, look_pos[1]+diff[1]/distance*SAFE_DISTANCE]
        #goal_pos = look_pos
        if moveTo(goal_pos):
            print 'from %.2f,%.2f to %.2f,%.2f' % (robot_pos[0], robot_pos[1], goal_pos[0], goal_pos[1])

#def recv_joint_states(data):
#    global head_positions
#    head_positions = data.actual.positions

from tf.transformations import euler_from_quaternion, quaternion_from_euler
def pose_callback(data):
    global robot_pos, robot_yaw
    robot_pos = [data.position.x, data.position.y, data.position.z]
    q = data.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    robot_yaw = yaw
    pubRobot.publish(createPoint(data.position.x, data.position.y, data.position.z))

def speech_callback(data):
    global state, reset
    speech = data.data.lower()
    print 'got speech', speech
    if 'reset' in speech or 'restart' in speech:
        clientMove.cancel_all_goals()
        clientLook.cancel_all_goals()
        defocus()
        reset = True
        panTilt((0,0.1))
        state = 'waiting'
    elif 'wait' in speech or 'stop' in speech:
        clientMove.cancel_all_goals()
        state = 'waiting'
        reset = False
        say("OK I'll wait", "Now waiting", "Waiting", "I stop here")
        defocus()
    elif 'follow' in speech or "let's go" in speech:
        state = 'following'
        say("OK I'll follow you", "Let's move", "Let's go")
        focus()
        reset = False
    elif 'hello' in speech or speech == "hey":
        say("Hello", 'Hi')
        focus()
        reset = False
        

panTilt((0,0.1))
print >>sys.stderr, 'pan tilt done'
reset = False
rospy.Subscriber('/robot_pose', Pose, pose_callback)
rospy.Subscriber("/tracker/nearest", Point, target_callback)
#rospy.Subscriber('/head_traj_controller/state', JointTrajectoryControllerState, recv_joint_states)
rospy.Subscriber('/pcsdk/recog/speech', String, speech_callback)
print >>sys.stderr, 'subscribers ok'
rospy.spin()
