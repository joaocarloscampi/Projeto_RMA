import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from simClasses import Robot, Ball
from execution import univec_controller

import numpy as np
import time
from random import uniform

pub = rospy.Publisher("/yellow_team/robot_0/diff_drive_controller/cmd_vel", Twist, queue_size=10)

robot = Robot(0, True)
ball = Ball()
arrived = False
def Tp(name, x, y, z, yaw):
    state_msg = ModelState()
    state_msg.model_name = name
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z

    orientation_list = [0, 0, yaw]
    (qx, qy, qz, qw) = quaternion_from_euler(0, 0, yaw)

    state_msg.pose.orientation.x = qx
    state_msg.pose.orientation.y = qy
    state_msg.pose.orientation.z = qz
    state_msg.pose.orientation.w = qw

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        print("Service call failed: ")

def Go_To_Goal(robot, ball):
    arrival_theta = 0
    print('Go_To_Goal')
    robot.target.update(ball.xPos, ball.yPos, arrival_theta)
    v, w = univec_controller(robot, robot.target, avoid_obst=False, double_face=False)
    robot.sim_set_vel(pub, v/100, w)

def Position_Robot(data):
    global arrived
    robot.sim_get_pose(data)
    Go_To_Goal(robot, ball)
    arrived = robot.arrive()
    if robot.arrive():
        sub_robot.unregister()


def Position_Ball(data):
    x_pos = data.pose.position.x*100 + 85
    y_pos = data.pose.position.y*100 + 65
    ball.sim_get_pose(data)
    if robot.arrive():
        sub_ball.unregister()

def Publisher_Twist(v, w):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)


def Run(tp_x,tp_y, tp_theta):
    rospy.init_node('testeTraveSim', anonymous=True) #make node
    global sub_ball
    global sub_robot

    sub_robot = rospy.Subscriber('/vision/yellow_team/robot_0',ModelState,Position_Robot)
    sub_ball = rospy.Subscriber('/vision/ball',ModelState,Position_Ball)
    rospy.spin()

    finish_time = time.time()
    dy = abs(robot.yPos - robot.target.yPos)
    dtheta = robot.theta
    dt = finish_time - start_time
    Tp('yellow_team/robot_0', tp_x, tp_y, 0.02, 0)
    Tp('vss_ball', 0, 0, 0.05, 0)

    return dy, dtheta, dt
