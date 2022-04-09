import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from simClasses import Robot, Ball, GA
from execution import univec_controller

import numpy as np
import time
from random import uniform

from ypstruct import structure
import ga


def Tp(name, x, y, z, yaw):
    
    x = (x - 85)/100
    y = (y - 65)/100
    yaw = np.deg2rad(yaw)

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

def Go_To_Goal(robot, ball, ind):

    arrival_theta = 0
    robot.target.update(ball.xPos, ball.yPos, arrival_theta)
    v, w = univec_controller(robot, robot.target, ind,avoid_obst=False, double_face=False)
    robot.sim_set_vel(pub, v/100, w)


def Position_Robot(data):
    
    global first_time
    global cont_pop
    global cont_pos
    global start_time
    global finish_time
    global robot
    global ball


    if first_time:
        start_time = time.time()
        Tp('yellow_team/robot_0', pos_x[0], pos_y[0], 0.02, pos_ang[0])
        Tp('vss_ball', 85, 65, 0.05, 0)
        ga_univector.initialize_pop()
        first_time = False

    robot.sim_get_pose(data)
    Go_To_Goal(robot, ball, ga_univector.pop[cont_pop])

    if robot.arrive():
        print('X:  ', robot.xPos)
        print('Y:  ', robot.yPos)
        robot.sim_set_vel(pub, 0, 0)
        finish_time = time.time()
        dt = finish_time - start_time
        dy = robot.yPos - ball.yPos
        dang = robot.theta
        ga_univector.update_cost_param(dy,dang,dt)

        if cont_pos < len(pos_x):
            Tp('yellow_team/robot_0', pos_x[cont_pos], pos_y[cont_pos], 0.02, pos_ang[cont_pos])
            Tp('vss_ball', 85, 65, 0.05, 0)

            cont_pos += 1 

        else:
            ga_univector.cost_func()
            cont_pos = 1
            cont_pop += 1
            first_time = True

        start_time = time.time()

        if cont_pop > len(ga_univector.pop):
            exit()
    else:
        Go_To_Goal(robot, ball, ga_univector.pop[cont_pop]) 
        print('Aqui')       

def Position_Ball(data):
    x_pos = data.pose.position.x*100 + 85
    y_pos = data.pose.position.y*100 + 65
    ball.sim_get_pose(data)

def Publisher_Twist(v, w):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)


if __name__ == '__main__':

    ## Simulation var
    pub = rospy.Publisher("/yellow_team/robot_0/diff_drive_controller/cmd_vel", Twist, queue_size=10)

    robot = Robot(0, True)
    ball = Ball()
    arrived = False
    first_time = True
    cont_pop = 0
    cont_pos = 1

    pos_x = [15,47.5,85,122.5,155,122.5,85,47.5]
    pos_y = [65,115,120,115,65,25,20,25]
    pos_ang = [0,0,180,180,180,180,180,0]

    start_time = 0
    finish_time = 0

    dy = 0
    dang = 0

    ##GA var
    ga_univector = GA(5,0,10,100,10)

    rospy.init_node('testeTraveSim', anonymous=True, disable_signals = True) #make node

    sub_robot = rospy.Subscriber('/vision/yellow_team/robot_0',ModelState,Position_Robot)
    sub_ball = rospy.Subscriber('/vision/ball',ModelState,Position_Ball)
    rospy.spin()
