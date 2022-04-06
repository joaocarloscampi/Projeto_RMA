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
from random import uniform

pub = rospy.Publisher("/yellow_team/robot_0/diff_drive_controller/cmd_vel", Twist, queue_size=10)

robot = Robot(0, True)
ball = Ball()

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
    arrival_theta = np.arctan2(65 - ball.yPos, 160 - ball.xPos)
    robot.target.update(ball.xPos, ball.yPos, arrival_theta)
    v, w = univec_controller(robot, robot.target, avoid_obst=False, double_face=False)
    print(v)
    print(w)
    robot.sim_set_vel(pub, v/100, w)

def Position_Robot(data):
    robot.sim_get_pose(data)
    Go_To_Goal(robot, ball)
    #robot.sim_set_vel(pub, 1, 0)
    if ball.xPos > 160:
        y_ball = uniform(-0.5, 0.5)
        Tp('yellow_team/robot_0', -0.5, 0.2, 0.02, 0)
        Tp('vss_ball', -0.3, y_ball, 0.05, 0)
    #print("x: ", robot.xPos)
    #print("y: ", robot.yPos)
    #print("angulo: ", robot.theta)

    # Publisher_Twist(0.4, 0.7)

def Position_Ball(data):
    x_pos = data.pose.position.x*100 + 85
    y_pos = data.pose.position.y*100 + 65

    ball.sim_get_pose(data)

    print("x_ball: ", ball.xPos)
    print("y_ball: ", ball.yPos)
    print("------------")

def Publisher_Twist(v, w):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('testeTraveSim', anonymous=True) #make node
        rospy.Subscriber('/vision/yellow_team/robot_0',ModelState,Position_Robot)
        rospy.Subscriber('/vision/ball',ModelState,Position_Ball)
        rospy.spin()
