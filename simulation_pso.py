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

import csv
from copy import deepcopy
#import ga

from pso import Particle

import matplotlib.pyplot as plt

def cost_func(dt, dang, dy, K_t = 10, K_p = 8, K_d = 2):
    global max_dt, index_dt, max_dy, index_dy, max_dang, index_dang
    cost = 0
    for i in range(len(dang)):
        cost += K_t*dt[i] + K_p*np.sqrt(np.rad2deg(dang[i])*np.rad2deg(dang[i])) + dy[i]*dy[i]
    max_dt.append(max(dt))
    index_dt.append(dt.index(max_dt[-1]) + 1)
    max_dy.append(max(dy))
    index_dy.append(dy.index(max_dy[-1]) + 1)
    max_dang.append(max(dang))
    index_dang.append(dang.index(max_dang[-1]) + 1)
    return cost

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
    global cont_ind
    global cont_pos
    global start_time
    global finish_time
    global robot
    global ball
    global tp
    global tp_firstime
    global cont_tp
    global vec_dt
    global vec_dy
    global vec_dang
    global flag_gen
    global cont_gen
    global header
    global data_csv
    global cont_line
    global line1
    global figure
    global yPlotMean
    global yPlotBest
    global flagTime
    global vec_flagsTime

    if cont_gen <= ga_univector.maxit:
        if first_time:
            if cont_ind != ga_univector.npop:
                print("Geração: " + str(cont_gen+1) + ", Individuo " + str(cont_ind+1))
            start_time = rospy.get_time()
            Tp('yellow_team/robot_0', pos_x[0], pos_y[0], 0.02, pos_ang[0])
            Tp('vss_ball', 85, 65, 0.05, 0)
            tp = True
            first_time = False

        robot.sim_get_pose(data)
        #Go_To_Goal(robot, ball, ga_univector.pop[cont_ind])

        if not tp:
            if robot.arrive() or flagTime:

                robot.sim_set_vel(pub, 0, 0)
                finish_time = rospy.get_time()

                dt = finish_time - start_time
                print("dt: ", dt)
                dy = robot.yPos - ball.yPos
                dang = robot.theta

                vec_dt.append(dt)
                vec_dy.append(dy)
                vec_dang.append(dang)
                vec_flagsTime.append(flagTime)

                #ga_univector.update_cost_param(dy,dang,dt)

                if cont_pos < len(pos_x):
                    #if not tp:
                    print("Posicao ", cont_pos+1 )
                    Tp('yellow_team/robot_0', pos_x[cont_pos], pos_y[cont_pos], 0.02, pos_ang[cont_pos])
                    Tp('vss_ball', 85, 65, 0.05, 0)
                    tp = True
                    cont_pos += 1

                else:
                    ga_univector.cost_func(vec_dt, vec_dang, vec_dy)
                    cont_pos = 1
                    cont_ind += 1
                    first_time = True
                    flagTime = False
                    vec_dt = []
                    vec_dy = []
                    vec_dang = []
                    vec_flagsTime = []

                    if len(ga_univector.vec_cost) == 2*ga_univector.npop:
                        temp_pop = np.zeros([2*ga_univector.npop,ga_univector.nvar])
                        aux_temp_pop = np.zeros([ga_univector.npop,ga_univector.nvar])
                        aux_cost = []
                        for i in range(2*ga_univector.npop):
                            if i < ga_univector.npop:
                                temp_pop[i] = ga_univector.oldPop[i]
                            else:
                                temp_pop[i] = ga_univector.pop[i-ga_univector.npop]
                        for i in range(ga_univector.npop):
                            min_value = min(ga_univector.vec_cost)
                            min_index = ga_univector.vec_cost.index(min_value)
                            aux_cost.append(min_value)
                            aux_temp_pop[i] = temp_pop[min_index]
                            ga_univector.vec_cost[min_index] = np.inf
                        ga_univector.pop = deepcopy(aux_temp_pop)
                        ga_univector.vec_cost = deepcopy(aux_cost)
                        #print("Os melhores foram selecionados!!!")
                        for i in range(ga_univector.npop):
                            data_csv.append([cont_gen,ga_univector.pop[i][0],ga_univector.pop[i][1],ga_univector.pop[i][2],ga_univector.pop[i][3],ga_univector.pop[i][4], ga_univector.vec_cost[i],
                                             ga_univector.index_dt[i], ga_univector.max_dt[i], ga_univector.index_dy[i], ga_univector.max_dy[i],ga_univector.index_dang[i], ga_univector.max_dang[i]])
                        ga_univector.findBetterCost()
                        with open('results.csv', 'w', encoding='UTF8', newline='') as f:
                            writer = csv.writer(f)

                            # write the header
                            writer.writerow(header)

                            # write multiple rows
                            writer.writerows(data_csv)

                            f.close()
                        print("Média da geração: ", sum(ga_univector.vec_cost)/ga_univector.npop)
                        print("Melhor custo: ", ga_univector.cost_better)
                        print("Parâmetros do individuo: ", ga_univector.pop[ga_univector.index_better])
                        print("----")
                        ga_univector.max_dt = []
                        ga_univector.index_dt = []
                        ga_univector.max_dy = []
                        ga_univector.index_dy = []
                        ga_univector.max_dang = []
                        ga_univector.index_dang = []

                start_time = rospy.get_time()
                flagTime = False
                #if cont_ind == len(ga_univector.pop):
                    #exit()
            elif cont_ind < ga_univector.npop:
                Go_To_Goal(robot, ball, ga_univector.pop[cont_ind])
                intermed_time = rospy.get_time()
                if intermed_time - start_time > 10:
                    flagTime = True



            else:
                #exit()
                #ga_univector.findBetterCost()
                ga_univector.nextGen()
                cont_gen += 1
                cont_pos = 1
                cont_ind = 0
                first_time = True
                flagTime = False
                vec_dt = []
                vec_dy = []
                vec_dang = []
                vec_flagsTime = []

        else:
            if cont_tp > 100:
                tp = False
                cont_tp = 0
            else:
                cont_tp = cont_tp+1

def Position_Robot_PSO(data):

    global cont_gen
    global cont_ind
    global cont_pos
    global maxiter
    global num_particles
    global vec_dt
    global vec_dy
    global vec_dang
    global err_best_g
    global pos_best_g
    global swarm
    global bounds
    global first_time
    global start_time
    global finish_time
    global tp
    global cont_tp
    global flagTime
    global max_dt, index_dt, max_dy, index_dy, max_dang, index_dang

    if cont_gen < maxiter:
        if first_time:
            if cont_ind != num_particles:
                print("Geração: " + str(cont_gen) + ", Individuo " + str(cont_ind+1))
                print("Individuo: ", swarm[cont_ind].position_i)
            start_time = rospy.get_time()
            Tp('yellow_team/robot_0', pos_x[0], pos_y[0], 0.02, pos_ang[0])
            Tp('vss_ball', 85, 65, 0.05, 0)
            tp = True
            first_time = False

        robot.sim_get_pose(data)
        #Go_To_Goal(robot, ball, ga_univector.pop[cont_ind])

        if not tp:
            if robot.arrive() or flagTime:
                robot.sim_set_vel(pub, 0, 0)
                finish_time = rospy.get_time()

                dt = finish_time - start_time
                print("dt: ", dt)
                dy = robot.yPos - ball.yPos
                dang = robot.theta

                vec_dt.append(dt)
                vec_dy.append(dy)
                vec_dang.append(dang)
                #vec_flagsTime.append(flagTime)

                #ga_univector.update_cost_param(dy,dang,dt)

                if cont_pos < len(pos_x):
                    #if not tp:
                    print("Posicao ", cont_pos+1 )
                    Tp('yellow_team/robot_0', pos_x[cont_pos], pos_y[cont_pos], 0.02, pos_ang[cont_pos])
                    Tp('vss_ball', 85, 65, 0.05, 0)
                    tp = True
                    cont_pos += 1

                else:
                    cost = cost_func(vec_dt, vec_dang, vec_dy)
                    print("Custo: ", cost)
                    swarm[cont_ind].err_i = cost
                    if swarm[cont_ind].err_i<swarm[cont_ind].err_best_i or swarm[cont_ind].err_best_i==-1:
                        swarm[cont_ind].pos_best_i=swarm[cont_ind].position_i.copy()
                        swarm[cont_ind].err_best_i=swarm[cont_ind].err_i
                    if swarm[cont_ind].err_i<err_best_g or err_best_g==-1:
                        pos_best_g=list(swarm[cont_ind].position_i)
                        err_best_g=float(swarm[cont_ind].err_i)
                    cont_pos = 1
                    cont_ind += 1
                    first_time = True
                    flagTime = False
                    vec_dt = []
                    vec_dy = []
                    vec_dang = []
                    vec_flagsTime = []

                    if cont_ind == num_particles:
                        for i in range(num_particles):
                            print("Index: ", index_dt)
                            data_csv.append([cont_gen ,swarm[i].position_i[0],swarm[i].position_i[1],1,1,1, swarm[i].err_i ,
                                             index_dt[i], max_dt[i], index_dy[i], max_dy[i],index_dang[i], max_dang[i]])
                        for j in range(0,num_particles):
                            swarm[j].update_velocity(pos_best_g)
                            swarm[j].update_position(bounds)
                        with open('results.csv', 'w', encoding='UTF8', newline='') as f:
                            writer = csv.writer(f)

                            # write the header
                            writer.writerow(header)

                            # write multiple rows
                            writer.writerows(data_csv)

                            f.close()
                        # print("Média da geração: ", sum(ga_univector.vec_cost)/ga_univector.npop)
                        # print("Melhor custo: ", ga_univector.cost_better)
                        # print("Parâmetros do individuo: ", ga_univector.pop[ga_univector.index_better])
                        # print("----")
                        # ga_univector.max_dt = []
                        # ga_univector.index_dt = []
                        # ga_univector.max_dy = []
                        # ga_univector.index_dy = []
                        # ga_univector.max_dang = []
                        # ga_univector.index_dang = []

                start_time = rospy.get_time()
                flagTime = False

                #if cont_ind == len(ga_univector.pop):
                    #exit()
            elif cont_ind < num_particles:
                Go_To_Goal(robot, ball, swarm[cont_ind].position_i)
                intermed_time = rospy.get_time()
                if intermed_time - start_time > 10:
                    flagTime = True



            else:
                #exit()
                #ga_univector.findBetterCost()
                #ga_univector.nextGen()
                print(f'iter: {cont_gen:>4d}, best solution: {pos_best_g}')
                cont_gen += 1
                cont_pos = 1
                cont_ind = 0
                first_time = True
                flagTime = False
                vec_dt = []
                vec_dy = []
                vec_dang = []
                vec_flagsTime = []
                max_dt = []
                index_dt = []
                max_dy = []
                index_dy = []
                max_dang = []
                index_dang = []

        else:
            if cont_tp > 100:
                tp = False
                cont_tp = 0
            else:
                cont_tp = cont_tp+1


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
    cont_ind = 0
    cont_pos = 1

    pos_x = [15,47.5,85,122.5,155,122.5,85,47.5]
    pos_y = [65,115,120,115,65,25,20,25]
    pos_ang = [0,0,180,180,180,180,180,0]

    start_time = 0
    finish_time = 0

    dy = 0
    dang = 0

    tp = False
    tp_firstime = False
    cont_tp = 0

    data_csv = []

    vec_dt = []
    vec_dy = []
    vec_dang = []
    vec_flagsTime = []

    max_dt = []
    index_dt = []
    max_dy = []
    index_dy = []
    max_dang = []
    index_dang = []

    flagTime = False

    ##GA var
    ga_univector = GA(5,0,50,100,20)
    ga_univector.initialize_pop()
    cont_gen = 0
    header = ['Generation','d_e', 'k_r','delta','k_o','d_min','Cost','index_dt','dt','index_dy','dy','index_dang','dang']

    ##PSO var
    global num_dimensions

    x0 = [3, 6]
    bounds = [(0,10), (0, 20)]
    num_particles = 2
    maxiter = 2

    num_dimensions=len(x0)
    err_best_g=-1                   # best error for group
    pos_best_g=[]                   # best position for group

    # establish the swarm
    swarm=[]
    for i in range(0,num_particles):
        swarm.append(Particle(x0, num_dimensions))

    rospy.init_node('testeTraveSim', anonymous=True, disable_signals = True) #make node

    sub_robot = rospy.Subscriber('/vision/yellow_team/robot_0',ModelState,Position_Robot_PSO)
    sub_ball = rospy.Subscriber('/vision/ball',ModelState,Position_Ball)
    rospy.spin()
