from numpy import sqrt, array, amin, where, zeros, delete, append, int32, argmin, random, argwhere, maximum

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# from scipy.spatial import distance -> Descomentar quando atividade do Grid voltar

# Units: cm, rad, s

"""
Input: Current target coordinates.
Description: Stores coordinates for the robots' current target.
Output: Current target coordinates.
"""
class Target:
    def __init__(self):
        self.xPos = 0  # ? Desired x position
        self.yPos = 0  # ? Desired y position
        self.theta = 0  # ? Orientation at the desired point (x,y)

    """
    Input: Current target coordinates.
    Description: Sets current target coordinates from vision data.
    Output: None
    """
    def update(self, x, y, theta):
        self.xPos = x
        self.yPos = y
        self.theta = theta

    """
    Input: None
    Description: Logs target coordinates to the console.
    Output: Current target coordinates.
    """
    def show_info(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f}'.format(self.xPos, self.yPos, float(self.theta)))


"""
Input: Coordinates and velocity of object.
Description: Stores coordinates and velocity of an obstacle to a robot.
Output: Coordinates and velocity of object.
"""

class Obstacle:
    def __init__(self):
        self.xPos = 0  # ? Obstacle x position
        self.yPos = 0  # ? Obstacle y position
        self.v = 0  # ? Obstacle velocity (cm/s)
        self.theta = 0  # ? Obstacle orientation

    """
    Input: Coordinates of obstacle.
    Description: Sets obstacle coordinates with data from vision.
    Output: None
    """
    def set_obst(self, x, y, v, theta):
        self.xPos = x
        self.yPos = y
        self.v = v
        self.theta = theta

    """
    Input: Object lists.
    Description: Detects nearest object and sets it as the current obstacle to avoid.
    Output: Current obstacle.
    """
    def update(self, robot, friend1, friend2, enemy1=None, enemy2=None, enemy3=None):
        if (enemy1 is None) and (enemy2 is None) and (enemy3 is None):
            d = array([[robot.dist(friend1)],
                       [robot.dist(friend2)]])
        elif (enemy2 is None) and (enemy3 is None):
            d = array([[robot.dist(friend1)],
                       [robot.dist(friend2)],
                       [robot.dist(enemy1)]])
        elif enemy3 is None:
            d = array([[robot.dist(friend1)],
                       [robot.dist(friend2)],
                       [robot.dist(enemy1)],
                       [robot.dist(enemy2)]])
        else:
            d = array([[robot.dist(friend1)],
                       [robot.dist(friend2)],
                       [robot.dist(enemy1)],
                       [robot.dist(enemy2)],
                       [robot.dist(enemy3)]])

        index = where(d == amin(d))
        if index[0][0] == 0:
            self.set_obst(friend1.xPos, friend1.yPos, friend1.v, friend1.theta)
        elif index[0][0] == 1:
            self.set_obst(friend2.xPos, friend2.yPos, friend2.v, friend2.theta)
        elif index[0][0] == 2:
            self.set_obst(enemy1.xPos, enemy1.yPos, 0, 0)
        elif index[0][0] == 3:
            self.set_obst(enemy2.xPos, enemy2.yPos, 0, 0)
        else:
            self.set_obst(enemy3.xPos, enemy3.yPos, 0, 0)
    """
    Input:
    Description: Detects nearest object and sets it as the current obstacle to avoid with some exceptions:
                 1 - The enemy player closest to the goal is not be considered obstacle
                 2 - If ball is too close to the enemy robot, he is not be considered obstacle
    Output:
    """
    def update2(self, robot, ball, friend1, friend2, enemy1, enemy2, enemy3):
        enemys = array([enemy1, enemy2, enemy3])
        d_ball = array([[enemy1.dist(ball)],
                        [enemy2.dist(ball)],
                        [enemy3.dist(ball)]]) # Distance to ball of all enemies robots
        index = argmin(d_ball) # Index of shortest distance

        if d_ball[index] < 15: # If robot is too close, disconsider
            enemys = delete(enemys, [index])

        if not robot.teamYellow: # Goal coordinates for each team
            x_gol = 160
            y_gol = 65
        else:
            x_gol = 10
            y_gol = 65

        if len(enemys) == 3: # If the first exception did not happen
            # Distances to goal
            d1 = sqrt((x_gol - enemy1.xPos) ** 2 + (y_gol - enemy1.yPos) ** 2)
            d2 = sqrt((x_gol - enemy2.xPos) ** 2 + (y_gol - enemy2.yPos) ** 2)
            d3 = sqrt((x_gol - enemy3.xPos) ** 2 + (y_gol - enemy3.yPos) ** 2)
            d_gol = array([[d1],
                           [d2],
                           [d3]])

            index = argmin(d_gol) # Index of shortest distance

            dballgol = sqrt((x_gol - ball.xPos) ** 2 + (y_gol - ball.yPos) ** 2) # Ball distance from goal

            if d_gol[index] < 20 and dballgol < 20: # If ball and enemy are close to goal, disconsider
                enemys = delete(enemys, index)
        else: # If the first exception did happen
            # Distances to goal
            d1 = sqrt((x_gol - enemys[0].xPos) ** 2 + (y_gol - enemys[0].yPos) ** 2)
            d2 = sqrt((x_gol - enemys[1].xPos) ** 2 + (y_gol - enemys[1].yPos) ** 2)
            d_gol = array([[d1],
                           [d2]])

            index = argmin(d_gol) # Index of shortest distance

            dballgol = sqrt((x_gol - ball.xPos) ** 2 + (y_gol - ball.yPos) ** 2) # Ball distance from goal

            if d_gol[index] < 20 and dballgol < 20: # If ball and enemy are close to goal, disconsider
                enemys = delete(enemys, index)

        # Adding the team robots
        enemys = append(enemys, friend1)
        enemys = append(enemys, friend2)
        d_robot = zeros(len(enemys))

        # Detecting nearest object
        for i in range(len(enemys)):
            d_robot[i] = robot.dist(enemys[i])
        index = argmin(d_robot)

        # Setting current obstacle
        self.set_obst(enemys[index].xPos, enemys[index].yPos, 0, 0)

    """
    Input: None
    Description: Logs obstacle info on the console.
    Output: Obstacle data.
    """
    def show_info(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos, self.yPos,
                                                                                      float(self.theta), self.v))


"""
Input: Ball coordinates.
Description: Stores data on the game ball.
Output: Ball data.
"""
class Ball:
    def __init__(self):
        self.xPos = 0
        self.yPos = 0
        self.vx = 0
        self.vy = 0
        self.pastPose = zeros(4).reshape(2, 2)  # Stores the last 3 positions (x,y) => updated on self.simGetPose()

    """
    Input: FIRASim ball location data.
    Description: Gets position of the ball from the simulator.
    Output: None
    """
    def sim_get_pose(self, data_ball):
        # self.xPos = data_ball.x + data_ball.vx * 100 * 8 / 60
        # self.yPos = data_ball.y + data_ball.vy * 100 * 8 / 60
        #
        # # Check if prev is out of field, in this case reflect ball moviment to reproduce the collision
        # if self.xPos > 160:
        #     self.xPos = 160 - (self.yPos - 160)
        # elif self.xPos < 10:
        #     self.xPos = 10 - (self.yPos - 10)
        #
        # if self.yPos > 130:
        #     self.yPos = 130 - (self.yPos - 130)
        # elif self.yPos < 0:
        #     self.yPos = - self.yPos
        #
        # self.vx = data_ball.vx
        # self.vy = data_ball.vy

        self.xPos = data_ball.pose.position.x*100 + 85
        self.yPos = data_ball.pose.position.y*100 + 65
        self.vx = data_ball.twist.linear.x
        self.vy = data_ball.twist.linear.y

        self.v = sqrt(self.vx ** 2 + self.vy ** 2)

    """
    Input: Ball data.
    Description: Logs ball data into console.
    Output: None.
    """
    def show_info(self):
        print('xPos: {:.2f} | yPos: {:.2f}'.format(self.xPos, self.yPos))


"""
Input: Robot data.
Description: Stores data about robots in the game.
Output: Robot data.
"""
class Robot:
    def __init__(self, index, mray):
        self.flagDirectGoal = False
        self.flagCruzamento = False
        self.flagTrocaFace = False
        self.isLeader = None
        self.teamYellow = mray
        self.spin = False
        self.contStopped = 0
        self.holdLeader = 0
        self.index = int32(index)
        #self.actuator = actuator
        self.face = 1  # ? Defines the current face of the robot
        self.xPos = 0  # ? X position
        self.yPos = 0  # ? Y position
        self.theta = 0  # ? Orientation
        self.rightMotor = 0  # ? Right motor handle
        self.leftMotor = 0  # ? Left motor handle
        self.v = 0  # ? Velocity (cm/s) => updated on execution.py
        self.vx = 0
        self.vy = 0
        self.vTheta = 0
        self.vL = 0  # ? Left wheel velocity (cm/s) => updated on simClasses.py -> simSetVel()
        self.vR = 0  # ? Right wheel velocity (cm/s) =>  updated on simClasses.py -> simSetVel()
        if self.index == 0: # ! Robot max velocity (cm/s)
            self.vMax=40#35
        else:
            self.vMax=50
        self.rMax = 3 * self.vMax  # ! Robot max rotation velocity (rad*cm/s)
        self.L = 7.5  # ? Base length of the robot (cm)
        self.LSimulador = 6.11  # ? Base length of the robot on coppelia (cm)
        self.R = 3.4  # ? Wheel radius (cm)
        self.obst = Obstacle()  # ? Defines the robot obstacle
        self.target = Target()  # ? Defines the robot target
        # ? Stores the last 3 positions (x,y) and orientation => updated on execution.py
        self.pastPose = zeros(12).reshape(4,
                                          3)


    """
    Input: Object data.
    Description: Calculates distance between robot and another object.
    Output: Distance between robot and object.
    """
    def dist(self, obj):
        return sqrt((self.xPos - obj.xPos) ** 2 + (self.yPos - obj.yPos) ** 2)


    """
    Input: None.
    Description: Returns True if the distance between the target and the robot is less than 3cm - False otherwise
    Output: True or False.
    """
    def arrive(self):
        if self.dist(self.target) <= 5:
            return True
        else:
            return False


    """
    Input: Simulator robot data.
    Description: Gets both position and orientation of the robot in FIRASim
    Output: None.
    """
    # def sim_get_pose(self, data_robot):
    #     self.xPos = data_robot.x
    #     self.yPos = data_robot.y
    #     self.vx = data_robot.vx
    #     self.vy = data_robot.vy
    #     self.theta = data_robot.a
    #     self.vTheta = data_robot.va
    #     self.v = sqrt(self.vx ** 2 + self.vy ** 2)

    def sim_get_pose(self, data_robot):
        self.xPos = data_robot.pose.position.x*100 + 85
        self.yPos = data_robot.pose.position.y*100 + 65
        self.vx = data_robot.twist.linear.x
        self.vy = data_robot.twist.linear.y

        orientation_q = data_robot.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        self.theta = yaw
        self.vTheta = data_robot.twist.angular.z
        self.v = sqrt(self.vx ** 2 + self.vy ** 2)


    """
    Input: Linear and angular velocity data.
    Description: Sends velocity data to simulator to move the robots.
    Output: None.
    """
    #def sim_set_vel(self, v, w):
        #if self.face == 1:
            #self.vR = v + 0.5 * self.L * w
            #self.vL = v - 0.5 * self.L * w
        #else:
            #self.vL = -v - 0.5 * self.L * w
            #self.vR = -v + 0.5 * self.L * w
        #self.actuator.send(self.index, self.vL, self.vR)


    """
    Input: Wheels velocity data.
    Description: Sends velocity data to simulator to move the robots.
    Output: None.
    """
    #def sim_set_vel2(self, v1, v2):
        #self.actuator.send(self.index, v1, v2)

    def sim_set_vel(self, publisher, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        publisher.publish(msg)


    """
    Input: None.
    Description: Logs robot data to the console.
    Output: Robot data.
    """
    def show_info(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos, self.yPos,
                                                                                      float(self.theta), float(self.v)))


class GA:
    def __init__(self,nvar,varmin,varmax,maxit,npop, K_t = 10, K_p = 5, K_d = 2):
        self.nvar = nvar
        self.varmin = varmin
        self.varmax = varmax
        self.maxit = maxit
        self.npop = npop
        self.K_t = K_t
        self.K_p = K_p
        self.K_d = K_d
        self.pop = []
        self.vec_cost = []
        self.vec_dy = []
        self.vec_dang = []
        self.vec_dt = []

    def update_cost_param(self,dy,dang,dt):
        
        self.vec_dy.append(dy)
        self.vec_dang.append(dang)
        self.vec_dt.append(dt)

    def initialize_pop(self):

        self.pop = zeros([self.npop,self.nvar])
        for i in range(self.npop):
            self.pop[i] = random.uniform(self.varmin, self.varmax, self.nvar)       
    
    def cost_func(self):
        
            cost = (self.K_t*dt + self.K_p*dang**2 + self.K_d*dy**2)
            self.vec_cost.append(sum(cost)) 

    def crossover(self,p1, p2, gamma=0.1):
        c1 = p1.deepcopy()
        c2 = p1.deepcopy()
        alpha = random.uniform(-gamma, 1+gamma, *c1.position.shape)
        c1.position = alpha*p1.position + (1-alpha)*p2.position
        c2.position = alpha*p2.position + (1-alpha)*p1.position
        return c1, c2

    def mutate(self,x, mu, sigma):
        y = x.deepcopy()
        flag = random.rand(*x.position.shape) <= mu
        ind = argwhere(flag)
        y.position[ind] += sigma*random.randn(*ind.shape)
        return y

    def apply_bound(self,x, varmin, varmax):
        x.position = maximum(x.position, varmin)
        x.position = minimum(x.position, varmax)

    def roulette_wheel_selection(self,p):
        c = cumsum(p)
        r = sum(p)*random.rand()
        ind = argwhere(r <= c)
        return ind[0][0]    

'''
-------------------Descomentar quando atividade do Grid voltar
class Grid:
    def __init__(self):

        # criando um grid 5x6
        self.gridv = array([[17.5, 13],[42.5, 13], [67.5, 13],[92.5, 13], [117.5, 13],[142.5, 13],
                      [17.5, 39],[42.5, 39], [67.5, 39],[92.5, 39], [117.5, 39],[142.5, 39],
                      [17.5, 65],[42.5, 65], [67.5, 65],[92.5, 65], [117.5, 65],[142.5, 65],
                      [17.5, 91],[42.5, 91], [67.5, 91],[92.5, 91], [117.5, 91],[142.5, 91],
                      [17.5, 117],[42.5, 117], [67.5, 117],[92.5, 117], [117.5, 117],[142.5, 117] ])

        # definindo os angulos de cada grid
        self.AttitudeGrid = array([-pi/2, 0.47282204, 0.56910571, 0.70991061, 0.9279823, 1.27818735,
                             -pi/2, 0.29463669, 0.35945951, 0.46006287, 0.63557154, 1.0038244,
                             0.06148337, 0.07225452, 0.08759046, 0.11115443, 0.15188589, 0.23793116,
                             pi/2, -0.29463669, -0.35945951, -0.46006287, -0.63557154, -1.0038244,
                             pi/2,  -0.47282204, -0.56910571, -0.70991061, -0.9279823, -1.27818735])
        self.robotGridPos = zeros(3)
        self.ballGridPos = 0

    def update(self, robot0, robot1, robot2, ball):

        # encontrando o indice em que cada robo e a bola se encontra
        index0 = argmin(distance.cdist(self.gridv, [robot0.xPos, robot0.yPos]))
        index1 = argmin(distance.cdist(self.gridv, [robot1.xPos, robot1.yPos]))
        index2 = argmin(distance.cdist(self.gridv, [robot2.xPos, robot2.yPos]))
        indexb = argmin(distance.cdist(self.gridv, [ball.xPos, ball.yPos]))

        # Atualizando os valores
        self.robotGridPos = array([index0, index1, index2])
        self.ballGridPos = indexb

    def bestGridMov():

        # Posição dos robôs
        pos0 = self.gridv[index[0]]
        pos1 = self.gridv[index[1]]
        pos2 = self.gridv[index[2]]

        # Lista dos grids mais próximos de cada robô
        listAux0 = distance.cdist(self.gridv, self.gridv[pos0]) # calcula a distancia

        # Removendo o valor 0 da lista de distancias
        zeroId = where(listAux0 == 0)
        listAux0[zeroId] = 1000
        listAux0[zeroId] = listAux0.min()

        listId0 = where(list0Aux <= 37) # encontra o indice dos valores min
        # salva a posição dos valores min
        list0 = []
        for index in listId0[0]:
            list0.append(self.gridv[index])

        listAux1 = distance.cdist(self.gridv, self.gridv[pos1])

        zeroId = where(listAux1 == 0)
        listAux1[zeroId] = 1000
        listAux1[zeroId] = listAux1.min()

        listId1 = where(listAux1 <= 37)

        list1 = []
        for index in listId1[0]:
            list1.append(self.gridv[index])

        listAux2 = distance.cdist(self.gridv, self.gridv[pos2])

        zeroId = where(listAux2 == 0)
        listAux2[zeroId] = 1000
        listAux2[zeroId] = listAux2.min()

        listId2 = where(listAux2 <= 37)
        list2 = []
        for index in listId2[0]:
            list0.append(self.gridv[index])

        #Verifica se a posição que ele vai se mover já tem algum robô

        if self.robotGridPos[1] in listId0:
            listId0n = delete(listId0[0], where(listId0 == self.robotGridPos[1]))
            list0 = delete(list0, where(listId0 == self.robotGridPos[1]), axis = 0)
        if self.robotGridPos[2] in listId0:
            listId0 = delete(listId0[0], where(listId0 == self.robotGridPos[2]))
            list0 = delete(list0, where(listId0 == self.robotGridPos[2]), axis = 0)

        if self.robotGridPos[0] in listId1:
            listId1n = delete(listId1[0], where(listId1 == self.robotGridPos[0]))
            list1 = delete(list1, where(listId1 == self.robotGridPos[0]), axis = 0)
        if self.robotGridPos[2] in listId1:
            listId1 = delete(listId1[0], where(listId1 == self.robotGridPos[2]))
            list1 = delete(list1, where(listId1 == self.robotGridPos[2]), axis = 0)

        if self.robotGridPos[0] in listId2:
            listId2n = delete(listId2[0], where(listId2 == self.robotGridPos[0]))
            list2 = delete(list2, where(listId2 == self.robotGridPos[0]), axis = 0)
        if self.robotGridPos[1] in listId2:
            listId2 = delete(listId2[0], where(listId2 == self.robotGridPos[1]))
            list2 = delete(list2, where(listId2 == self.robotGridPos[1]), axis = 0)

        # Encontrando qual grid é o mais próximo da bola
        targetId0 = argmin(distance.cdist(list0, self.gridv[indexb]))
        target0 = self.gridv[listId0n[0][targetId0]]

        targetId1 = argmin(distance.cdist(list1, self.gridv[indexb]))
        target1 = self.gridv[listId1n[0][targetId1]]

        targetId2 = argmin(distance.cdist(list2, self.gridv[indexb]))
        target2 = self.gridv[listId2n[0][targetId2]]

    #def doInGrid():
'''
