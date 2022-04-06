from numpy import cos, sin, arctan2, sqrt, sign, pi, delete, append, array

from behaviours import Univector
from corners import target_in_corner


'''
Input: Robot object, Target object, Flag to activate Obstacle Avoidance, Obstacle object,
       Constants n and d of Univector, Flag to activate Hiperbolic Field
Description: Estimate of robot desired angle in projection ( x + dl, y + dl )
Output: stp_theta -> Angle referring to robot projection (float)
'''
def approx(robot, target, avoid_obst=True, obst=None, n=8, d=2, field_is_hiperbolic=True):
    navigate = Univector()  # Defines the navigation algorithm
    dl = 0.000001  # Constant to approximate phi_v

    x = robot.xPos  # Saving (x,y) coordinates to calculate phi_v
    y = robot.yPos
    robot.xPos = robot.xPos + dl * cos(robot.theta)  # Incrementing robot (x,y) position
    robot.yPos = robot.yPos + dl * sin(robot.theta)

    if avoid_obst:                                                          # If obstacle avoidance is activated
        if field_is_hiperbolic:                                             # Use of the Hyperbolic field
            stp_theta = navigate.univec_field_h(robot, target, obst)        # Computing a step Theta to determine phi_v
        else:                                                               # Use of the old field
            stp_theta = navigate.univec_field_n(robot, target, obst, n, d)  # Computing a step Theta to determine phi_v
    else:
        if field_is_hiperbolic:                                             # Use of the Hyperbolic field
            stp_theta = navigate.hip_vec_field(robot, target)               # Computing a step Theta to determine phi_v
        else:                                                               # Use of the old field
            stp_theta = navigate.n_vec_field(robot, target, n, d, have_face=False) # Computing a step Theta to determine phi_v

    robot.xPos = x  # Returning original (x,y) coordinates
    robot.yPos = y

    return stp_theta


'''
Input: Robot object, Target object, Flag to activate Obstacle Avoidance, Obstacle object, Constants n and d of Univector,
       Flag to activate deceleration when approaching target, Flag to activate face swap, Flag to activate Hiperbolic Field
Description: Function to control the robot with or without obstacle avoidance
Output: v -> Linear Velocity (float)
        w -> Angular Velocity (float)
'''
def univec_controller(robot, target, avoid_obst=True, obst=None, n=8, d=2, stop_when_arrive=False, double_face=True,
                      field_is_hiperbolic=True):

    flagCorner, corner = target_in_corner(target, robot) # Checks if the robot is in some corner

    navigate = Univector()  # Defines the navigation algorithm
    dl = 0.000001  # Constant to approximate phi_v
    k_w = 1.8  # Feedback constant for angle error (k_w=1 means no gain)
    k_p = 1  # Proporcional constant for stopping when arrive in target (k_p=1 means no gain)

    '''
    Target angle estimation
    '''

    # Angle correction if robot face is inverted
    if robot.face == -1:
        robot.theta = arctan2(sin(robot.theta - pi), cos(robot.theta - pi))

    # Navigation: Go-to-Goal + Avoid Obstacle Vector Field
    if avoid_obst: # If obstacle avoidance is activated
        if field_is_hiperbolic:                                             # Use of the Hyperbolic field
            des_theta = navigate.univec_field_h(robot, target, obst)        # Desired angle w/ go-to-goal and avoid obstacle vector field
        else:                                                               # Use of the old field
            des_theta = navigate.univec_field_n(robot, target, obst, n, d)  # Desired angle w/ go-to-goal and avoid obstacle vector field

    # Navigation: Go-to-Goal Vector Field
    else:
        if field_is_hiperbolic:                                                     # Use of the Hyperbolic field
            des_theta = navigate.hip_vec_field(robot, target)                       # Desired angle w/ go-to-goal
        else:                                                                       # Use of the old field
            des_theta = navigate.n_vec_field(robot, target, n, d, have_face=False)  # Desired angle w/ go-to-goal

    '''
    Controller estimation
    '''

    stp_theta = approx(robot, target, avoid_obst, obst, n, d, field_is_hiperbolic) # Desired angle prediction
    phi_v = arctan2(sin(stp_theta - des_theta),     # Difference between the prediction and current angle
                    cos(stp_theta - des_theta)) / dl
    theta_e = which_face(robot, target, des_theta, double_face) # Angle error

    # Controller velocities v1, v2, v3 estimation

    v1 = (2 * robot.vMax - robot.LSimulador * k_w * sqrt(abs(theta_e))) / (2 + robot.LSimulador * abs(phi_v))
    v2 = (sqrt(k_w ** 2 + 4 * robot.rMax * abs(phi_v)) - k_w * sqrt(abs(theta_e))) / (2 * abs(phi_v) + dl)

    if stop_when_arrive:
        v3 = k_p * robot.dist(target)
    else:
        v3 = robot.vMax

    if stop_when_arrive and robot.arrive(): # If robot needs stop when arrive target
        v = 0
        w = 0
    else:
        v = min(abs(v1), abs(v2), abs(v3))  # Controller velocities v and w
        w = v * phi_v + k_w * sign(theta_e) * sqrt(abs(theta_e))

    # Some code to store the past position, orientation and velocity

    #robot.v=v
    robot.pastPose = delete(robot.pastPose, 0, 1)  # Deleting the first column
    robot.pastPose = append(robot.pastPose, array(
        [[round(robot.xPos)], [round(robot.yPos)], [round(float(robot.theta))], [round(float(v))]]), 1)

    return v, w


# TODO #3 Check the need for flagTrocaFace - lock the face swap in obstacle avoidance
'''
Input: Robot object, Target object, Desired angle, Flag to activate face swap
Description: Defines de better face to robot movement and estimate angle error
Output: theta_e -> Angle error (float)
'''
def which_face(robot, target, des_theta, double_face):
    theta_e = arctan2(sin(des_theta - robot.theta), cos(des_theta - robot.theta))  # Error estimation with current face

    if (abs(theta_e) > pi / 2 + pi / 12) and (
            not robot.flagTrocaFace) and double_face:  # If the angle is convenient for face swap
        robot.face = robot.face * (-1)  # Swaps face
        robot.theta = arctan2(sin(robot.theta + pi), cos(robot.theta + pi))  # Angle re-estimate
        theta_e = arctan2(sin(des_theta - robot.theta), cos(des_theta - robot.theta))  # Error angle re-estimate

    return theta_e
