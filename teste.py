import numpy as np
import rospy
from simClasses import GA

'''
pos_x = [15,47.5,85,122.5,155,122.5,85,47.5]
pos_y = [65,115,120,115,65,25,20,25]
pos_ang = [0,0,180,180,180,180,180,0]

d = np.zeros(len(pos_x))
ang = np.zeros(len(pos_x))
t = np.zeros(len(pos_x))

for i in range(len(pos_x)):

	d[i], ang[i], t[i] = simulation.Run(pos_x[i], pos_y[i], pos_ang[i])

	print("d: ", d)
	print("ang: ", ang)
	print("t: ", t)
'''
ga_univector = GA(5,0,10,500,20)

pop = ga_univector.initialize_pop()

print(pop)
