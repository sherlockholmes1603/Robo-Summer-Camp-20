from numpy import angle
from math import *
import pybullet as p
import pybullet_data
import os
import time

file_name = "2R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0,0,-10)

'''
Note:

*these are the two end point of the line segment you want to 
 draw,so after finding the line segment you want to trace change 
 the two point below.

*the function addUserDebugLine just draws the line segment joining
point_A and point_B 

'''

point_A = [0,1,0]
point_B = [0,1,1]
p.addUserDebugLine(point_A,point_B,[1,0,0],2)

l1 = 1 #update the length of link-1,from the urdf
l2 = 1.005 #updare the length of link-2,from the urdf

def Forward_kinematics(angle_1,angle_2):

	'''
	This function should do the necessary trignometric 
	calculations using angle_1,angle_2,l1,l2 and resturn the [0,y,z] as the robot
	works space is the y-z plane
	'''
	y=0 # calculate y
	z=0 # calculate z
	return [0,y,z]

def Inverse_kinematics(target):
	'''
	This function should do the necessary trignometric 
	calculations using y ,z,l1,l2 and return angle_1 and angle_2 to 
	reach a given target of the form [0,y,z],as the robot 
	works space is th y-z plane.
	'''
	x = target[2]
	y = target[1]
	a = (x**2 + y**2 -l1**2 - l2**2)/(2*l1*l2)
	c = tanh(y/x)
	# print(a,b,c)
	angle_2 = -cosh(a)#calculate angle_2
	b = tanh((l2*sin(angle_2))/(l1 + l2*a))

	angle_1 = c - b  #calculate angle_1
	return angle_1,angle_2

'''
Write the equation of the line you are going to follow:-
Example, 

*it might be of the for z = m*y + c, so in the
 while loop below increament y = y + dy and find new z
 and give the target ie [0,y,z] to the Inverse_Kinematics 
 function.

*so trace the line from point_A to point_B and reset position 
 to point_A and continue the same process infinitely.  

'''
i = 1
y = 1
z = 0

while(True):

	z += 0.001
	if z>=1:
		z = 0
	# angle = []
	angle = p.calculateInverseKinematics(robot,2,[0,y,z])
	# angle.append(angle1)
	# angle.append(angle2)
	# if (i%100 == 0):
	# 	print(y, z)
	# 	print("angle_1: ", angle[0], "angle_2:", angle[1])
	# 	print("Position: ",p.getLinkState(robot,1)[0])
	# if abs(angle_2)>=1 or abs(angle_1)>=1:
	# 	angle_1 = 0
	# 	angle_2 = 0
	# 	y = 0
	# print(angle_1, angle_2)

	p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=0,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle[0],
                            force=500)

	p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=1,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle[1],
                            force=500)

	p.stepSimulation()
	time.sleep(0.01)
