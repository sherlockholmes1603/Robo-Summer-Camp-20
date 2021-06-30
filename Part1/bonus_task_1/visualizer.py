
import pybullet as p
import pybullet_data
import os
import time
'''
urdf file in the same folder as that of the python script
'''
file_path = os.getcwd()

#Enter the file name with a '/' in front of it 
file_name = "/final_robot.urdf"
'''
these comands are explained in detail in the next subpart
for now u can directly use it to visualize the model
'''
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_path+file_name)
botpos = [1,0,15]
botorie = p.getQuaternionFromEuler([0,0,0])
p.resetBasePositionAndOrientation(robot, botpos, botorie)
p.setGravity(0,0,10)

while(True):
	# time.sleep(10000000)
	p.stepSimulation()