
import pybullet as p
import pybullet_data
import os
'''
urdf file in the same folder as that of the python script
'''
file_path = os.getcwd()

#Enter the file name with a '/' in front of it 
file_name = "/drone.urdf"
'''
these comands are explained in detail in the next subpart
for now u can directly use it to visualize the model
'''
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_path+file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 5], [0, 0, 0, 0.707])
p.setGravity(0,0,0)

while(True):
	p.stepSimulation()
