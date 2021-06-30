import pybullet as p
import time
import pybullet_data
import os

file_path = os.getcwd()

physicsClient = p.connect(p.GUI)

dabba = "/dabba.urdf"
sphere = "/sphere.urdf"
dabbaStartPos = [0,0,1]
sphereStartPos = [2,2,1]
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robotd = p.loadURDF(file_path + dabba)
robots = p.loadURDF(file_path + sphere)
p.resetBasePositionAndOrientation(robotd,dabbaStartPos,[0,0,0,0.707])
p.resetBasePositionAndOrientation(robots,sphereStartPos,[0,0,0,0.707])
i=0

while True:
    i+=1
    if(i>98):
        i=0
        p.resetBasePositionAndOrientation(robotd,dabbaStartPos,[0,0,0,0.707])
        p.resetBasePositionAndOrientation(robots,sphereStartPos,[0,0,0,0.707])
    p.setGravity(i/1.414,i/1.414,0)
    dabbapos , dabbaors = p.getBasePositionAndOrientation(robotd)
    spherepos , sphereorn = p.getBasePositionAndOrientation(robots)
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()

