import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0.0,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube1 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
cubeStartPos = [0.1,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube2 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
cubeStartPos = [0.2,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube3 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
cubeStartPos = [0.3,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube4 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
cubeStartPos = [0.4,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube5 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
cubeStartPos = [0.5,0,2]
cubeStartOrie = p.getQuaternionFromEuler([0,0,0])
cube6 = p.loadURDF("/cube.urdf",cubeStartPos,cubeStartOrie,useFixedBase = 1)
sphereStartPos = [0.05,0,1]
sphereStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId1 = p.loadURDF("/sphere.urdf",sphereStartPos, sphereStartOrientation)
p.changeDynamics(boxId1,-1,lateralFriction=0.0, restitution = 1)
p.createConstraint(cube1, -1, boxId1,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
p.createConstraint(cube2, -1, boxId1,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
sphereStartPos = [0.15,0,1]
sphereStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId2 = p.loadURDF("/sphere.urdf",sphereStartPos, sphereStartOrientation)
p.changeDynamics(boxId2,-1,lateralFriction=0.0, restitution = 1)
p.createConstraint(cube2, -1, boxId2,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
p.createConstraint(cube3, -1, boxId2,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
sphereStartPos = [0.25,0,1]
sphereStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId3 = p.loadURDF("/sphere.urdf",sphereStartPos, sphereStartOrientation)
p.changeDynamics(boxId3,-1,lateralFriction=0.0, restitution = 1)
p.createConstraint(cube3, -1, boxId3,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
p.createConstraint(cube4, -1, boxId3,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
sphereStartPos = [0.35,0,1]
sphereStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId4 = p.loadURDF("/sphere.urdf",sphereStartPos, sphereStartOrientation)
p.changeDynamics(boxId4,-1,lateralFriction=0.0, restitution = 1)
p.createConstraint(cube4, -1, boxId4,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
p.createConstraint(cube5, -1, boxId4,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
sphereStartPos = [0.45,0,1]
sphereStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId5 = p.loadURDF("/sphere.urdf",sphereStartPos, sphereStartOrientation)
p.changeDynamics(boxId5,-1,lateralFriction=0.0, restitution = 1)
p.createConstraint(cube5, -1, boxId5,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
p.createConstraint(cube6, -1, boxId5,-1, p.JOINT_POINT2POINT, [1, 0, 0], [0, 0, 0], [0, 0, 1])
i = 0
while True:
    if i<150:
        p.applyExternalTorque(boxId1,-1,[0,100,0],flags = 1)
    p.stepSimulation()
    i+=1
    time.sleep(0.01)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
p.disconnect()