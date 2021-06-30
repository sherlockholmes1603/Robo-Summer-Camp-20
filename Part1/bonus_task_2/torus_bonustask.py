import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])
p.setGravity(0, 0, -9000)

tex = p.loadTexture("uvmap.png")
planeId = p.loadURDF("plane.urdf", [0,0,-2])

#boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

cubeStartOrientation = p.getQuaternionFromEuler([0,100,10])
i = 0
while(i<3):
    bunnyId = p.loadSoftBody("torus/torus_textured.obj",[0,1 , i] ,cubeStartOrientation ,simFileName="torus.vtk", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)
    p.changeVisualShape(bunnyId, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0) 
    i = i + 1

a=0
while( a < 4 ):
    bunny3 = p.loadURDF("torus_deform.urdf", [0,1 + a/2,-0.5 + a], flags=p.URDF_USE_SELF_COLLISION)
    a = a + 1
    p.changeVisualShape(bunny3, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=20)
p.setRealTimeSimulation(0)

while p.isConnected():
  p.stepSimulation()
  p.getCameraImage(320,200)
  p.setGravity(0,0,-10)