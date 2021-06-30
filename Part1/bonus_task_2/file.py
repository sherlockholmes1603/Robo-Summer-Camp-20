import pybullet as p
from time import sleep
import pybullet_data
import os
physicsClient = p.connect(p.GUI)                        ## connect to server
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)         ## Flag to enable Deformable bodies

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

ball1Id = p.loadSoftBody( "ball.obj",
                          simFileName = "ball.vtk", 
                          basePosition = [0,0,5], 
                          scale = 0.5, 
                          mass = 4,
                          useNeoHookean = 1,
                          NeoHookeanMu = 400,
                          NeoHookeanLambda = 600,
                          NeoHookeanDamping = 0.001,
                          useSelfCollision = 1,
                          frictionCoeff = .5,
                          collisionMargin = 0.001)      ## Loading softbody with given description

ball2Id = p.loadSoftBody( "ball.obj",
                          simFileName = "ball.vtk", 
                          basePosition = [0,0,0], 
                          scale = 0.5, 
                          mass = 4,
                          useNeoHookean = 1,
                          NeoHookeanMu = 400,
                          NeoHookeanLambda = 600,
                          NeoHookeanDamping = 0.001,
                          useSelfCollision = 1,
                          frictionCoeff = .5,
                          collisionMargin = 0.001)      ## Loading another softbody


p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)    ## Setting a limit for the resolution of
                                                        ## voxel to increase performance and decrease accuracy



while p.isConnected():
  p.stepSimulation()                                    ## Run Run Run!!!