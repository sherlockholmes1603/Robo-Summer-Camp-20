
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
targetVel = 1  #rad/s
maxForce = 100 #Newton
i=0
#p.applyExternalForce(car,3,[100,0,0],)
while (1):
    i+=1
    if(i%10000 == 0):
        print(targetVel)
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel*2,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel*2,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel*2,force = maxForce)
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel*2,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()

        if (k == ord('a') and (v & p.KEY_IS_DOWN)):
            targetVel += 1
            time.sleep(1)

        if (k == ord('d') and (v & p.KEY_IS_DOWN)):
            targetVel -= 1
            time.sleep(1)

        if (k == ord('r') and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            p.stepSimulation()



p.getContactPoints(car)

p.disconnect()
