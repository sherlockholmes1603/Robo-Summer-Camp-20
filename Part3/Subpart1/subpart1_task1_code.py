import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
targetVel = 3  #rad/s
maxForce = 100 #Newton
i=0
#p.applyExternalForce(car,3,[100,0,0],)
while (1):
    i+=1
    if(i%100000 == 0):
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
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = 2*targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = 2*targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = 2*targetVel,force = maxForce)
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = 2*targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()

        if (k == ord('a') and (v & p.KEY_IS_DOWN)):
            targetVel += 1
            time.sleep(1)
            p.stepSimulation()

        if (k == ord('d') and (v & p.KEY_IS_DOWN)):
            targetVel -= 1
            time.sleep(1)
            p.stepSimulation()

        if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
            p.changeVisualShape(car, -1, rgbaColor=[1, 1, 1, 0.1])
            width = 512
            height = 512

            fov = 60
            aspect = width / height
            near = 0.02
            far = 5
            #pos = p.getBasePositionAndOrientation(car)
            # pos[0] = float(pos[0])
            # pos[1] = float(pos[1])
            # pos[2] = float(pos[2])
            #print(pos)
            

            x = p.getLinkState(car,2)
            y = p.getLinkState(car,4)
            z = p.getLinkState(car,8)
            print(x)
            print(y)
            print(z)
            view_matrix = p.computeViewMatrix( z[0],  [z[0][0] + 10*(x[0][0] - y[0][0]),z[0][1] + 10*(x[0][1] - y[0][1]),z[0][2] + 10*(x[0][2] - y[0][2])], [0, 0, 2])
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

            # Get depth values using the OpenGL renderer
            images = p.getCameraImage(width,
                                    height,
                                    view_matrix,
                                    projection_matrix,
                                    shadow=True,
                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            cv2.imshow('rgb',rgb_opengl)
            cv2.waitKey(0)
            cv2.destroyAllWindows()



p.getContactPoints(car)

p.disconnect()