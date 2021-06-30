import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

huskypos = [0, 0, 0.1]
target_block=p.loadURDF("block0.urdf",3,0,0)


Kp=1
Kd=10*240
last_error=0
PID_CONTROL=False

husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
p.createConstraint(husky, -1, -1, -1, p.JOINT_POINT2POINT, [0, 1, 0], [0, 0,0 ], [0, 0,0])
numJoints = p.getNumJoints(husky)
for joint in range(numJoints):
  print(p.getJointInfo(husky, joint))
maxForce = 200 #Newton.m
#camera should be facing in the direction of the husky


def turn(error, prev_error, hsv_lower, hsv_upper):
    baseSpeed = 0
    p.changeVisualShape(husky, -1, rgbaColor=[1, 1, 1, 0.1])
    width = 512
    height = 512

    fov = 60
    aspect = width / height
    near = 0.02
    far = 5
    #targetVel_L = speed
    #targetVel_R = speed

    x = p.getLinkState(husky,3)
    y = p.getLinkState(husky,5)
    z = p.getLinkState(husky,8)
    # print(x)
    # print(y)
    # print(z)
    view_matrix = p.computeViewMatrix( z[0], [z[0][0] + 10*(x[0][0] - y[0][0]) , z[0][1] + 10*(x[0][1] - y[0][1]), z[0][2] + 10*(x[0][2] - y[0][2])], [0, 0, 1])
    # view_matrix = p.computeViewMatrix( z[0], [d1, d2, d3] , [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)


            # Get depth values using the OpenGL renderer
    images = p.getCameraImage(width,
                            height,
                            view_matrix,
                            projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb_opengl = np.reshape(images[2][:,:,:3], (height, width, 3))
    cv2.imwrite("cam.jpg",rgb_opengl)
    rgb_opengl = cv2.imread("cam.jpg")
    rgb_opengl = cv2.resize(rgb_opengl,(50,50))
    hsv = cv2.cvtColor(rgb_opengl,cv2.COLOR_BGR2HSV)
    #hsv = cv2.medianBlur(hsv,3)
    final = cv2.inRange(hsv, hsv_lower, hsv_upper)
    # # print(rgb_opengl.shape," ",rgb_opengl.dtype)
    # cv2.imshow("segment",final)
    # cv2.waitKey(1)
    det_color = cv2.bitwise_and(rgb_opengl,rgb_opengl,mask=final)
    contours, _1 = cv2.findContours(final, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    targetVel_R = baseSpeed + error
    targetVel_L = baseSpeed - error
    if(len(contours)>=1):
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0])>100):
            M = cv2.moments(contours[0])
            if(M['m00']!=0):
                cx = int(M['m10']/M['m00'])
                print(cx, cv2.contourArea(contours[0]), len(contours))
                error = (cx - 25)
                pid_val = Kp*error + Kd*(error-prev_error)
                print(Kd*(error-prev_error))
                targetVel_R = baseSpeed + pid_val
                targetVel_L = baseSpeed - pid_val

    for joint in range(1,3):
        p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,(2*joint) +1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    return error
'''
tune the kp and kd from experiments, 
a hint set kd = 10*kp
'''


while (1):
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    if (PID_CONTROL):
    	# 1. Get the image feed, as in subpart-1.
    	hsv_lower=np.array([30,0,75])
    	hsv_upper=np.array([100,255,255])
    	# 2. using the above limits, mask the green colour 
      	#    and draw the area with maximuma contour and find 
      	#    its moments.
      	# 3. Calculate the error, and apply pid control to find the
      	#    optimal speed correction and call the turn function with
      	#    that speed.

      	# apply pid law to calulate this value,use Kp and Kd variable above
      	# and tune em properly.

        
    	speed_correction = 10
    	error = turn(speed_correction, last_error, hsv_lower, hsv_upper)
    	last_error=error #initialize accordingly
        
        
    for k, v in keys.items():

            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1* targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

                p.stepSimulation()
            if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-on")
                PID_CONTROL = True
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-off ,back to manual")
                PID_CONTROL = False
    time.sleep(1/240)
p.disconnect()
