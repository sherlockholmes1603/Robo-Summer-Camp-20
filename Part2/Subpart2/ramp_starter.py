import pybullet as p
import pybullet_data
import time
a = int(input("Enter any natural number for Velocity Control and 0 for Torque Control: "))
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

'''
1.print Number of Joints and joint info and state

2.Get the user input about the control function they 
want to simulate and see.(its upto you, it can be a string / int anything but just leave
a comment with the legend to your menu)

'''
joint_info = p.getNumJoints(husky)
print("Total number of joint info:  ",joint_info)
for i in range(joint_info):
	print("Joint info: ",i+1," = ",p.getJointInfo(husky,i))

p.enableJointForceTorqueSensor(husky,2)
p.enableJointForceTorqueSensor(husky,3)
p.enableJointForceTorqueSensor(husky,4)
p.enableJointForceTorqueSensor(husky,5)
#function to be filled to implement torque control
def Torque_control(maxForce):

	# find this value to climb the ramp without sliping and topling
	 
	p.setJointMotorControl2(husky, 2, p.TORQUE_CONTROL, force = maxForce)
	p.setJointMotorControl2(husky, 3, p.TORQUE_CONTROL, force = maxForce)
	p.setJointMotorControl2(husky, 4, p.TORQUE_CONTROL, force = maxForce)
	p.setJointMotorControl2(husky, 5, p.TORQUE_CONTROL, force = maxForce)
	'''
	this function should have the setJointMotorControl in TORQUE_CONTROL configuration
    with forc = optimal_force_value
    ''' 



#function to be filled to implement velocity control
def Velocity_control(maxForce,optimal_velocity_value):
	# Keep a constant non zero value for maxForce and try getting the velocity that makes it climb the ramp.
	

	# find this value to climb the ramp without sliping
	p.setJointMotorControl2(husky,2,p.VELOCITY_CONTROL,targetVelocity = optimal_velocity_value,force = maxForce)
	p.setJointMotorControl2(husky,3,p.VELOCITY_CONTROL,targetVelocity = optimal_velocity_value,force = maxForce)
	p.setJointMotorControl2(husky,4,p.VELOCITY_CONTROL,targetVelocity = optimal_velocity_value,force = maxForce)
	p.setJointMotorControl2(husky,5,p.VELOCITY_CONTROL,targetVelocity = optimal_velocity_value,force = maxForce)


	'''
	this function should have the setJointMotorControl in VELOCITY_CONTROL configuration
	with targetvelocity = optimal_velocity_value 
	'''

i=0

while (1):
	time.sleep(.01)
	'''
	1.Here call either the Torque_control function or Velocity_control 
	  function according to the initial user choice and apply the optimal velocity/Torque
	  to the motors that you have found by experimentation.

	2.print base state and velocity 100 iteration steps once.
	'''
	if(a):
		Velocity_control(500,-3)
	else:
		Torque_control(-250)
	if(i%100==0):
		print("Base velocity : ",p.getBaseVelocity(husky))
		print("Base state : ",p.getLinkState(husky,0))
	i+=1
	p.stepSimulation()



p.disconnect()