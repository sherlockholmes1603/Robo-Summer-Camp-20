import pybullet as p
import time
import os
import pybullet_data


physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")

file_path = os.getcwd()

#sphere = "/sphere.urdf"
p.setGravity(0,0,-10)


def fibon(i):
    if((i==1) or (i==2)):
        return 1
    else:
        return fibon(i-1) + fibon(i-2)


def sphere_load(i):
    no_of_sphere = fibon(i)
    for j in range(no_of_sphere):
        p.loadURDF("sphere.urdf",[0.5*j,0,5])
    return no_of_sphere

a = 0
b = True
wave = 1
while b:
    no_of_sphere = sphere_load(wave)
    for i in range(400):
        p.stepSimulation()
        a += 1
        if a>10000:
            b = False
            break
        print(i)
        time.sleep(1/240)
    print(wave," ",no_of_sphere)
    wave +=1

p.disconnect()
