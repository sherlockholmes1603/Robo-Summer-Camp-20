import pybullet as p
import time
import pybullet_data
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
# sphere= p.loadURDF("sphere")
p.setGravity(0, 0, -10)
fibo = [1, 1]


def load_drops(wave_no):
    global fibo
    if wave_no > 2:
        new_no = fibo[len(fibo) - 1] + fibo[len(fibo) - 2]
        fibo.append(new_no)
        for i in range(new_no):
            p.loadURDF("sphere.urdf", [(0.5 * i), 0, 5], [0, 0, 0, 1])
    else:
        p.loadURDF("sphere.urdf", [0, 0, 5], [0, 0, 0, 1])


wave = 1
while True:
    load_drops(wave)
    for i in range(400):
        p.stepSimulation()
        print(i)
        time.sleep(1. / 240.)
    wave += 1
