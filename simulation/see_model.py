#! /usr/bin/env python3
import pybullet as p
import time
import pybullet_data
import math
from grocery_items import Grocery_item, Shopping_List


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())

p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath('models')

sl = Shopping_List(p)

for i in range(100):
	p.stepSimulation()
	time.sleep(10)