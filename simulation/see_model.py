#! /usr/bin/env python3
import pybullet as p
import time
import pybullet_data
import math
from grocery_items import Grocery_item, Shopping_List
from PIL import Image
import sys
import numpy as np
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())

p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath('models')
planeId = p.loadURDF("plane/plane.urdf")
tableid = p.loadURDF('table/table.urdf',[0,0,0],p.getQuaternionFromEuler([0,0,0]))
trayId = p.loadURDF("container/container.urdf", [-.5,.0,0.62],p.getQuaternionFromEuler([0,0,0]))
boundaries = p.getAABB(trayId)
lwh = np.array(boundaries[1])-np.array(boundaries[0])
print(lwh)
p.disconnect()

sl = Shopping_List(p)

# for i in range(100):
# 	p.stepSimulation()
# 	time.sleep(10)
while 1:
	viewMatrix = p.computeViewMatrix(
	cameraEyePosition=[0, -1., 2],
	cameraTargetPosition=[0, 0, 0],
	cameraUpVector=[0, 1, 0])

	projectionMatrix = p.computeProjectionMatrixFOV(
		fov=60.0,
		aspect=1.0,
		nearVal=0.02,
		farVal=3.1)

	width, height, rgbImg, depthImg, segImg = p.getCameraImage(
		width=640, 
		height=640,
		viewMatrix=viewMatrix,
		projectionMatrix=projectionMatrix,
		shadow=True,
		renderer=p.ER_BULLET_HARDWARE_OPENGL)
	
	
	rgbImg = Image.fromarray(rgbImg).convert('RGB')
	rgbImg.save('15-21.png')
	break
	camera_view = cv2.cvtColor(np.array(rgbImg), cv2.COLOR_RGB2BGR)
	cv2.imshow('Grocery Item detection', camera_view)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	p.stepSimulation()