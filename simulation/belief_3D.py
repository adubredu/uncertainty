#! /usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import math


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf") 
h = 1
cubeStartPos = [0,0,h]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0]) 
gripperOrientation = p.getQuaternionFromEuler([0,3.14,3.14])
# p.setAdditionalSearchPath('/home/developer/pybullet/ss-pybullet/models/pr2_description')
boxId = p.loadURDF("cube_small.urdf",[5,1,0.5], cubeStartOrientation)
tableId = p.loadURDF("table/table.urdf",[0,0,0], cubeStartOrientation)
lgripperId = p.loadURDF("gripper/wsg50_one_motor_gripper_left_finger.urdf",[-0.05,0,1], gripperOrientation)
rgripperId = p.loadURDF("gripper/wsg50_one_motor_gripper_right_finger.urdf",[0.05,0,1], gripperOrientation)
trayId = p.loadURDF("tray/traybox.urdf",[0,0,0.625], cubeStartOrientation)


def pick_up(targetID,width,breadth):
	(olx,oly,olz), _ = p.getBasePositionAndOrientation(lgripperId)
	(orx,ory,orz), _ = p.getBasePositionAndOrientation(rgripperId)
	(lx,ly,lz), _ = p.getBasePositionAndOrientation(lgripperId)
	(rx,ry,rz), _ = p.getBasePositionAndOrientation(rgripperId)
	(tx,ty,tz), _ = p.getBasePositionAndOrientation(targetID)
	delta = 0.0001
	while math.fabs(lx - (tx-(width/2)))>delta \
	 or math.fabs(rx - (tx+(width/2)))>delta:
		if lx < (tx-(width/2)):
			lx+=delta
		else:
			lx-=delta
		if rx < (tx+(breadth/2)):
			rx+=delta
		else:
			rx-=delta
		time.sleep(1./10000.)
		p.resetBasePositionAndOrientation(lgripperId, [lx,ly,lz], gripperOrientation)
		p.resetBasePositionAndOrientation(rgripperId, [rx,ry,rz], gripperOrientation)
		p.stepSimulation()
	while math.fabs(ly - ty)>delta or math.fabs(ry - ty)>delta:
		if ly < ty:
			ly+=delta
		else:
			ly-=delta
		ry = ly
		time.sleep(1./10000.)
		p.resetBasePositionAndOrientation(lgripperId, [lx,ly,lz], gripperOrientation)
		p.resetBasePositionAndOrientation(rgripperId, [rx,ry,rz], gripperOrientation)
		p.stepSimulation()
	while math.fabs(lz - (tz+0.05))>delta:
		if lz < (tz+0.05):
			lz+=delta
		else:
			lz-=delta
		rz = lz
		time.sleep(1./10000.)
		p.resetBasePositionAndOrientation(lgripperId, [lx,ly,lz], gripperOrientation)
		p.resetBasePositionAndOrientation(rgripperId, [rx,ry,rz], gripperOrientation)
		p.stepSimulation()




for i in range(10):
	# p.stepSimulation()
	time.sleep(1./420)
	pick_up(boxId,width=0.07,breadth=0.07)

	# p.resetBasePositionAndOrientation(boxId, [h,0,1], cubeStartOrientation)
	# h+=.01

(x,y,z), cubeOrn = p.getBasePositionAndOrientation(boxId)
print((x,y,z))
p.disconnect()