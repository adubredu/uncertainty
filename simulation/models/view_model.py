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
p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')
# objectid = p.loadURDF('bottle/bottle.urdf',[0,0,0.5],p.getQuaternionFromEuler([1.57,0,0]))
# objectid = p.loadURDF('apple/apple.urdf',[0,0,0.5],p.getQuaternionFromEuler([3.14,0,0]))
# objectid = p.loadURDF('coke/coke.urdf',[0,0,0.5],p.getQuaternionFromEuler([1.57,0,0]))
# objectid = p.loadURDF('orange/orange.urdf',[0,0,0.5],p.getQuaternionFromEuler([0,0,0]))
objectid = p.loadURDF('pepsi/pepsi.urdf',[0,0,0.5],p.getQuaternionFromEuler([1.57,0,0]))




for i in range(300):
	p.stepSimulation()
	time.sleep(300)