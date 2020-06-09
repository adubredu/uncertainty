import pybullet as p
import time
import pybullet_data
import math


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())


p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
tableid = p.loadURDF('table/table.urdf',[0,0,0],p.getQuaternionFromEuler([0,0,0]))

p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')
bottle = p.loadURDF('bottle/bottle.urdf',[0,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
apple = p.loadURDF('apple/apple.urdf',[0.5,0,0.65],p.getQuaternionFromEuler([3.14,0,0]))
coke = p.loadURDF('coke/coke.urdf',[1.0,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
orange = p.loadURDF('orange/orange.urdf',[1.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
pepsi = p.loadURDF('pepsi/pepsi.urdf',[2,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
cereal = p.loadURDF('cereal/cereal.urdf',[2.5,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
lysol = p.loadURDF('lysol/lysol.urdf',[3,0,0.65],p.getQuaternionFromEuler([0,0,0]))
lipton = p.loadURDF('lipton/lipton.urdf',[3.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
nutella = p.loadURDF('nutella/nutella.urdf',[4,0,0.65],p.getQuaternionFromEuler([0,0,0]))
ambrosia = p.loadURDF('ambrosia/ambrosia.urdf',[4.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
oreo = p.loadURDF('oreo/oreo.urdf',[5,0,0.65],p.getQuaternionFromEuler([3.14,0,0]))
milk = p.loadURDF('milk/milk.urdf',[5.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
banana = p.loadURDF('banana/banana.urdf',[6,0,0.65],p.getQuaternionFromEuler([0,0,0]))

p.setRealTimeSimulation(1)


while 1:
	viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 3],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

	projectionMatrix = p.computeProjectionMatrixFOV(
	    fov=60.0,
	    aspect=1.0,
	    nearVal=0.02,
	    farVal=3.1)

	width, height, rgbImg, depthImg, segImg = p.getCameraImage(
	    width=224, 
	    height=224,
	    viewMatrix=viewMatrix,
	    projectionMatrix=projectionMatrix,
	    shadow=True,
	    renderer=p.ER_BULLET_HARDWARE_OPENGL)
	p.stepSimulation()
	# time.sleep(300)