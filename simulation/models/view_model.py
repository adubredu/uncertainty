import pybullet as p
import time
import pybullet_data
import math
import cv2
import numpy as np 
from detecto import core, utils, visualize
from PIL import Image


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())


p.setGravity(0,0,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
tableid = p.loadURDF('table/table.urdf',[0,0,0],p.getQuaternionFromEuler([0,0,0]))

p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')
bottle = p.loadURDF('bottle/bottle.urdf',[-0.5,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
apple = p.loadURDF('apple/apple.urdf',[0.25,0,0.65],p.getQuaternionFromEuler([3.14,0,0]))
coke = p.loadURDF('coke/coke.urdf',[0.25,0.3,0.65],p.getQuaternionFromEuler([0,0,0]))
orange = p.loadURDF('orange/orange.urdf',[0,0.25,0.65],p.getQuaternionFromEuler([0,0,0]))
pepsi = p.loadURDF('pepsi/pepsi.urdf',[0.,0,0.66],p.getQuaternionFromEuler([0,0,0]))
cereal = p.loadURDF('cereal/cereal.urdf',[2.5,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))
lysol = p.loadURDF('lysol/lysol.urdf',[3,0,0.75],p.getQuaternionFromEuler([0,0,0]))
lipton = p.loadURDF('lipton/lipton.urdf',[3.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
nutella = p.loadURDF('nutella/nutella.urdf',[4,0,0.65],p.getQuaternionFromEuler([0,0,0]))
ambrosia = p.loadURDF('ambrosia/ambrosia.urdf',[4.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
oreo = p.loadURDF('oreo/oreo.urdf',[5,0,0.65],p.getQuaternionFromEuler([3.14,0,0]))
milk = p.loadURDF('milk/milk.urdf',[5.5,0,0.65],p.getQuaternionFromEuler([0,0,0]))
banana = p.loadURDF('banana/banana.urdf',[6,0,0.65],p.getQuaternionFromEuler([0,0,0]))

p.setRealTimeSimulation(1)

def normalize_scene_weights(scene):
	norm_scene = {}

	for item in scene:
		names=[]; weights=[];coord=[]
		norm_scene[item]=[]
		for name,wt,cd in scene[item]:
			names.append(name)
			weights.append(wt)
			coord.append([int(c) for c in cd])

		summ = np.sum(weights)
		norm_wt = weights/summ
		for name, wt, cd in zip(names, norm_wt, coord):
			norm_scene[item].append((name,wt,cd))

	return norm_scene



while 1:
	viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 2],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

	projectionMatrix = p.computeProjectionMatrixFOV(
	    fov=60.0,
	    aspect=1.0,
	    nearVal=0.02,
	    farVal=3.1)

	width, height, rgbImg, depthImg, segImg = p.getCameraImage(
	    width=480, 
	    height=480,
	    viewMatrix=viewMatrix,
	    projectionMatrix=projectionMatrix,
	    shadow=True,
	    renderer=p.ER_BULLET_HARDWARE_OPENGL)
	model = core.Model.load('/home/developer/garage/grocery_detector.pth', \
		['ambrosia','apple','banana','bottle','cereal','coke',\
				'lipton','lysol','milk','nutella','orange','oreo','pepsi'])
	
	rgbImg = Image.fromarray(rgbImg).convert('RGB')
	predictions = model.predict(rgbImg)
	camera_view = cv2.cvtColor(np.array(rgbImg), cv2.COLOR_RGB2BGR)
	# camera_view = np.array(rgbImg)

	labels, boxes, scores = predictions
	boxes = boxes.numpy()
	scores = scores.numpy()
	# print(labels)
	# visualize.show_labeled_image(camera_view, boxes, labels)
	num_observed = len(labels)
	observed = {}
	preds = []
	idd = 0
	for i in range(num_observed):
		dicts = {'name':labels[i],
				 'id':idd,
				'coordinates': boxes[i],
				 'confidence':scores[i],
				 'color': (np.random.randint(255),\
				 		np.random.randint(255),\
				 		np.random.randint(255))
				 }
		preds.append(dicts)
		observed[labels[i]] = dicts
		idd+=1

	clusters = {}
	for box in preds:
		fit = False
		mid = ((box['coordinates'][0] + box['coordinates'][2])/2, \
			(box['coordinates'][1] + box['coordinates'][3])/2)

		for key in clusters:
			kcd = key.split('_')
			ikcd = [float(i) for i in kcd]
			dist = np.sqrt((ikcd[0] - mid[0])**2 + (ikcd[1]-mid[1])**2)
			if dist < 10:
				clusters[key].append(box)
				fit = True
				break
		if not fit:
			clusters[str(mid[0])+'_'+str(mid[1])] = [box]
	# print(len(clusters))

	scene = {}
	for key in clusters:
		weights = [box['confidence'] for box in clusters[key]]
		maxind = np.argmax(weights)
		maxweightedbox = clusters[key][maxind]
		scene[maxweightedbox['name']] = [(box['name'], box['confidence'], \
						box['coordinates']) for box in clusters[key]]

	nscene = normalize_scene_weights(scene)
	# print(nscene)

	# print('**********************************')

	for item in nscene:
		for nm, cf, cd in nscene[item]:
			if nm == item:
				color = (np.random.randint(255),\
				 		np.random.randint(255),\
				 		np.random.randint(255))
				camera_view = cv2.rectangle(camera_view, (cd[0],
				 cd[1]), (cd[2], cd[3]),color , 1)
				cv2.putText(camera_view, nm+':'+str(round(cf,2)), (cd[0],cd[1]-10),\
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,2)





	# for item in observed:
	# 	camera_view = cv2.rectangle(camera_view, (observed[item]['coordinates'][0],
	# 			 observed[item]['coordinates'][1]), (observed[item]['coordinates'][2], 
	# 			 observed[item]['coordinates'][3]), observed[item]['color'], 1)



	# print(observed)
	# print('\n\n')
	# cv2.rectangle(camera_view, (1,1), (100,100),(0,255,0),1)
	cv2.imshow('Grocery Item detection', camera_view)
	if cv2.waitKey(1) & 0xFF == ord('q'):
	    break
	p.stepSimulation()
	# time.sleep(300)