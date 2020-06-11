#! /usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import math
import threading


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf") 



class Grocery_item:
    def __init__(self, x, y, z, orr, op, oy, urdf_path, width, 
                breadth, height, object_name, mass):
        self.x = x
        self.y = y
        self.z = z
        self.width = width 
        self.breadth = breadth 
        self.height = height 
        self.mass = mass 

        self.orr = orr 
        self.op = op 
        self.oy = oy 
        self.quat = p.getQuaternionFromEuler([self.orr,self.op,self.oy])
        self.name = object_name
        self.id = p.loadURDF(urdf_path, [self.x,self.y,self.z], self.quat)

    def update_object_position(self):
    	p.resetBasePositionAndOrientation(self.id, \
    		[self.x,self.y,self.z], self.quat)

    def get_position(self):
    	(x,y,z), _ = p.getBasePositionAndOrientation(self.id)
    	self.x = x; self.y = y; self.z = z;
    	return (x,y,z)


class Grocery_packing:
	def __init__(self):
		self.cube = Grocery_item(.5,0.,0.65, 0,0,0, "cube_small.urdf",0.07,0.07,0.07,'cube','light')
		self.table = Grocery_item(0,0,0, 0,0,0, "table/table.urdf",1,1,1,'table','heavy')
		self.lgripper = Grocery_item(0.2,0.0,1.5, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_left_finger.urdf",1,1,1,'lgripper','light')
		self.rgripper = Grocery_item(0.23,0.0,1.5, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_right_finger.urdf",1,1,1,'rgripper','light')
		p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')

		self.tray = Grocery_item(-.5,.0,0.62, 0,0,0, "container/container.urdf",1,1,1,'tray','heavy')

		self.items = {
						'cube':self.cube,
						'table':self.table,
						'lgripper':self.lgripper,
						'rgripper':self.rgripper,
						'tray': self.tray
		}
		self.delta = 0.01
		self.fps = 60
		self.scene_belief = None
		perception = threading.Thread(target=self.start_perception,args=(1,))
		perception.start()


	def refresh_world(self):
		for key in self.items:
			self.items[key].update_object_position()
		p.stepSimulation()


	def start_perception(self,x):
		import pybullet as p
		import time
		import pybullet_data
		import math
		import cv2
		import numpy as np 
		from detecto import core, utils, visualize
		from PIL import Image
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
			    fov=90.0,
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
			labels, boxes, scores = predictions
			boxes = boxes.numpy()
			scores = scores.numpy()
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

			scene = {}
			for key in clusters:
				weights = [box['confidence'] for box in clusters[key]]
				maxind = np.argmax(weights)
				maxweightedbox = clusters[key][maxind]
				scene[maxweightedbox['name']] = [(box['name'], box['confidence'], \
								box['coordinates']) for box in clusters[key]]

			self.scene_belief = normalize_scene_weights(scene)
			for item in self.scene_belief:
				for nm, cf, cd in self.scene_belief[item]:
					if nm == item and cf > 0.5 and cd[0]>240:
						color = (np.random.randint(255),\
						 		np.random.randint(255),\
						 		np.random.randint(255))
						camera_view = cv2.rectangle(camera_view, (cd[0],
						 cd[1]), (cd[2], cd[3]),color , 1)
						cv2.putText(camera_view, nm+':'+str(round(cf,2)), (cd[0],cd[1]-10),\
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,2)
			cv2.imshow('Grocery Item detection', camera_view)
			if cv2.waitKey(1) & 0xFF == ord('q'):
			    break
			p.stepSimulation()



	def pick_up(self,targetID):
		item = self.items[targetID]
		(olx,oly,olz) = self.lgripper.get_position()
		(orx,ory,orz) = self.rgripper.get_position()

		width = self.items[targetID].width
		breadth = self.items[targetID].breadth

		while math.fabs(self.lgripper.x - (item.x-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (item.x+(width/2)))>self.delta:
			if self.lgripper.x < (item.x-(width/2)):
				self.lgripper.x+=self.delta
			else:
				self.lgripper.x-=self.delta
			if self.rgripper.x < (item.x+(breadth/2)):
				self.rgripper.x+=self.delta
			else:
				self.rgripper.x-=self.delta
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.y - item.y)>self.delta or math.fabs(self.rgripper.y - item.y)>self.delta:
			if self.lgripper.y < item.y:
				self.lgripper.y+=self.delta
			else:
				self.lgripper.y-=self.delta
			self.rgripper.y = self.lgripper.y
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.z - (item.z+0.05))>self.delta:
			if self.lgripper.z < (item.z+0.05):
				self.lgripper.z+=self.delta
			else:
				self.lgripper.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		##########################################
		while math.fabs(self.lgripper.z - (olz+0.05))>self.delta:
			if self.lgripper.z < (olz+0.05):
				self.lgripper.z+=self.delta
				item.z+=self.delta
			else:
				self.lgripper.z-=self.delta
				item.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.x - (olx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (olx+(width/2)))>self.delta:
			if self.lgripper.x < (olx-(width/2)):
				self.lgripper.x+=self.delta
				item.x+=self.delta
			else:
				self.lgripper.x-=self.delta
				item.x-=self.delta
			if self.rgripper.x < (olx+(breadth/2)):
				self.rgripper.x+=self.delta
			else:
				self.rgripper.x-=self.delta
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.y - oly)>self.delta or math.fabs(self.rgripper.y - oly)>self.delta:
			if self.lgripper.y < oly:
				self.lgripper.y+=self.delta
				item.y+=self.delta
			else:
				self.lgripper.y-=self.delta
				item.y-=self.delta
			self.rgripper.y = self.lgripper.y
			time.sleep(1./self.fps)
			self.refresh_world()


	def put_in_box(self,targetID,bx,by,bz):
		item = self.items[targetID]
		(olx,oly,olz) = self.lgripper.get_position()
		(orx,ory,orz) = self.rgripper.get_position()

		width = self.items[targetID].width
		breadth = self.items[targetID].breadth

		while math.fabs(self.lgripper.x - (bx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (bx+(width/2)))>self.delta:
			if self.lgripper.x < (bx-(width/2)):
				self.lgripper.x+=self.delta
				item.x+=self.delta
			else:
				self.lgripper.x-=self.delta
				item.x-=self.delta
			if self.rgripper.x < (bx+(breadth/2)):
				self.rgripper.x+=self.delta
			else:
				self.rgripper.x-=self.delta
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.y - by)>self.delta or math.fabs(self.rgripper.y - by)>self.delta:
			if self.lgripper.y < by:
				self.lgripper.y+=self.delta
				item.y+=self.delta
			else:
				self.lgripper.y-=self.delta
				item.y-=self.delta
			self.rgripper.y = self.lgripper.y
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.z - (bz+0.05))>self.delta:
			if self.lgripper.z < (bz+0.05):
				self.lgripper.z+=self.delta
				item.z+=self.delta
			else:
				self.lgripper.z-=self.delta
				item.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		##########################################
		while math.fabs(self.lgripper.z - (olz+0.05))>self.delta:
			if self.lgripper.z < (olz+0.05):
				self.lgripper.z+=self.delta
				
			else:
				self.lgripper.z-=self.delta
				
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.x - (olx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (olx+(width/2)))>self.delta:
			if self.lgripper.x < (olx-(width/2)):
				self.lgripper.x+=self.delta
				
			else:
				self.lgripper.x-=self.delta
				
			if self.rgripper.x < (olx+(breadth/2)):
				self.rgripper.x+=self.delta
			else:
				self.rgripper.x-=self.delta
			time.sleep(1./self.fps)
			self.refresh_world()

		while math.fabs(self.lgripper.y - oly)>self.delta or math.fabs(self.rgripper.y - oly)>self.delta:
			if self.lgripper.y < oly:
				self.lgripper.y+=self.delta
				
			else:
				self.lgripper.y-=self.delta
				
			self.rgripper.y = self.lgripper.y
			time.sleep(1./self.fps)
			self.refresh_world()
	






for i in range(1):
	# p.stepSimulation()
	time.sleep(1./420)
	g = Grocery_packing()
	g.pick_up('cube')
	g.put_in_box('cube',-0.6,0,0.7)
	time.sleep(60)

	# p.resetBasePositionAndOrientation(boxId, [h,0,1], cubeStartOrientation)
	# h+=.01

# (x,y,z), cubeOrn = p.getBasePositionAndOrientation(boxId)
# print((x,y,z))
p.disconnect()