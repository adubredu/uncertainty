#! /usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import time
import pybullet_data
import math
import threading
import numpy as np
from fd import Fast_Downward

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf") 

class Box:
	def __init__(self, bottom_capacity):
		self.cpty = bottom_capacity
		self.full_cpty = 9
		self.index = 0
		self.old_index = 0
		self.lx = 260 
		self.ly = 290
		self.ys = [-0.1, 0, 0.1]
		self.xs = [-0.6, -0.5, -0.4]
		self.z = 0.7
		#i is per row, j is per column
		self.occupancy = [[0 for j in range(self.cpty)] for i in range(self.cpty)]
		self.items_added = {}
		self.to_resolve = False
		self.num_items = 0
		self.cascade = False

	def add_item(self, item):
		# self.items_added[item.name] = self.index%self.cpty
		self.num_items+=1
		xyind = (99,99)
		for j in range(self.cpty):
			for i in range(self.cpty):
				if self.occupancy[i][j] == 0:
					xyind = (i,j)
					break
		if xyind[0] != 99:
			x = self.xs[xyind[0]]
			y = self.ys[xyind[1]]
			self.occupancy[xyind[0]][xyind[1]] = 1
			self.items_added[item] = xyind
		else:
			print('Box full')
			return (99,99,99)
			# x = self.widths[self.index%self.cpty]
			# if self.cascade: 
			#     y = self.heights[self.index%self.cpty]- item.height                 
			# else:
			#     y = self.ly - item.height 

			# self.heights[self.index%self.cpty] = y
			# self.index +=1
		return x,y,self.z

	def remove_item(self, item):
		if item in self.items_added:
			index = self.items_added[item]
			self.occupancy[index[0]][index[1]] = 0
			self.items_added.pop(item)
			self.num_items-=1



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
		self.inbox = False
		self.inclutter = True

		self.item_on_top = None 


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
		self.table = Grocery_item(0,0,0, 0,0,0, "table/table.urdf",1,1,1,'table','heavy')
		self.lgripper = Grocery_item(0.2,0.0,1.5, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_left_finger.urdf",1,1,1,'lgripper','light')
		self.rgripper = Grocery_item(0.23,0.0,1.5, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_right_finger.urdf",1,1,1,'rgripper','light')
		p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')
		self.tray = Grocery_item(-.5,.0,0.62, 0,0,0, "container/container.urdf",0.3,0.3,0.3,'tray','heavy')
		self.items = {
						'table':self.table,
						'lgripper':self.lgripper,
						'rgripper':self.rgripper,
						'tray': self.tray
		}
		self.init_clutter()

		self.planning_time = 0
		self.domain_path='/home/developer/uncertainty/pddl/belief_domain.pddl'


		self.box = Box(3)
		self.delta = 0.01
		self.confidence_threshold = 0.7
		self.fps = 60
		self.scene_belief = {}
		self.clutter_xs = [-0.65, -0.55, -.45, -.35, -.25, -.15, -.05]
		self.clutter_ys = [-.4, -.3, -.2, .2, .3, .4]
		perception = threading.Thread(target=self.start_perception,args=(1,))
		perception.start()


	def refresh_world(self):
		for key in self.items:
			self.items[key].update_object_position()
		p.stepSimulation()


	def init_clutter(self):
		self.bottle = Grocery_item(.5,0.,0.65, 1.57,0,0, "bottle/bottle.urdf",0.07,0.07,0.35,'bottle','light')
		self.coke = Grocery_item(.5,.1,.65, 0,0,0, 'coke/coke.urdf', 0.07,0.07,0.1,'coke','light')
		self.nutella = Grocery_item(.6, 0., .65, 0,0,0, 'nutella/nutella.urdf', 0.07,0.07,0.07,'nutella','light')
		self.orange = Grocery_item(.6,.1,.65,0,0,0, 'orange/orange.urdf', 0.07,0.07,0.05, 'orange','light')
		self.cereal = Grocery_item(.5, .2, .65, 1.57,0,0, 'cereal/cereal.urdf',0.07,0.07,0.1, 'cereal','light')
		self.lysol = Grocery_item(.6, -.1, .65, 0,0,0, 'lysol/lysol.urdf', 0.07,0.07,0.2, 'lysol','light')
		self.lipton = Grocery_item(.6, .2, .65, 0,0,0, 'lipton/lipton.urdf',0.07,0.07,0.05, 'lipton','light' )
		self.apple = Grocery_item(.7, 0., .65, 3.14,0,0, 'apple/apple.urdf', 0.07,0.07,0.03, 'apple','light')

		self.items['bottle'] = self.bottle
		self.items['coke'] = self.coke
		self.items['nutella'] = self.nutella
		self.items['orange'] = self.orange
		# self.items['pepsi'] = self.pepsi 
		self.items['cereal'] = self.cereal
		self.items['lysol'] = self.lysol
		self.items['lipton'] = self.lipton
		self.items['apple'] = self.apple



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
					if cd[0] > 240:
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
					if nm == item and cf >= self.confidence_threshold:
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
		item.inbox = False
		item.inclutter = False 

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
		item.inbox = True
		item.inclutter = False

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


	def put_on(self, topitem, botitem):
		if topitem == botitem:
			return False

		item = self.items[topitem]
		bot = self.items[botitem]
		item.inbox = True
		bot.item_on_top = topitem

		(bx, by, bz) = bot.get_position()
		bz = bz + bot.height

		(olx,oly,olz) = self.lgripper.get_position()
		(orx,ory,orz) = self.rgripper.get_position()

		width = item.width
		breadth = item.breadth

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

		while math.fabs(self.lgripper.z - bz)>self.delta:
			if self.lgripper.z < bz:
				self.lgripper.z+=self.delta
				item.z+=self.delta
			else:
				self.lgripper.z-=self.delta
				item.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		##########################################
		while math.fabs(self.lgripper.z - olz)>self.delta:
			if self.lgripper.z < olz:
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


	def put_in_clutter(self, itemname):
		item = self.items[itemname]
		bx = self.clutter_xs[np.random.randint(7)]
		by = self.clutter_ys[np.random.randint(6)]
		bz = 0.65
		self.clutter_xs.remove(bx)

		item.inclutter = True
		item.inbox = False

		(olx,oly,olz) = self.lgripper.get_position()
		(orx,ory,orz) = self.rgripper.get_position()

		width = item.width
		breadth = item.breadth

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

		while math.fabs(self.lgripper.z - bz)>self.delta:
			if self.lgripper.z < bz:
				self.lgripper.z+=self.delta
				item.z+=self.delta
			else:
				self.lgripper.z-=self.delta
				item.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()

		##########################################
		while math.fabs(self.lgripper.z - olz)>self.delta:
			if self.lgripper.z < olz:
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


	def select_perceived_objects_and_classify_weights(self):
		confident_seen_list = []; inbox_list = []
		lightlist = []; heavylist=[]

		for item in self.scene_belief:
			if len(self.scene_belief[item]) > 0:
				if self.scene_belief[item][0][1] >= self.confidence_threshold:
					confident_seen_list.append(item)

		for item in self.items:
			if self.items[item].inbox:
				inbox_list.append(item)

		for item in inbox_list+confident_seen_list:
			if self.items[item].mass == 'heavy':
				heavylist.append(item)
			else:
				lightlist.append(item)

		return inbox_list, confident_seen_list, lightlist, heavylist


	def create_pddl_problem(self, inbox, topfree, mediumlist, heavylist):
		itlist = heavylist+mediumlist
		alias = {}
		hc = 0
		for item in heavylist:
			alias[item] = 'h'+str(hc)
			hc+=1

		mc = 0
		for item in mediumlist:
			alias[item] = 'm'+str(mc)
			mc+=1

		init = "(:init (handempty) "
		for item in inbox:
			init += "(inbox "+alias[item]+") "
			it = self.items[item].item_on_top
			if it != None:
				init+= "(on "+alias[it]+" "+alias[item]+") "
			else:
				init += "(topfree "+alias[item]+") "


		for item in topfree:
			init += "(topfree "+alias[item]+") "
			init += "(inclutter "+alias[item]+") "

		# if self.box.num_items >= 5:
		#     init += "(boxfull)"

		init +=  ")\n"    

		goal = "(:goal (and "
		for h in heavylist:
			goal += "(inbox "+alias[h]+") "
			
		mlen=len(mediumlist)
		hlen=len(heavylist)
		stop = self.box.full_cpty - hlen

		if hlen == self.box.full_cpty and mlen > hlen:

			for m in mediumlist[:hlen]:
				goal += "(or "
				for h in heavylist:
					goal += "(on "+alias[m]+" "+alias[h]+") "
				goal+=") "

			for m in mediumlist[hlen:]:
				goal += "(or "
				for mm in mediumlist[:hlen]:
					goal += "(on "+alias[m]+" "+alias[mm]+") "
				goal+=") "
			goal +=")))"

		else:
			for m in mediumlist[:stop]:
				goal += "(inbox "+alias[m]+") "
			for m in mediumlist[stop:stop+self.box.full_cpty]:
				goal+="(or "
				for mm in heavylist+mediumlist[:stop]:
					goal += "(on "+alias[m]+" "+alias[mm]+") "
				goal+=") "
			for m in mediumlist[stop+self.box.full_cpty:]:
				goal += "(or "
				for mm in mediumlist[stop:self.box.full_cpty]:
					goal += "(on "+alias[m]+" "+alias[mm]+") "
				goal+=") "
			goal +=")))"


		definition = "(define (problem PACKED-GROCERY) \n(:domain GROCERY) \n (:objects "
		for al in alias.values():
			definition += al+" "
		definition += "- item)\n"

		problem = definition + init + goal

		f = open("newprob.pddl","w")
		f.write(problem)
		f.close()
		dir_path = os.path.dirname(os.path.realpath(__file__))
		prob_path = dir_path+"/"+"newprob.pddl"
		
		swapped_alias  = dict([(value, key) for key, value in alias.items()]) 
		return prob_path, swapped_alias

	def execute_plan(self, plan, alias):
		if plan is None or len(plan) == 0:
			print('NO  VALID PLAN FOUND')
			return

		for action in plan:
			print('Performing action: '+str(action))
			self.belief_execute_action(action, alias)


	def plan_and_run_belief_space_planning(self, domain_path, problem_path, alias):
		f = Fast_Downward()
		start = time.time()
		plan = f.plan(domain_path, problem_path)
		self.planning_time += time.time()-start
		self.execute_plan(plan, alias)

	def belief_execute_action(self, action, alias):
		if action[0] == 'pick-from-clutter':
			self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'pick-from-box':
			self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'pick-from':
			self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'put-in-box':
			x,y,z = self.box.add_item(alias[action[1]])
			self.put_in_box(alias[action[1]],x,y,z)

		elif action[0] == 'put-in-clutter':
			self.drop_in_clutter(alias[action[1]])

		elif action[0] == 'put-on':
			self.put_on(alias[action[1]], alias[action[2]])


	def perform_optimistic_belief_grocery_packing(self):
		empty_clutter = True if len(self.scene_belief) == 0 else False

		while not empty_clutter:
			inboxlist, topfreelist, lightlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
			problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
												lightlist, heavylist)
			self.plan_and_run_belief_space_planning(self.domain_path, 
														problem_path, alias)
			empty_clutter = True if len(self.scene_belief) == 0 else False

	
	def perform_optimistic(self):
		start = time.time()
		self.perform_optimistic_belief_grocery_packing()
		end = time.time()
		total = end-start
		print('PLANNING TIME FOR OPTIMISTIC: '+str(self.planning_time))
		print('EXECUTION TIME FOR OPTIMISTIC: '+str(total - self.planning_time))


def test_pick_place():
	g = Grocery_packing()
	box = Box(3)
	time.sleep(10)

	# g.pick_up('lysol')
	# g.put_on('lysol', 'cereal')
	# g.pick_up('lysol')
	# g.put_in_clutter('lysol')
	g.pick_up('bottle')
	i,j,k = box.add_item('bottle')
	g.put_in_box('bottle',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")

	g.pick_up('nutella')
	i,j,k = box.add_item('nutella')
	g.put_in_box('nutella',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")


	g.pick_up('coke')
	i,j,k = box.add_item('coke')
	g.put_in_box('coke',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")


	g.pick_up('cereal')
	i,j,k = box.add_item('cereal')
	g.put_in_box('cereal',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")

	g.pick_up('lipton')
	i,j,k = box.add_item('lipton')
	g.put_in_box('lipton',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")

	g.pick_up('orange')
	i,j,k = box.add_item('orange')
	g.put_in_box('orange',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")

	g.pick_up('apple')
	i,j,k = box.add_item('apple')
	g.put_in_box('apple',i,j,k)
	print(g.scene_belief)
	print("\n**********************************\n")

	g.pick_up('lysol')
	i,j,k = box.add_item('lysol')
	g.put_in_box('lysol',i,j,k)	
	print(g.scene_belief)


if __name__ == '__main__':
	g = Grocery_packing()
	time.sleep(10)
	g.perform_optimistic()

# for i in range(1):
# 	# p.stepSimulation()
# 	time.sleep(1./420)
# 	test_pick_place()
# 	time.sleep(60)

	# p.resetBasePositionAndOrientation(boxId, [h,0,1], cubeStartOrientation)
	# h+=.01

# (x,y,z), cubeOrn = p.getBasePositionAndOrientation(boxId)
# print((x,y,z))
p.disconnect()































