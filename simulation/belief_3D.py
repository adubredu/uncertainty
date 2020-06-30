#! /usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import time
import pybullet_data
import math
import copy
import threading
import numpy as np
from fd import Fast_Downward
from planner import Planner 
import rospy
from std_msgs.msg import String, Bool
from grocery_items import Shopping_List, Grocery_item

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
p.setAdditionalSearchPath('models')
planeId = p.loadURDF("plane/plane.urdf") 
tableId = p.loadURDF("table/table.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]))

trayId = p.loadURDF("container/container.urdf", [-.5,.0,0.65],p.getQuaternionFromEuler([0,0,0]))





class Gripper:
	def __init__(self):
		self.holding = None 

class Box:
	def __init__(self, bottom_capacity, vast=False):
		self.cpty = bottom_capacity
		self.full_cpty = 9
		self.index = 0
		self.old_index = 0
		self.lx = 260 
		self.ly = 290
		self.ys = [-0.1, 0, 0.1]
		self.xs = [-0.6, -0.5, -0.4]
		if self.cpty == 2:
			self.ys = [-0.05,  0.05]
			self.xs = [-0.55,  -0.45]

		self.z = 0.8
		self.vast = vast


		#i is per row, j is per column
		self.occupancy = [[0 for j in range(self.cpty)] for i in range(self.cpty)]
		self.items_added = {}
		self.to_resolve = False
		self.num_items = 0
		self.cascade = False

	def add_item(self, item):
		# self.items_added[item.name] = self.index%self.cpty
		
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
			self.num_items+=1
		else:
			if not self.vast:
				print('Box full')
				print(self.items_added)
				print(self.num_items)
				return (99,99,99)
			else:
				return -.5,0,.8
		return x,y,0.8

	def remove_item(self, item):
		if item in self.items_added:
			index = self.items_added[item]
			self.occupancy[index[0]][index[1]] = 0
			self.items_added.pop(item)
			self.num_items-=1


class Grocery_packing:
	def __init__(self):
		self.start_time = time.time()
		self.time_pub = rospy.Publisher('/time', String, queue_size=1)

		self.box = Box(3)
		self.gripper = Gripper()
		self.item_list = ['baseball',
					  'beer',
					  'can_coke',
					  'can_pepsi',
					  'can_fanta',
					  'can_sprite',
					  'chips_can',
					  'coffee_box',
					  'cracker',
					  'cup',
					  'donut',
					  'fork',
					  'gelatin',
					  'meat',
					  'mustard',
					  'newspaper',
					  'orange',
					  'pear',
					  'plate',
					  'soccer_ball',
					  'soup',
					  'sponge',
					  'sugar',
					  'toy']

		self.clutter_ps = []
		self.xs = [0.65,  .45,  .25, .10]
		self.ys = [-.3, -.2, .3, .2]
		for x in self.xs:
			for y in self.ys:
				self.clutter_ps.append((x,y))

		self.shopping_list = Shopping_List(p)
		self.items = self.shopping_list.get_items_dict()
		self.objects_list = self.shopping_list.get_items_list()
		self.items_in_box = []

		self.plan_pub = rospy.Publisher('/plan', String, queue_size=1)
		self.boxitems_pub = rospy.Publisher('/box_items', String, queue_size=1)
		self.scene_belief_publisher = rospy.Publisher('/scene_belief', String, queue_size=1)
		self.action_pub = rospy.Publisher('/current_action', String, queue_size=1)
		self.method_pub = rospy.Publisher('/method', String, queue_size=1)
		self.should_plan = rospy.Publisher('/should_plan', Bool, queue_size=1)


		self.arrangement_difficulty = 'hard'
		self.space_allowed = 'low'
		self.arrangement_num = 2
		self.init_clutter(self.arrangement_num)
		# self.generate_clutter_coordinates(self.space_allowed)


		self.planning_time = 0
		self.num_mc_samples = 100
		self.num_pick_from_box = 0
		self.domain_path='/home/alphonsus/3dmodels/uncertainty/pddl/belief_domain.pddl'


		self.lgripper = self.items['lgripper']
		self.rgripper = self.items['rgripper']
		
		
		self.delta = 0.01
		self.confidence_threshold = 0.7
		self.fps = 60
		self.scene_belief = {}
		
		self.num_false = 0
		self.alive = True
		# self.f = Planner()
		self.perception = threading.Thread(target=self.start_perception,args=(1,))
		self.perception.start()
		# time.sleep(1000)



	def refresh_world(self):
		for key in self.items:
			if not self.items[key].dummy:
				self.items[key].update_object_position()
		duration = int(time.time()-self.start_time)
		d = String()
		d.data = str(int(duration))
		self.time_pub.publish(d)

		a=''
		for it in self.items_in_box:
			a+=it 
			a+='*'
		a=a[:-1]
		b = String()
		b.data = a 
		self.boxitems_pub.publish(b)
		p.stepSimulation()


	def generate_init_coordinates(self, space):
		mx = 0.4; my = 0.0; z = 0.65
		if space == "high":
			delta = 0.3
			self.box.full_cpty = 9
		elif space == "medium":
			delta = 0.2
			self.box.full_cpty = 6
			self.clutter_ps=[]
			for x in self.xs[:-1]:
				for y in self.ys[:-1]:
					self.clutter_ps.append((x,y))
		else:
			delta = 0.1
			self.box = Box(2)
			self.box.full_cpty = 4
			self.clutter_ps=[]
			for x in self.xs[:-2]:
				for y in self.ys[:-2]:
					self.clutter_ps.append((x,y))

		x = np.random.uniform(low=mx-delta, high=mx+delta)
		y = np.random.uniform(low=my-delta, high=my+delta)

		return (x,y,z)




	def generate_clutter_coordinates(self, space):
		mindist = 0.05
		generated = {}
		for item in self.item_list:
			(x,y,z) = self.generate_init_coordinates(space)
			generated[item] = [x,y,z]
		
		taken_care_of = []
		zs = []
		scene_structure =[]
		for item1 in generated:
			for item2 in generated:
				if item2 not in taken_care_of and item1!=item2 and item1 not in taken_care_of:
					x1 = generated[item1][0]; x2 = generated[item2][0];
					y1 = generated[item1][1]; y2 = generated[item2][1];
					dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
					if dist < mindist:
						zs.append((item2, generated[item2][2] + self.items[item1].height))
						scene_structure.append((item2, 'on', item1))
						taken_care_of.append(item2)

		points=0
		for top, _, bot in scene_structure:
			if self.items[top].mass == 'light' and self.items[bot].mass == 'heavy':
				points += 1
		arr = 'n'
		if len(scene_structure) > 0:
			score = points/len(scene_structure)
			if score < 0.5:
				arr = 'easy'
			else:
				arr = 'hard'

		if arr != self.arrangement_difficulty:
			self.generate_clutter_coordinates(space)
		else:
			for item, z in zs:
				generated[item][2] = z 

			f = open(self.arrangement_difficulty+'_'+self.space_allowed+'.txt', 'a')
			for item in generated:
				x = generated[item][0]; y=generated[item][1]; z=generated[item][2];
				f.write(item + ','+str(x) + ',' +str(y) + ','+str(z))
				f.write('\n')
			f.write('*\n')
			f.close()
			print('DONE GENERATING!')



	def init_clutter(self, index):
		init_positions = self.read_init_positions(index)
		for name, x, y, z in init_positions:
			self.items[name].x = float(x)
			self.items[name].y = float(y)
			self.items[name].z = float(z)

		# self.generate_clutter_coordinates(self.space_allowed)
		# self.objects_list = self.shopping_list.get_items_list()
		self.refresh_world()


	def read_init_positions(self, index):
		f = open(self.arrangement_difficulty+'_'+self.space_allowed+'.txt','r')
		content = f.read()
		stages = content.split('*')
		coords = stages[index-1].split('\n')
		xyz = []
		for coord in coords:
			abc = coord.split(',')
			xyz.append(abc)
		xyz = xyz[:-1]
		if index!=1:
			xyz = xyz[1:]

		# print(xyz)
		return xyz


	def start_perception(self,x):
		import pybullet as p
		import time
		import pybullet_data
		import math
		import sys
		sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
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

					if cd[0] > 300 and cd[1] > 60 and cd[1]<400:
						names.append(name)
						weights.append(wt)
						coord.append([int(c) for c in cd])

				summ = np.sum(weights)
				norm_wt = weights/summ
				for name, wt, cd in zip(names, norm_wt, coord):
					norm_scene[item].append((name,wt,cd))

			return norm_scene

		while self.alive:
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

			model = core.Model.load('/home/alphonsus/3dmodels/grocery_detector_v9_2.pth', \

				['baseball',
					  'beer',
					  'can_coke',
					  'can_pepsi',
					  'can_fanta',
					  'can_sprite',
					  'chips_can',
					  'coffee_box',
					  'cracker',
					  'cup',
					  'donut',
					  'fork',
					  'gelatin',
					  'meat',
					  'mustard',
					  'newspaper',
					  'orange',
					  'pear',
					  'plate',
					  'soccer_ball',
					  'soup',
					  'sponge',
					  'sugar',
					  'toy'])
			
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
			scene_data = String()
			scene_data.data = ''
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
						scene_data.data +=nm+'-'+str(round(cf,2))+'*'
			
			self.scene_belief_publisher.publish(scene_data)

			# print(self.scene_belief)

			cv2.imshow('Grocery Item detection', camera_view)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			p.stepSimulation()



	def pick_up(self,targetID):
		item = self.items[targetID]
		if item.dummy:
			return False
		for it in self.objects_list:
			if it.item_on_top == targetID:
				it.item_on_top = None
		if item.inbox:
			self.box.remove_item(targetID)
		try:
			self.items_in_box.remove(targetID)
		except:
			pass
		item.inbox = False
		item.inclutter = False 
		self.gripper.holding = targetID

		(olx,oly,olz) = (-0.13,-.5,1.5)
		(orx,ory,orz) = (-0.1, -.5, 1.5)

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
		# print('start with x')
		while math.fabs(self.lgripper.y - item.y)>self.delta or math.fabs(self.rgripper.y - item.y)>self.delta:
			if self.lgripper.y < item.y:
				self.lgripper.y+=self.delta
			else:
				self.lgripper.y-=self.delta
			self.rgripper.y = self.lgripper.y
			time.sleep(1./self.fps)
			self.refresh_world()
		# print('start with y')
		while math.fabs(self.lgripper.z - (item.z+0.05))>self.delta:
			if self.lgripper.z < (item.z+0.05):
				self.lgripper.z+=self.delta
			else:
				self.lgripper.z-=self.delta
			self.rgripper.z = self.lgripper.z
			time.sleep(1./self.fps)
			self.refresh_world()
		# print('start with z')
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
		# print('done with z')
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
		# print('done with x')
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
		# print('done with y')
		return True


	def put_in_box(self,targetID,bx,by,bz):
		item = self.items[targetID]
		if item.dummy or bx == 99:
			return False
		item.inbox = True
		item.inclutter = False
		self.gripper.holding = None
		self.items_in_box.append(targetID)
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

		return True


	def put_on(self, topitem, botitem):
		if topitem == botitem:
			return False
		item = self.items[topitem]
		bot = self.items[botitem]
		if item.dummy or bot.dummy:
			return False
		# item.inbox = True
		self.gripper.holding = None
		bot.item_on_top = topitem
		self.items_in_box.append(topitem)
		# if bot.inclutter:
		# 	item.inclutter = True
		# elif bot.inbox:
		# 	item.inbox = True

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
		return True


	def put_in_clutter(self, itemname):
		item = self.items[itemname]
		if item.dummy:
			return False
		r = np.random.randint(len(self.clutter_ps))
		bx,by = self.clutter_ps[r]
		bz = 0.7
		self.gripper.holding = None 

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
		return True


	def select_perceived_objects_and_classify_weights(self):
		confident_seen_list = []; inbox_list = []
		lightlist = []; heavylist=[]
		scene_belief = copy.deepcopy(self.scene_belief)

		for item in scene_belief:
			if len(scene_belief[item]) > 0 and item in self.items:
				if scene_belief[item][0][1] >= self.confidence_threshold:
					if len(scene_belief[item]) == 2 and self.num_false > 2:
						cf1 = scene_belief[item][0][1]
						cf2 = scene_belief[item][1][1]
						nm1 = scene_belief[item][0][0]
						nm2 = scene_belief[item][1][0]
						wts = [cf1, cf2]/np.sum([cf1, cf2])
						choice = np.random.choice([nm1, nm2], size=1,p=wts)
						if not self.items[choice[0]].inbox:
							confident_seen_list.append(choice[0])
						print('USED NUM FALSE')
					elif len(scene_belief[item]) > 2 and self.num_false > 2:
						cf1 = scene_belief[item][0][1]
						cf2 = scene_belief[item][1][1]
						cf3 = scene_belief[item][2][1]
						nm1 = scene_belief[item][0][0]
						nm2 = scene_belief[item][1][0]
						nm3 = scene_belief[item][2][0]
						wts = [cf1, cf2, cf3]/np.sum([cf1, cf2,cf3])
						choice = np.random.choice([nm1, nm2,nm3], size=1,p=wts)
						if not self.items[choice[0]].inbox:
							confident_seen_list.append(choice[0])
						print('USED NUM FALSE')
					else:
						# if self.items[item].inclutter:  #BIAS TO ONLY RECOGNIZE ITEMS IN CLUTTER
						if not self.items[item].inbox:
							confident_seen_list.append(item)
		print('confidently scene items: '+str(confident_seen_list))
		# for item in self.objects_list:
		# 	if item.inbox and not item.dummy:
				# inbox_list.append(item.name)
		for key in self.box.items_added:
			inbox_list.append(key)

		for item in inbox_list+confident_seen_list:
			if item in self.items and not self.items[item].dummy:
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

		if self.gripper.holding is not None:
			name = self.gripper.holding
			if self.items[name].mass == 'heavy':
				alias[name] = 'h'+str(hc)
				hc+=1
			else:
				alias[name] = 'm'+str(mc)
				mc+=1

		init = "(:init (handempty) "
		if self.gripper.holding is not None:
			init = "(:init (holding "+alias[self.gripper.holding]+") "

		for item in inbox:
			init += "(inbox "+alias[item]+") "
			it = self.items[item].item_on_top
			if it != None and it in alias:
				init+= "(on "+alias[it]+" "+alias[item]+") "
			else:
				init += "(topfree "+alias[item]+") "


		for item in topfree:
			if not self.items[item].dummy:
				init += "(topfree "+alias[item]+") "
				init += "(inclutter "+alias[item]+") "

		if self.box.num_items >= self.box.full_cpty:
			init += "(boxfull)"

		init +=  ")\n"    

		goal = "(:goal (and "
		for h in heavylist[:self.box.full_cpty]:
			goal += "(inbox "+alias[h]+") "

		hleft = heavylist[self.box.full_cpty:]
		hin = heavylist[:self.box.full_cpty]
		hputon = hin[:len(hleft)]

		for l, i in zip(hleft, hputon):
			goal += "(on "+alias[l]+" "+alias[i]+") "


		hvlist_free = hleft + hin[len(hleft):]
			
		mlen=len(mediumlist)
		hlen=len(hvlist_free)
		stop = self.box.full_cpty - hlen

		if hlen >= self.box.full_cpty and mlen > hlen:
			for m,h in zip(mediumlist[:hlen],hvlist_free):
				goal += "(on "+alias[m]+" "+alias[h]+") "

			lenontop = len(mediumlist[:hlen])
			for m, mm in zip(mediumlist[hlen:][:lenontop], mediumlist[:hlen]):
				goal += "(on "+alias[m]+" "+alias[mm]+") "

			# lenleft
		else:
			for m in mediumlist[:stop]:
				goal += "(inbox "+alias[m]+") "

			currfreeinbox = hvlist_free+mediumlist[:stop]
			for m, mh in zip(mediumlist[stop:(stop+self.box.full_cpty)], currfreeinbox):
				goal +="(on "+alias[m]+" "+alias[mh]+") "

			left = mediumlist[(stop+self.box.full_cpty):]
			newcurron = mediumlist[stop:(stop+self.box.full_cpty)]
			for m, mm in zip(left, newcurron[:len(left)]):
				goal +="(on "+alias[m]+" "+alias[mm]+") "

		goal+=")))\n"
		'''
		# USES DISJUNCTIONS. SEEMS LIKE A DIFFICULT
		#PLANNING PROBLEM FOR LARGE STATE SPACES: N>16
		if hlen == self.box.full_cpty and mlen > hlen:

			for m in mediumlist[:hlen]:
				goal += "(or "
				for h in hvlist_free:
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
				for mm in hvlist_free+mediumlist[:stop]:
					goal += "(on "+alias[m]+" "+alias[mm]+") "
				goal+=") "
			for m in mediumlist[stop+self.box.full_cpty:]:
				goal += "(or "
				for mm in mediumlist[stop:self.box.full_cpty]:
					goal += "(on "+alias[m]+" "+alias[mm]+") "
				goal+=") "
			goal +=")))"
			'''

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

	def read_plan(self):
		filename = 'fdplan'
		try:
			with open(filename, 'r') as f:
				plan = f.read()
		except:
			return None
		p = plan.split('\n')
		retplan = []
		for act in p:
			tup = act.replace(')','').replace('(','').replace("'","").replace(" ","").split(',')
			tup = tuple(tup)
			retplan.append(tup)
		return retplan



		#FDREPLAN ALGO

	def plan_and_run_belief_space_planning(self, domain_path, problem_path, alias, declutter=False):
		start = time.time()
		
		b = Bool(); b.data = True; self.should_plan.publish(b)
		# rospy.Subscriber('/planned', String, self.get_plan)
		# time.sleep(2)
			# print('planning...')
			# rospy.Subscriber('/planned', String, self.get_plan)

		# plan = f.plan(domain_path, problem_path)
		time.sleep(5)
		plan = self.read_plan()
		print(plan)
		self.planning_time += time.time()-start
		self.convert_to_string_and_publish(plan,alias)

		if plan is None or len(plan) <= 1:
			print('NO VALID PLAN FOUND')
			print(self.scene_belief)
			if declutter:
				self.declutter_surface_items()
			self.num_false +=1
			if self.confidence_threshold > 0.2:
				self.confidence_threshold -= 0.1
				print(self.confidence_threshold)

			return

		for action in plan:
			if action[1] not in alias:
				print('wrong aliasing')
				print(alias)
				return
			else:
				if len(action) == 3:
					if action[2] not in alias:
						print('wrong aliasing')
						print(alias)
						return
		self.convert_to_string_and_publish(plan, alias)
		for action in plan:
			a = String()
			f = list(action)
			f[1] = alias[action[1]]
			if len(action) == 3:
				f[2] = alias[action[2]]
			f = str(f)
			a.data = f 
			print(f)
			self.action_pub.publish(a)
			result = self.execute_sbp_action(action, alias)
			if not result:
				try:
					os.remove('fdplan')
				except:
					pass
				self.current_action = "Action: REPLANNING..."  
				print('REPLANNING')
				self.current_plan = 'Blah'
				self.num_false+=1
				a.data = 'REPLANNING'		
				self.action_pub.publish(a)
				print('Box num is: '+str(self.box.num_items))
				inboxlist, topfreelist, mediumlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
				new_problem_path, nalias = self.create_pddl_problem(inboxlist, topfreelist,
												mediumlist, heavylist)
				self.plan_and_run_belief_space_planning(self.domain_path, new_problem_path, nalias)
				break
		try:
			os.remove('fdplan')
		except:
			pass
		a = String()
		a.data = ''
		
		self.action_pub.publish(a)
		self.plan_pub.publish(a)
		return


	

	def is_clutter_empty(self):
		for item in self.objects_list:
			if not item.dummy and item.inclutter:
				return False
		return True


	def perform_optimistic_belief_grocery_packing(self,declutter=False):
		empty_clutter = self.is_clutter_empty()

		while not empty_clutter:
			if declutter:
				self.declutter_surface_items()
			inboxlist, topfreelist, lightlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
			problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
												lightlist, heavylist)

			self.plan_and_run_belief_space_planning(self.domain_path, 
														problem_path, alias,declutter=declutter)
			empty_clutter = self.is_clutter_empty()

	
	def perform_optimistic(self):
		start = time.time()
		self.perform_optimistic_belief_grocery_packing()
		end = time.time()
		total = end-start
		exe = total - self.planning_time
		print('PLANNING TIME FOR OPTIMISTIC: '+str(self.planning_time))
		print('EXECUTION TIME FOR OPTIMISTIC: '+str(total - self.planning_time))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))

		self.save_results('Optimistic',self.planning_time,exe)


	def perform_declutter(self):
		obs = []; too_close=[]
		for item in self.item_list:
			if self.items[item].inclutter:
				obs.append(item)
		for item1 in obs:
			for item2 in obs:
				if item1 != item2:
					it1 = self.items[item1]
					it2 = self.items[item2]
					dist = np.sqrt((it1.x-it2.x)**2+(it1.y-it2.y)**2)
					if dist < 0.05 and self.items[item2].inclutter:
						too_close.append(item2)
		too_close = list(set(too_close))
		print(too_close)

		for item in too_close:
			self.pick_up(item)
			self.put_in_clutter(item)
		print('done decluttering')


	def perform_declutter_belief_grocery_packing(self):
		time.sleep(5)
		start = time.time()
		# self.should_declutter = True
		# self.declutter_surface_items()
		# self.should_declutter = False
		self.perform_optimistic_belief_grocery_packing(declutter=True)
		end = time.time()
		total = end - start
		exe = total - self.planning_time
		print('PLANNING TIME FOR DECLUTTER: '+str(self.planning_time))
		print('EXECUTION TIME FOR DECLUTTER: '+str(total - self.planning_time))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))
		self.save_results('Declutter',self.planning_time,exe)

	def create_sbp_problem(self, inbox, topfree, mediumlist, heavylist):
		num_hypotheses = 5
		topfree =[]
		mediumlist =[]
		heavylist=[]

		for item in inbox:
			if self.items[item].mass == 'heavy':
				heavylist.append(item)
			else:
				mediumlist.append(item)

		alias = {}
		hc = 0
		for item in heavylist:
			alias[item] = 'h'+str(hc)
			hc+=1

		mc = 0
		for item in mediumlist:
			alias[item] = 'm'+str(mc)
			mc+=1

		if self.gripper.holding is not None:
			name = self.gripper.holding
			if self.items[name].mass == 'heavy':
				alias[name] = 'h'+str(hc)
				hc+=1
			else:
				alias[name] = 'm'+str(mc)
				mc+=1


		init = "(:init "
		for item in inbox:
			init += "(inbox "+alias[item]+") "
			it = self.items[item].item_on_top
			if it != None and it in alias:
				init+= "(on "+alias[it]+" "+alias[item]+") "
			else:
				init += "(topfree "+alias[item]+") "

		if self.box.num_items >= self.box.full_cpty:
			init += "(boxfull)"
		# for item in topfree:
		# 	init += "(topfree "+alias[item]+") "
		# 	init += "(inclutter "+alias[item]+") "

		#generating scene hypotheses and choosing the hypothesis with
		#highest weight
		scene_hypotheses = []
		for i in range(num_hypotheses):
			subhyp=[]
			for item in self.scene_belief:
				if len(self.scene_belief[item]) > 0:
					obs = [obconf[0] for obconf in self.scene_belief[item]]
					wts = [obconf[1] for obconf in self.scene_belief[item]]
					wts = wts/np.sum(wts)
					choice = np.random.choice(obs, size=1, p=wts)
					name = choice[0]
					ind = obs.index(name)
					subhyp.append((name, wts[ind]))
			scene_hypotheses.append(subhyp)

		#scoring hypothesis
		scores = [0 for i in range(num_hypotheses)]
		for i in range(num_hypotheses):
			for obwt in scene_hypotheses[i]:
				scores[i]+=obwt[1]
		maxind = np.argmax(scores)
		selected_hypothesis = scene_hypotheses[maxind]

		#add them to problem. TO CHANGE IF CONDITION LATER
		for item,_ in selected_hypothesis:
			if not item in alias:
				if self.items[item].mass == 'heavy':
					heavylist.append(item)
					alias[item] = 'h'+str(hc)
					hc+=1
					init += "(topfree "+alias[item]+") "
					init += "(inclutter "+alias[item]+") "

				else:
					mediumlist.append(item)
					alias[item] = 'm'+str(mc)
					mc +=1
					init += "(topfree "+alias[item]+") "
					init += "(inclutter "+alias[item]+") "

		if self.gripper.holding is not None:
			print('IS HOLDING '+self.gripper.holding)
			init+="(holding "+alias[self.gripper.holding]+") "
		else:
			init+='(handempty) '
		init +=  ")\n"

		goal = "(:goal (and "

		for h in heavylist[:self.box.full_cpty]:
			goal += "(inbox "+alias[h]+") "

		hleft = heavylist[self.box.full_cpty:]
		hin = heavylist[:self.box.full_cpty]
		hputon = hin[:len(hleft)]

		for l, i in zip(hleft, hputon):
			goal += "(on "+alias[l]+" "+alias[i]+") "

		hvlist_free = hleft + hin[len(hleft):]
			
		mlen=len(mediumlist)
		hlen=len(hvlist_free)
		stop = self.box.full_cpty - hlen

		if hlen == self.box.full_cpty and mlen > hlen:

			for m in mediumlist[:hlen]:
				goal += "(or "
				for h in hvlist_free:
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
				for mm in hvlist_free+mediumlist[:stop]:
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


	def convert_to_string_and_publish(self,plan,alias):
		concat = ''
		for action in plan:
			if action[0]!='Fail':
				action = list(action)
				action[1] = alias[action[1]]
				if len(action) == 3:
					action[2] = alias[action[2]]
				concat+=str(action)
				concat+='_'
		p = String()
		p.data = concat
		
		self.plan_pub.publish(p)

	def run_sbp(self, domain_path, problem_path, alias):
		# f = Planner()#PFast_Downward()
		start = time.time()

		b = Bool(); b.data = True; self.should_plan.publish(b)
		time.sleep(5)
		plan = self.read_plan()
		print(plan)
		self.planning_time += time.time()-start

		if plan is None or len(plan) <= 1:
			print('NO VALID PLAN FOUND')
			print(self.scene_belief)
			self.num_false +=1
			if self.confidence_threshold > 0.2:
				self.confidence_threshold -= 0.1
				print(self.confidence_threshold)

			return

		for action in plan:
			if action[1] not in alias:
				print('wrong aliasing')
				return
			else:
				if len(action) == 3:
					if action[2] not in alias:
						print('wrong aliasing')
						return

		self.convert_to_string_and_publish(plan,alias)
		
		for action in plan:
			# print(action)
			a = String()
			f = list(action)
			f[1] = alias[action[1]]
			if len(action) == 3:
				f[2] = alias[action[2]]
			f = str(f)
			print(f)
			a.data = f		
			self.action_pub.publish(a)
			result = self.execute_sbp_action(action, alias)
			if not result:
				self.current_action = "Action: REPLANNING..."  
				a.data = 'REPLANNING'		
				self.action_pub.publish(a)
				print('REPLANNING')
				inboxlist, topfreelist, mediumlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
				new_problem_path, nalias = self.create_sbp_problem(inboxlist, topfreelist,
												mediumlist, heavylist)
				self.run_sbp(self.domain_path, new_problem_path, nalias)
				break
		a = String()
		a.data = ''
		try:
			os.remove('fdplan')
		except:
			pass
		
		self.action_pub.publish(a)
		self.plan_pub.publish(a)
		return



	def execute_sbp_action(self,action, alias):
		self.current_action = "Action: "+str(action)
		success = True
		if action[0] == 'pick-from-clutter':
			if not self.items[alias[action[1]]].inclutter:
				print('Wrong pick')
				return False
			success = self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'pick-from-box':
			if not self.items[alias[action[1]]].inbox:
				print('Wrong pick')
				return False
			self.num_pick_from_box+=1
			success = self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'pick-from':
			self.num_pick_from_box+=1
			success = self.pick_up(alias[action[1]])
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'put-in-box':
			x,y,z = self.box.add_item(alias[action[1]])
			success = self.put_in_box(alias[action[1]],x,y,z)

		elif action[0] == 'put-in-clutter':
			success = self.put_in_clutter(alias[action[1]])

		elif action[0] == 'put-on':
			success = self.put_on(alias[action[1]], alias[action[2]])

		return success

		
	def perform_sbp_grocery_packing(self):
		st = time.time()

		empty_clutter = self.is_clutter_empty()

		while not empty_clutter:
			inboxlist, topfreelist, mediumlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
			problem_path, alias = self.create_sbp_problem(inboxlist, topfreelist,
												mediumlist, heavylist)
			self.run_sbp(self.domain_path, problem_path, alias)

			empty_clutter = self.is_clutter_empty()
		end = time.time()
		total = end-st
		exe = total-self.planning_time
		print('PLANNING TIME FOR SBP: '+str(self.planning_time))
		print('EXECUTION TIME FOR SBP: '+str(total-self.planning_time))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))

		self.save_results('sbp',self.planning_time,exe)


	def single_sample(self, occluded_items):
		sampled_items=[]
		for bunch in occluded_items:
			# print(bunch)
			items = [b[0] for b in bunch]
			weights = [b[1] for b in bunch]
			weights = weights/np.sum(weights)
			sample = np.random.choice(items, size=1, p=weights)
			sampled_items.append(sample[0])

		return sampled_items


	def monte_carlo_sample(self, occluded_items):
		mc_counts={}
		items = [x.name for x in self.objects_list]
		for t in items: mc_counts[t] = 0
		mc_samples=[]
		for i in range(self.num_mc_samples):
			sampled_items = self.single_sample(occluded_items)
			joined=''
			for it in set(sampled_items):
				joined+= it+'*'
			mc_samples.append(joined[:-1])

		final_sample = max(set(mc_samples), key=mc_samples.count)
		sample = final_sample.split('*')
		return sample


	def divergent_set_sample_1(self, occluded_items):
		num_samples = 10
		divergent_samples = []
		sample_scores = [0 for i in range(num_samples)]

		for i in range(num_samples):
			sample = self.single_sample(occluded_items)
			divergent_samples.append(sample)

		#generate mc sample
		mc_sample = self.monte_carlo_sample(occluded_items)

		#score each sample in set
		for i in range(num_samples):
			for obj in divergent_samples[i]:
				if obj in mc_sample:
					sample_scores[i] += 1

		#get max sample and min sample and choose one with 0.5 probability
		arg_max_samp = np.argmax(sample_scores)
		arg_min_samp = np.argmin(sample_scores)

		max_sample = divergent_samples[arg_max_samp]
		min_sample = divergent_samples[arg_min_samp]

		toss = np.random.randint(2)

		if toss == 1:
			return max_sample
		else:
			return min_sample


	def divergent_set_sample_2(self, occluded_items):
		pass


	def estimate_clutter_content(self,surface_items,inboxlist,sample_procedure):
		
		# occluded are items in self.scene_belief whose confidence < threshold
		occluded_items = []
		for item in self.scene_belief:
			if len(self.scene_belief[item]) > 0:
				if self.scene_belief[item][0][1] < self.confidence_threshold:
					occluded_items.append(self.scene_belief[item])

		if len(occluded_items) == 0:
			r = np.random.randint(2)
			if r == 0:
				return 1,0
			else:
				return 0,1
		#one-time weighted sample. To change sampling strategy, alter 
		#the function: self.high_uncertainty_sample(name)
		if sample_procedure == 'weighted_sample':
			# SAMPLE WITH JUST THE ORIGINAL WEIGHTS
			sampled_occluded_items = self.single_sample(occluded_items)

		elif sample_procedure == 'mc_sample':
			# SAMPLE MULTIPLE TIMES FROM ORIGINAL WEIGHT AND CHOOSE ONE WITH
			# HIGHEST FREQ
			sampled_occluded_items = self.monte_carlo_sample(occluded_items)

		elif sample_procedure == 'divergent_set_1':
			# 1. DRAW MULTIPLE SAMPLES, SCORE THEM BY SIMILARITY TO MC SAMPLE
			# CHOOSE FROM THE BEST AND WORST
			sampled_occluded_items = self.divergent_set_sample_1(occluded_items)

		elif sample_procedure == 'divergent_set_2':
			# 2. DRAW MULTIPLE SAMPLES, COMPUTE PLANS FOR EACH, FIND THE
			# TWO WITH THE HIGHEST DIFF AND CHOOSE ONE WITH PROBABILITY 0.5
			sampled_occluded_items = self.divergent_set_sample_2(occluded_items)
		
		num_heavy = 0; num_light = 0;
		for it in sampled_occluded_items:
			if self.items[it].mass == 'heavy':
				num_heavy +=1
			else:
				num_light += 1
		#finding percentage of heavy and light
		# print("NUM HEAVY: "+str(num_heavy)+" over "+str(N_o_s))
		oh = (float(num_heavy)/float(len(sampled_occluded_items)))

		suh = 0
		print('surface items:')
		print(surface_items)
		for it in surface_items:
			if self.items[it].mass == 'heavy':
				suh +=1

		print("sNUM HEAVY: "+str(suh)+" over "+str(len(surface_items)))
		if len(surface_items) == 0:
			sh = 1
		else:
			sh = float(suh)/float(len(surface_items))
		return oh, sh


	def declutter_surface_items(self):
		self.perform_declutter()
		'''
		occluded_items = []
		num = int(len(self.item_list)/4)

		for item in self.item_list:
			if self.items[item].inclutter:
				occluded_items.append(item)
		# print(self.scene_belief)
		# for item in self.scene_belief:
		# 	if not self.items[item].dummy:
		# 		if len(self.scene_belief[item]) > 0:
		# 			if self.scene_belief[item][0][1] < self.confidence_threshold+0.3:
		# 				occluded_items.append(self.scene_belief[item][0][0])
		for item in occluded_items[:num]:
			self.pick_up(item)
			self.put_in_clutter(item)
		'''
		

	def perform_dynamic_grocery_packing(self,sample_procedure):
		st = time.time()
		empty_clutter = self.is_clutter_empty()

		while not empty_clutter:
			inboxlist, topfreelist, mediumlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
			problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
												mediumlist, heavylist)
			
			unoccluded_items = topfreelist
			oh, sh = self.estimate_clutter_content(unoccluded_items,inboxlist,sample_procedure)
			print("probs are "+str(oh)+" "+str(sh))

			if sh >= oh:
				print('\nPERFORMING OPT\n')
				self.plan_and_run_belief_space_planning(self.domain_path, \
										problem_path, alias)
			else:
				print(('\nPERFORMING DECLUTTER\n'))
				self.declutter_surface_items()
			
			empty_clutter = self.is_clutter_empty()
		end = time.time()
		total = end-st
		exe = total-self.planning_time
		print('PLANNING TIME FOR DYNAMIC: '+str(self.planning_time))
		print('EXECUTION TIME FOR DYNAMIC: '+str(total-self.planning_time))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))

		self.save_results('Dynamic_'+sample_procedure,self.planning_time,exe)


	def get_objects_in_order(self):
		obs=[]
		for item in self.objects_list:
			if not item.dummy:
				obs.append(item.name)
		return obs

	def perform_conveyor_belt_pack(self):
		# items_in_order = self.get_objects_in_order()
		start = time.time()
		empty_clutter = self.is_clutter_empty()

		while not empty_clutter:
			print('Beginning while')
			_,items_in_order,_,_ = self.select_perceived_objects_and_classify_weights()
			print('ITEMS IN ORDER: '+str(len(items_in_order)))
			for item in items_in_order:
				inboxlist, topfreelist, mediumlist, heavylist = \
						self.select_perceived_objects_and_classify_weights()
				for t in topfreelist:
					if t != item:
						try:
							mediumlist.remove(t)
						except:
							pass
				for t in topfreelist:
					if t != item:
						try:
							heavylist.remove(t)
						except:
							pass
				topfreelist = [item]

				if self.items[item].mass == 'heavy':
					if item not in heavylist:
						heavylist.append(item)
				else:
					if item not in mediumlist:
						mediumlist.append(item)

				print('creating prob')
				problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
													mediumlist, heavylist)
				print('planning and running')
				self.plan_and_run_belief_space_planning(self.domain_path, 
														problem_path, alias)
			print('RUN IS OVER')
			empty_clutter = self.is_clutter_empty()

		end = time.time()
		total = end-start
		exe = total - self.planning_time
		print('PLANNING TIME FOR CONVEYORBELT: '+str(self.planning_time))
		print('EXECUTION TIME FOR CONVEYORBELT: '+str(total - self.planning_time))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))

		self.save_results('Conveyor_Belt',self.planning_time,exe)


	def perform_pick_n_roll(self):
		st = time.time()
		items_in_order = self.get_objects_in_order()
		light = []
		box = Box(3,vast=True)
		box.cascade = True 

		for item in items_in_order:
			if self.items[item].mass == 'heavy':
				x,y,z= box.add_item(item)
				self.pick_up(item)
				self.put_in_box(item,x,y,z)

			else:
				light.append(item)
				self.pick_up(item)
				self.put_in_clutter(item)

		for item in light:
			x,y,z = box.add_item(item)
			self.pick_up(item)
			self.put_in_box(item,x,y,z)

		duration = time.time() - st
		print("PLANNING TIME FOR PICKNROLL: 0")
		print("EXECUTION TIME FOR PICKNROLL: "+str(duration))
		self.save_results('Pick_n_roll',0,duration)



	def perform_bag_sort(self):
		st = time.time()
		items_in_order = self.get_objects_in_order()
		light = []; heavy=[]
		box = Box(3,vast=True)
		box.cascade = True 

		for item in items_in_order:
			if self.items[item].mass == 'heavy':
				heavy.append(item)
				self.pick_up(item)
				self.put_in_clutter(item)				
			else:
				light.append(item)
				self.pick_up(item)
				self.put_in_clutter(item)

		for item in heavy:
			x,y,z= box.add_item(item)
			self.pick_up(item)
			self.put_in_box(item,x,y,z)

		for item in light:
			x,y,z = box.add_item(item)
			self.pick_up(item)
			self.put_in_box(item,x,y,z)

		duration = time.time() - st
		print("PLANNING TIME FOR BAGSORT: 0")
		print("EXECUTION TIME FOR BAGSORT: "+str(duration))
		print('NUMBER OF BOX REMOVES: '+str(self.num_pick_from_box))

		self.save_results('Bag_sort',0,duration)


	def save_results(self, algo, planning_time, execution_time):
		# return
		f = open("results_"+self.arrangement_difficulty+'_'+self.space_allowed+'.txt',"a")		
		f.write(algo+'_'+str(self.arrangement_num))
		f.write('\n')
		f.write('planning_time: '+str(planning_time)+'\n')
		f.write('execution_time: '+str(execution_time) +'\n')
		f.write('num_pick_from_box: '+str(self.num_pick_from_box)+'\n\n')
		f.close()

	def run_strategy(self, strategy):
		if strategy == 'conveyor-belt':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_conveyor_belt_pack()
		elif strategy == 'pick-n-roll':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_pick_n_roll()
		elif strategy == 'bag-sort':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_bag_sort()
		elif strategy == 'sbp':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_sbp_grocery_packing()
		elif strategy == 'optimistic':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_optimistic()
		elif strategy == 'declutter':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_declutter_belief_grocery_packing()
		elif strategy == 'mc-dynamic':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_dynamic_grocery_packing('mc_sample')
		elif strategy == 'weighted-dynamic':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_dynamic_grocery_packing('weighted_sample')
		elif strategy == 'divergent-dynamic':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_dynamic_grocery_packing('divergent_set_1')
		self.alive = False
		a = Bool()
		a.data = False



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
	args = sys.argv
	if len(args) != 2:
		print("Arguments should be strategy and order")
	else:        
		rospy.init_node('grocery_packing')
		strategy = args[1]
		# order = int(args[2])
		g = Grocery_packing()

		time.sleep(30)
		g.run_strategy(strategy)

	# g.perform_pick_n_roll()
	# g.perform_conveyor_belt_pack()
	# g.perform_dynamic_grocery_packing('divergent_set_1')
	# g.perform_sbp_grocery_packing()
	# g.perform_declutter_belief_grocery_packing()
	# g.perform_optimistic()

# for i in range(1):
# 	# p.stepSimulation()
# 	time.sleep(1./420)
# 	test_pick_place()
# 	time.sleep(60)

	# p.resetBasePositionAndOrientation(boxId, [h,0,1], cubeStartOrientation)
	# h+=.01

# (x,y,z), cubeOrn = p.getBasePositionAndOrientation(boxId)
# print((x,y,z))
# p.disconnect()































