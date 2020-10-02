#! /usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import time
import signal
import pybullet_data
import math
import random
import copy
import threading
import numpy as np
from fd import Fast_Downward
from planner import Planner 
import rospy
from std_msgs.msg import String, Bool
from grocery_items import Shopping_List, Grocery_item
import pomcp, pomcp_er
from scipy.stats import entropy as sp_entropy
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from detecto import core, utils, visualize

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
p.setAdditionalSearchPath('models')
planeId = p.loadURDF("plane/plane.urdf") 
tableId = p.loadURDF("table/table.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]))

trayId = p.loadURDF("container/container.urdf", [-.5,.0,0.65],p.getQuaternionFromEuler([0,0,0]))
# tid = p.loadTexture('container/boxtexture.png')
# p.changeVisualShape(trayId, -1, textureUniqueId=tid)





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
	def __init__(self, diff, arr, run_num, strat):
		self.start_time = time.time()
		self.time_pub = rospy.Publisher('/time', String, queue_size=1)

		self.gripper = Gripper()

		np.random.seed(int(arr))

		self.clutter_ps = []
		self.xs = [0.4,  .35,  .25, .15, .1]
		self.ys = [-.3, -.2, -.1, 0, .1, .25, .2]
		for x in self.xs:
			for y in self.ys:
				self.clutter_ps.append((x,y))

		self.shopping_list = Shopping_List(p)
		self.items = self.shopping_list.get_items_dict()
		self.objects_list = self.shopping_list.get_items_list()
		self.item_list = self.shopping_list.get_item_string_list()
		self.items_in_box = []
		self.deccount = 0
		self.num_actions = 0
		self.num_mistakes = 0


		self.credentials = [str(diff), str(arr), str(run_num), str(strat)]

		self.plan_pub = rospy.Publisher('/plan', String, queue_size=1)
		self.boxitems_pub = rospy.Publisher('/box_items', String, queue_size=1)
		self.scene_belief_publisher = rospy.Publisher('/scene_belief', String, queue_size=1)
		self.action_pub = rospy.Publisher('/current_action', String, queue_size=1)
		self.method_pub = rospy.Publisher('/method', String, queue_size=1)
		self.should_plan = rospy.Publisher('/should_plan', Bool, queue_size=1)
		self.holding_pub = rospy.Publisher('/holding', String, queue_size=1)
		self.startpub = rospy.Publisher('/starting', Bool, queue_size=1)
		print(self.items.keys())
		self.pl_time = 0
		self.loaded=[]
		rospy.Subscriber('/planning_time', String, self.planning_time_callback)
		
		if diff == 'las':
			self.arrangement_difficulty = 'hard'
		else: 
			self.arrangement_difficulty = 'easy'
		self.space_allowed = 'high'
		self.arrangement_num = int(arr)

		if self.space_allowed == 'high':
			self.box = Box(2)
			self.box.full_cpty = 4
		else:
			self.box = Box(2) 
			self.box.full_cpty = 4
		self.init_clutter(self.arrangement_num)
		# self.generate_clutter_coordinates(self.space_allowed)
		# '''
		self.observation = None

		self.planning_time = 0.
		self.total_execution_time = 0.
		self.added_time = 0.
		self.num_mc_samples = 100
		self.num_pick_from_box = 0
		self.raw_belief_space = None
		self.domain_path='/home/bill/uncertainty/pddl/belief_domain.pddl'

		self.lgripper = self.items['lgripper']
		self.rgripper = self.items['rgripper']

		self.model = core.Model.load('/home/bill/backyard/grocery_detector_v9_2.pth', \

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
		
		self.delta = 0.01
		self.confidence_threshold = 0.7
		self.fps = 60
		self.scene_belief = {}

		self.detection_to_real ={}
		
		self.num_false = 0
		self.alive = True
		
		 

		self.perception = threading.Thread(target=self.start_perception,args=(1,))
		self.perception.start()
		# '''

		# self.pick_up('donut')
		# time.sleep(30)
		# self.validate('donut')

	def planning_time_callback(self, data):
		self.pl_time = data.data
		# fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
		# 		self.credentials[1]+'_'+self.credentials[2]+'_ptime.txt'
		# f  = open(fname, 'w')
		# f.write(self.pl_time)
		# f.close()




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


	def compute_entropy(self,N=10):
		'''
		1. Draw N samples
		2. Sum confidence of each n in N
		3. Normalize all N confidence sums
			making them into a probability distro.
		4. Compute entropy of probability distro

		'''
		num_objects = 20.0

		scene_belief = copy.deepcopy(self.scene_belief)

		beliefs = []
		for item in scene_belief:
			hypotheses = []; iih=[]; wih=[]
			for hypothesis in scene_belief[item]:
				s = (hypothesis[0], hypothesis[1])
				hypotheses.append(s)
				iih.append(hypothesis[0])
				wih.append(hypothesis[1])
			p = (1 - np.sum(wih))/(num_objects - len(iih))
			for it in self.item_list:
				if it not in iih:
					hypotheses.append((it, p))
			beliefs.append(hypotheses)

		# print(beliefs)
		# print('num of hypotheses is: ',len(beliefs))

		total_entropy = 0.0
		for bel in beliefs:
			wt = [b[1] for b in bel]
			wt /=np.sum(wt)
			h = sp_entropy(wt, base=2)
			total_entropy += h

		n_left = num_objects-len(beliefs)
		for i in range(int(n_left)):
			wt = [1./num_objects for _ in range(int(num_objects))]
			h = sp_entropy(wt, base=2)
			total_entropy += h

		#H_max
		H_max = 0.0
		for i in range(int(num_objects)):
			wt = [1./num_objects for i in range(int(num_objects))]
			H_max += sp_entropy(wt,base=2)


		norm_entropy = total_entropy/H_max
		print(norm_entropy)
		return norm_entropy
		# sample_confidences = []
		# for i in range(N):
		# 	s,w = self.sample_entropy(beliefs)
		# 	print('should be twenty four: ',len(s))
		# 	sample_confidences.append(np.sum(w))

		# print(scene_belief)
		# sample_confidences /= np.sum(sample_confidences)
		# entropy = -np.sum([p*np.log2(p) for p in sample_confidences])
		# print(sample_confidences)
		# print("Entropy is : ",entropy)
		# print("Scipy entropy is : ",sp_entropy(sample_confidences,base=2))

		# return entropy


	def sample_entropy(self,beliefs):
		s,w = self.single_sample(beliefs)
		u_s =[]; u_w = []
		for item in self.item_list:
			if item not in s:
				u_s.append(item)
				u_w.append(0)
		s = s+u_s 
		w = w+u_w
		

		return s,w




	def generate_init_coordinates(self, space):
		mx = 0.4; my = 0.0; z = 0.65
		if space == "high":
			delta = 0.2
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

		x = np.random.uniform(low=mx-(delta+0.3), high=mx+delta)
		y = np.random.uniform(low=my-(delta+0.2), high=my+delta)

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

			f = open('le_'+self.arrangement_difficulty+'_'+self.space_allowed+'.txt', 'a')
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
			self.loaded.append(name)

		# self.generate_clutter_coordinates(self.space_allowed)
		# self.objects_list = self.shopping_list.get_items_list()
		self.refresh_world()


	def read_init_positions(self, index):
		f = open('le_'+self.arrangement_difficulty+'_'+self.space_allowed+'.txt','r')
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
		
		import numpy as np 
		
		from PIL import Image
		def normalize_scene_weights(scene, segmented):
			norm_scene = {}

			for item in scene:
				names=[]; weights=[];coord=[]; ids=[]
				norm_scene[item]=[]
				for name,wt,cd in scene[item]:

					if cd[0] > 250 and cd[1] > 20 and cd[1]<400:
						names.append(name)
						weights.append(wt)
						coord.append([int(c) for c in cd])
						gx = int((cd[0]+cd[2])/2); gy = int((cd[1]+cd[3])/2)
						idd = segmented[gy-3][gx-3]
						ids.append(idd)
					# else:
					# 	norm_scene.pop(item, None)


				summ = np.sum(weights)
				norm_wt = weights/summ
				if len(names) > 0:
					for name, wt, cd, idd in zip(names, norm_wt, coord, ids):
						norm_scene[item].append((name,wt,cd, idd))
				else:
					norm_scene.pop(item, None)

			return norm_scene

		def add_gaussian_noise(image):
			row, col, ch = image.shape 
			mean = 0; var = 0.1; sigma = var**0.5;
			gauss = np.random.normal(mean,sigma, (row,col,ch))
			gauss = gauss.reshape(row,col,ch).astype('uint8')
			noisy = cv2.add(image, gauss)
			return noisy

		def add_some_entropy(labels):
			r = np.random.randint(2)
			if r==1 and len(labels)>1:
				print(' labels 0: '+labels[0]+' to '+labels[1])
				tmp = labels[0]
				labels[0] = labels[1]
				labels[1] = tmp
				return labels
			else:
				return labels



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

			width, height, colorImage, depthImg, segImg = p.getCameraImage(
				width=640, 
				height=640,
				viewMatrix=viewMatrix,
				projectionMatrix=projectionMatrix,
				shadow=True,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
			
			self.segmented = segImg
			noisyimage = add_gaussian_noise(colorImage)
			rgbImg = Image.fromarray(noisyimage).convert('RGB')
			self.observation = copy.deepcopy(rgbImg)
			predictions = self.model.predict(rgbImg)
			camera_view = cv2.cvtColor(np.array(rgbImg), cv2.COLOR_RGB2BGR)
			labels, boxes, scores = predictions
			# print(labels, scores)
			# labels = add_some_entropy(labels)
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
					if dist < 5:
						clusters[key].append(box)
						fit = True
						break
				if not fit:
					clusters[str(mid[0])+'_'+str(mid[1])] = [box]

			scene = {}
			for key in clusters:
				weights = [box['confidence'] for box in clusters[key]]
				r = np.random.randint(2)
				if r == 1 and len(weights)>1:
					secmaxind = weights.index(np.partition(weights,-2)[-2])
					maxweightedbox = clusters[key][secmaxind]
					scene[maxweightedbox['name']] = [[box['name'], box['confidence'], \
								box['coordinates'].tolist()] for box in clusters[key]]
					tmp = scene[maxweightedbox['name']][0][0]
					scene[maxweightedbox['name']][0][0] = scene[maxweightedbox['name']][1][0]
					scene[maxweightedbox['name']][1][0] = tmp
					# print('sorry ',tmp, len(weights))

				else:
					maxind = np.argmax(weights)
					maxweightedbox = clusters[key][maxind]
					r = np.random.randint(2)
					if r == 1:
						its = self.shopping_list.get_item_string_list()
						name = np.random.choice(its)
						scene[name] = [[box['name'], box['confidence'], \
									box['coordinates'].tolist()] for box in clusters[key]]
						# scene[name][0][0] = name
						# print(name, 'for', maxweightedbox['name'])
					else:
						scene[maxweightedbox['name']] = [(box['name'], box['confidence'], \
									box['coordinates'].tolist()) for box in clusters[key]]

			self.raw_belief_space = scene
			self.scene_belief = normalize_scene_weights(scene, self.segmented)


			# print(self.scene_belief)
			# print('***'*50)
			# print(self.raw_belief_space)
			# print('&&&'*50)
			# print('\n\n')

			scene_data = String()
			scene_data.data = ''
			for item in self.raw_belief_space:
				for nm, cf, cd in self.raw_belief_space[item]:
					if nm == item and cd[0] > 250 and cd[1] > 20 and cd[1]<400: # and cf >= self.confidence_threshold:
						color = (np.random.randint(255),\
								np.random.randint(255),\
								np.random.randint(255))
						camera_view = cv2.rectangle(camera_view, (int(cd[0]),
						 int(cd[1])), (int(cd[2]), int(cd[3])),color , 1)
						cv2.putText(camera_view, nm+':'+str(round(cf,2)), (int(cd[0]),int(cd[1])-10),\
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,2)
						scene_data.data +=nm+'-'+str(round(cf,2))+'*'
			
			self.scene_belief_publisher.publish(scene_data)

			

			cv2.imshow('Grocery Item detection', camera_view)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			p.stepSimulation()



	def pick_up(self,targetID):
		try:
			item = self.items[targetID]
		except:
			print('dummied: ',targetID)
			return False
		if item.dummy:
			return False
		# if item not in self.loaded:
		# 	return False
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

		(olx,oly,olz) = (-0.13,-.5,1.0)
		(orx,ory,orz) = (-0.1, -.5, 1.0)

		width = self.items[targetID].width

		while math.fabs(self.lgripper.x - (item.x-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (item.x+(width/2)))>self.delta:
			if self.lgripper.x < (item.x-(width/2)):
				self.lgripper.x+=self.delta
			else:
				self.lgripper.x-=self.delta
			if self.rgripper.x < (item.x+(width/2)):
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
			if self.rgripper.x < (olx+(width/2)):
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
		hold = String()
		hold.data = item.name 
		self.holding_pub.publish(hold)
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

		while math.fabs(self.lgripper.x - (bx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (bx+(width/2)))>self.delta:
			if self.lgripper.x < (bx-(width/2)):
				self.lgripper.x+=self.delta
				item.x+=self.delta
			else:
				self.lgripper.x-=self.delta
				item.x-=self.delta
			if self.rgripper.x < (bx+(width/2)):
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
				
			if self.rgripper.x < (olx+(width/2)):
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
		if bot.inbox:
			item.inbox = True
			self.items_in_box.append(topitem)
		else:
			item.inclutter=True
			item.inbox=False
		# if bot.inclutter:
		# 	item.inclutter = True
		# elif bot.inbox:
		# 	item.inbox = True

		(bx, by, bz) = bot.get_position()
		bz = bz + bot.height

		(olx,oly,olz) = self.lgripper.get_position()
		(orx,ory,orz) = self.rgripper.get_position()

		width = item.width

		while math.fabs(self.lgripper.x - (bx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (bx+(width/2)))>self.delta:
			if self.lgripper.x < (bx-(width/2)):
				self.lgripper.x+=self.delta
				item.x+=self.delta
			else:
				self.lgripper.x-=self.delta
				item.x-=self.delta
			if self.rgripper.x < (bx+(width/2)):
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
				
			if self.rgripper.x < (olx+(width/2)):
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
		try:
			item = self.items[itemname]
		except:
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

		while math.fabs(self.lgripper.x - (bx-(width/2)))>self.delta \
		 or math.fabs(self.rgripper.x - (bx+(width/2)))>self.delta:
			if self.lgripper.x < (bx-(width/2)):
				self.lgripper.x+=self.delta
				item.x+=self.delta
			else:
				self.lgripper.x-=self.delta
				item.x-=self.delta
			if self.rgripper.x < (bx+(width/2)):
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
				if item.z < 0.7:
					item.z = 0.7
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
				
			if self.rgripper.x < (olx+(width/2)):
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
		# scene_belief = copy.deepcopy(self.scene_belief)
		confident_seen_list = []
		self.detection_to_real = {}
		for item in self.scene_belief:
			if len(self.scene_belief[item])>0:
				confident_seen_list.append(item)
				idd = self.scene_belief[item][0][3]
				self.detection_to_real[item] = idd


		random.shuffle(confident_seen_list)
		print('confidently seen items: '+str(confident_seen_list))
		
		for key in self.box.items_added:
			inbox_list.append(key)

		for it in confident_seen_list:
			if it in inbox_list:
				inbox_list.remove(it)

		for item in inbox_list+confident_seen_list:
			# if item in self.items and not self.items[item].dummy:
			try:
				if self.items[item].mass == 'heavy':
					heavylist.append(item)
				else:
					lightlist.append(item)
			except:
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
			# if not self.items[item].dummy:
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



		#FD-REPLAN ALGO

	def run_fdreplan(self, domain_path, problem_path, alias, declutter=False):
		start = time.time()
		
		b = Bool(); b.data = True; self.should_plan.publish(b)

		time.sleep(5)
		self.added_time += 5.0
		plan = self.read_plan()
		print(plan)
		self.planning_time += time.time()-start
		

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
		# self.convert_to_string_and_publish(plan,alias)
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
			t = time.time()
			if alias[action[1]] not in self.item_list:
				print('not in item list', alias[action[1]])
				result = False 
			else:
				result = self.execute_action(action, alias)
			self.total_execution_time += time.time() - t
			print('Total Execution Time: ', self.total_execution_time)
			print('Num retracts: ', self.num_pick_from_box)
			fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
				self.credentials[1]+'_'+self.credentials[2]+'_etime.txt'
			f  = open(fname, 'w')
			f.write('exe: '+str(self.total_execution_time)+'\n')
			f.write('plan: '+str(self.pl_time)+'\n')
			f.write('num pick: '+str(self.num_pick_from_box)+'\n')
			f.write('num actions: '+str(self.num_actions)+'\n')
			f.write('num packed items: '+str(len(self.items_in_box))+'\n')
			f.write('num mistakes: '+str(self.num_mistakes)+'\n')
			f.close()
			if not result:
				try:
					os.remove('fdplan')
				except:
					pass
				if self.is_clutter_empty():
					return
				
				self.num_mistakes+=1
				self.current_action = "Action: REPLANNING..."  
				print('REPLANNING')
				self.current_plan = 'Blah'
				self.num_false+=1
				a.data = 'REPLANNING'		
				self.action_pub.publish(a)
				print('Box num is: '+str(self.box.num_items))
				inboxlist, topfreelist, mediumlist, heavylist = \
					self.sample_belief_space()
				new_problem_path, nalias = self.create_pddl_problem(inboxlist, topfreelist,
												mediumlist, heavylist)
				self.run_fdreplan(self.domain_path, new_problem_path, nalias)
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


	def run_classical_replanning(self, domain_path, problem_path, alias, declutter=False):
		start = time.time()
		
		b = Bool(); b.data = True; self.should_plan.publish(b)

		time.sleep(5)
		self.added_time += 5.0
		plan = self.read_plan()
		print(plan)
		self.planning_time += time.time()-start
		

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
		# self.convert_to_string_and_publish(plan,alias)
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
			t = time.time()
			if alias[action[1]] not in self.item_list:
				print('not in item list', alias[action[1]])
				result = False 
			else:
				result = self.execute_action(action, alias)
			self.total_execution_time += time.time() - t
			print('Total Execution Time: ', self.total_execution_time)
			print('Num retracts: ', self.num_pick_from_box)
			fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
				self.credentials[1]+'_'+self.credentials[2]+'_etime.txt'
			f  = open(fname, 'w')
			f.write('exe: '+str(self.total_execution_time)+'\n')
			f.write('plan: '+str(self.pl_time)+'\n')
			f.write('num pick: '+str(self.num_pick_from_box)+'\n')
			f.write('num actions: '+str(self.num_actions)+'\n')
			f.write('num packed items: '+str(len(self.items_in_box))+'\n')
			f.write('num mistakes: '+str(self.num_mistakes)+'\n')
			f.close()
			if not result:
				try:
					os.remove('fdplan')
				except:
					pass
				if self.is_clutter_empty():
					return
				
				self.num_mistakes +=1 
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
				self.run_classical_replanning(self.domain_path, new_problem_path, nalias)
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

	
	def perform_classical_replanning(self):
		start = time.time()
		empty_clutter = self.is_clutter_empty()

		while not empty_clutter:
			inboxlist, topfreelist, lightlist, heavylist = \
					self.select_perceived_objects_and_classify_weights()
			problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
												lightlist, heavylist)

			self.run_classical_replanning(self.domain_path, problem_path, alias)
			empty_clutter = self.is_clutter_empty()

		end = time.time()
		total = end-start-self.added_time
		print('sub PLANNING TIME FOR OPTIMISTIC: ',total - self.total_execution_time)
		print('EXECUTION TIME FOR OPTIMISTIC: ', self.total_execution_time)
		print('NUMBER OF BOX REMOVES: ',self.num_pick_from_box)
		fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
				self.credentials[1]+'_'+self.credentials[2]+'_STATUS.txt'
		f  = open(fname, 'w')
		f.write('SUCCESS')
		f.close()

		self.save_results('Optimistic',self.planning_time,self.total_execution_time)


	
	def convert_to_string_and_publish(self,plan,alias):
		# pass
		concat = ''
		for action in plan:
			if action[0]!='Fail':
				action = list(action)
				action[1] = alias[action[1]]
				if len(action) == 3:
					action[2] = alias[action[2]]
				concat+=str(action)
				concat+='*'
		p = String()
		p.data = concat
		
		self.plan_pub.publish(p)

	
	def execute_action(self,action, alias):
		self.current_action = "Action: "+str(action)
		success = True
		if action[0] == 'pick-from-clutter':
			proposed_name = alias[action[1]]
			real_name = self.get_real_name_of_detection(proposed_name)
			if real_name not in list(self.items.keys()):
				return False
			success = self.pick_up(real_name)
			self.num_actions+=1
			time.sleep(.5)
			success = success and self.validate(proposed_name)
			if not success:
				self.put_in_clutter(real_name)
				self.num_actions+=1
				self.num_mistakes+=1

		elif action[0] == 'pick-from-box':
			if not self.items[alias[action[1]]].inbox:
				print('Wrong pick from box')
				return False
			self.num_pick_from_box+=1
			success = self.pick_up(alias[action[1]])
			self.num_actions+=1
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'pick-from':
			self.num_pick_from_box+=1
			success = self.pick_up(alias[action[1]])
			self.num_actions+=1
			self.box.remove_item(alias[action[1]])

		elif action[0] == 'put-in-box':
			x,y,z = self.box.add_item(alias[action[1]])
			success = self.put_in_box(alias[action[1]],x,y,z)
			self.num_actions+=1

		elif action[0] == 'put-in-clutter':
			success = self.put_in_clutter(alias[action[1]])
			self.num_actions+=1

		elif action[0] == 'put-on':
			success = self.put_on(alias[action[1]], alias[action[2]])
			self.num_actions+=1

		return success

	

	def single_sample(self, occluded_items):
		sampled_items=[]
		item_weights = []
		ids = []
		self.detection_to_real = {}
		for bunch in occluded_items:
			# print(bunch)
			items = [b[0] for b in bunch]
			weights = [b[1] for b in bunch]
			ids = [b[2] for b in bunch]
			# item_weights.append(weights)
			weights = [np.abs(w) for w in weights]
			norm_weights = weights/np.sum(weights)
			sample = np.random.choice(items, size=1, p=norm_weights)
			ind = items.index(sample[0])
			idd = ids[ind]
			item_weights.append(weights[ind])
			sampled_items.append(sample[0])
			self.detection_to_real[sample[0]] = idd

		return sampled_items, item_weights


	def monte_carlo_sample(self, occluded_items):
		mc_counts={}
		items = [x.name for x in self.objects_list]
		for t in items: mc_counts[t] = 0
		mc_samples=[]
		for i in range(self.num_mc_samples):
			sampled_items,_ = self.single_sample(occluded_items)
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
			sample,_ = self.single_sample(occluded_items)
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


	def get_whole_scene_belief(self):
		belief = []
		for item in self.scene_belief:
			if len(self.scene_belief[item]) > 0:
				belief.append(self.scene_belief[item])
		return belief


	def get_real_name_of_detection(self, maybefalse):
		try:
			idd =  self.detection_to_real[maybefalse]
			name = maybefalse
			for it in self.items:
				if self.items[it].id == idd:
					name = it 
			return name
		except:
			print("Couldn't save its ID")
			return maybefalse


	def validate(self, proposed):
		m = String()
		m.data = "Validating..."
		self.action_pub.publish(m)
		holding = None
		gx=0; gy=0
		for item in self.raw_belief_space:
			for nm, cf, cd in self.raw_belief_space[item]:
				if nm == item:
					x = (int(cd[0]) + int(cd[2]))/2
					y = (int(cd[1]) + int(cd[3]))/2

					if np.abs(x-218) < 50 and np.abs(y-104) < 50:
						holding = nm 
						gx=x; gy=y;
						break

		rand = np.random.randint(2)
		if rand == 1:
			try: 
				idd = self.detection_to_real[proposed]
			except:
				return False
			name = None
			for it in self.items:
				if self.items[it].id == idd:
					name = it 
					break

			if holding is not None and name is not None:
				if holding == proposed and name == proposed:
					print('Validated holding')
					m.data = "Validation returned True"
					self.action_pub.publish(m)
					return True 
				else:
					m.data = "Validation returned False"
					self.action_pub.publish(m)
					print('Not valid. Holding ',holding)
					return False
			else:
				try:
					idd = self.detection_to_real[proposed]
				except:
					return False
				name = None
				for it in self.items:
					if self.items[it].id == idd:
						name = it 
						break
				if name == proposed:
					m.data = "Validation returned True"
					self.action_pub.publish(m)
					print('Validated id')
					return True 
				else:
					if idd == 1:
						print('Validated id=1')
						m.data = "Validation returned True"
						self.action_pub.publish(m)
						return True
					print('Not valid id',idd)
					m.data = "Validation returned False"
					self.action_pub.publish(m)
					return False
		else:
			try:
				idd = self.detection_to_real[proposed]
			except:
				return False
			name = None
			for it in self.items:
				if self.items[it].id == idd:
					name = it 
					break
			if name == proposed:
				m.data = "Validation returned True"
				self.action_pub.publish(m)
				print('Validated id')
				return True 
			else:
				if idd == 1:
					print('Validated id=1')
					m.data = "Validation returned True"
					self.action_pub.publish(m)
					return True
				print('Not valid id',idd)
				m.data = "Validation returned False"
				self.action_pub.publish(m)
				return False

	def perform_pomcp_er(self):
		start = time.time()
		empty_clutter = self.is_clutter_empty()
		state_space = {'holding':self.gripper.holding,
		'items':self.items}

		while not empty_clutter:
			belief = self.get_whole_scene_belief()
			state_space = {'holding':self.gripper.holding,
							'items':self.items}
			state = pomcp_er.State(state_space, belief)
			root_node = pomcp_er.Node(state)
			st = time.time()
			result_root = pomcp_er.perform_pomcp(root_node, num_iterations=10)
			if result_root is None:
				print('Root is None')
				continue
			self.planning_time += time.time()-st
			select_node = pomcp_er.select_action(result_root,infer=True)
			if select_node is None:
				print('No action found')
				continue
			action = select_node.birth_action
			print(action)
			if action[1] !='':
				t = time.time()
				self.detection_to_real = state.detection_to_real
				if action[1] not in self.item_list:
					print('not in item list', action[1])
					result = False 
				else:
					success = self.execute_pomcp_action(action)
				self.total_execution_time += time.time() - t
				self.num_actions+=1
				print('Total execution time: ', self.total_execution_time)
				print('Num retracts: ', self.num_pick_from_box)
				fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
					self.credentials[1]+'_'+self.credentials[2]+'_etime.txt'
				f  = open(fname, 'w')
				f.write('exe: '+str(self.total_execution_time)+'\n')
				f.write('plan: '+str(self.planning_time)+'\n')
				f.write('num pick: '+str(self.num_pick_from_box)+'\n')
				f.write('num actions: '+str(self.num_actions)+'\n')
				f.write('num packed items: '+str(len(self.items_in_box))+'\n')
				f.write('num mistakes: '+str(self.num_mistakes)+'\n')
				f.close()


			empty_clutter = self.is_clutter_empty()

		end = time.time()
		total = end-start
		print('PLANNING TIME FOR POMCP: ',(total-self.total_execution_time))
		print('EXECUTION TIME FOR POMCP: ',self.total_execution_time)
		print('NUMBER OF BOX REMOVES: ',self.num_pick_from_box)

		self.save_results('pomcp_er',self.planning_time,exe)
		

	def perform_pomcp(self):
		start = time.time()
		empty_clutter = self.is_clutter_empty()
		state_space = {'holding':self.gripper.holding,
		'items':self.items}

		while not empty_clutter:
			belief = self.get_whole_scene_belief()
			state_space = {'holding':self.gripper.holding,
							'items':self.items}
			state = pomcp.State(state_space, belief)
			root_node = pomcp.Node(state)
			st = time.time()
			result_root = pomcp.perform_pomcp(root_node, num_iterations=10)
			if result_root is None:
				print('Root is None')
				continue
			self.planning_time += time.time()-st
			select_node = pomcp.select_action(result_root,infer=True)
			if select_node is None:
				print('No action found')
				continue
			action = select_node.birth_action
			print(action)
			if action[1] !='':
				self.detection_to_real = state.detection_to_real
				t = time.time()
				success = self.execute_pomcp_action(action)
				self.total_execution_time += time.time() - t
				self.num_actions+=1
				print('Total execution time: ', self.total_execution_time)
				print('Num retracts: ', self.num_pick_from_box)
				fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
					self.credentials[1]+'_'+self.credentials[2]+'_etime.txt'
				f  = open(fname, 'w')
				f.write('exe: '+str(self.total_execution_time)+'\n')
				f.write('plan: '+str(self.planning_time)+'\n')
				f.write('num pick: '+str(self.num_pick_from_box)+'\n')
				f.write('num actions: '+str(self.num_actions)+'\n')
				f.write('num packed items: '+str(len(self.items_in_box))+'\n')
				f.write('num mistakes: '+str(self.num_mistakes)+'\n')
				f.close()


			empty_clutter = self.is_clutter_empty()

		end = time.time()
		total = end-start
		print('PLANNING TIME FOR POMCP: ',(total-self.total_execution_time))
		print('EXECUTION TIME FOR POMCP: ',self.total_execution_time)
		print('NUMBER OF BOX REMOVES: ',self.num_pick_from_box)

		self.save_results('pomcp',self.planning_time,exe)


	def perform_classical_planner(self):
		start = time.time()
		items_seen = list(self.raw_belief_space.keys())
		items_seen = [it for it in items_seen if not self.items[it].dummy]
		mediumlist=[]; heavylist=[]
		for item in items_seen:
			if self.items[item].mass == 'heavy':
				heavylist.append(item)
			else:
				mediumlist.append(item)
		added_time = 0
		problem_path, alias = self.create_pddl_problem([], items_seen, mediumlist, heavylist)

		
		
		b = Bool(); b.data = True; self.should_plan.publish(b)
		
		time.sleep(5)
		added_time+=5
		plan = self.read_plan()
		print(plan)
		self.planning_time += time.time()-start
		
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

		for action in plan:
			t = time.time()
			result = self.execute_action(action, alias)
			self.total_execution_time += time.time() - t 
			print('TOTAL EXECUTION TIME: ', self.total_execution_time)
			print('Num retracts: ', self.num_pick_from_box)
			if not result:
				print('Failed to plan')
				try:
					os.remove('fdplan')
				except:
					pass
				return
		total = time.time() - start - added_time
		print('Number of packed: ', str(len(items_seen)))
		print('Total time taken: ', total)
		print('Execution time: ', self.total_execution_time)
				
		return


	def sample_belief_space(self):
		confident_seen_list = []; inbox_list = []
		lightlist = []; heavylist=[]
		# scene_belief = copy.deepcopy(self.raw_belief_space)

		# occluded_items = []
		# for item in scene_belief:
		# 	occluded_items.append(scene_belief[item])

		num_objects = 20.0

		scene_belief = copy.deepcopy(self.scene_belief)

		occluded_items = []
		for item in scene_belief:
			hypotheses = []; iih=[]; wih=[]; ids = 1
			for hypothesis in scene_belief[item]:
				s = (hypothesis[0], hypothesis[1], hypothesis[3])
				hypotheses.append(s)
				iih.append(hypothesis[0])
				wih.append(hypothesis[1])
				ids = hypothesis[3]
			p = (1 - np.sum(wih))/(num_objects - len(iih))
			for it in self.item_list:
				if it not in iih:
					hypotheses.append((it, p,ids))
			occluded_items.append(hypotheses)
		# print(occluded_items)
		confident_seen_list,_ = self.single_sample(occluded_items)
		random.shuffle(confident_seen_list)
		print('confident sampled scene items: '+str(confident_seen_list))
		
		for key in self.box.items_added:
			inbox_list.append(key)
######################################################
		for item in self.items:
			if item in inbox_list:
				it = self.items[item].item_on_top
				if it != None:
					inbox_list.append(it)
#####################################################
		for it in confident_seen_list:
			if it in inbox_list:
				inbox_list.remove(it)

		for item in inbox_list+confident_seen_list:
			try:
				if self.items[item].mass == 'heavy':
					heavylist.append(item)
				else:
					lightlist.append(item)
			except:
				lightlist.append(item)

		return inbox_list, confident_seen_list, lightlist, heavylist


	def perform_fdreplan(self,declutter=False):
		empty_clutter = self.is_clutter_empty()
		start = time.time()
		while not empty_clutter:
			if declutter:
				self.declutter_surface_items()
			inboxlist, topfreelist, lightlist, heavylist = \
					self.sample_belief_space()

			firstfree = topfreelist


			problem_path, alias = self.create_pddl_problem(inboxlist, firstfree,
												lightlist, heavylist)

			self.run_fdreplan(self.domain_path, problem_path, alias,declutter)
			empty_clutter = self.is_clutter_empty()
		end = time.time()
		total = end-start-self.added_time
		print('sub PLANNING TIME FOR FDREPLAN: ', total - self.total_execution_time)
		print('EXECUTION TIME FOR FDREPLAN: ', self.total_execution_time)
		print('TOTAL TIME FOR FDREPLAN: ', total)
		print('NUMBER OF BOX REMOVES: ',self.num_pick_from_box)
		fname = 'exp_data/'+self.credentials[3]+'/'+self.credentials[0]+'_'+ \
				self.credentials[1]+'_'+self.credentials[2]+'_STATUS.txt'
		f  = open(fname, 'w')
		f.write('SUCCESS')
		f.close()

		self.save_results('fdreplan',self.planning_time,self.total_execution_time)




	def execute_pomcp_action(self,action):
		self.current_action = "Action: "+str(action)
		success = True
		if action[0] == 'pick-from-clutter':
			# if not self.items[action[1]].inclutter:
			# 	print('Wrong pick')
			# 	return False
			proposed_name = action[1]
			real_name = self.get_real_name_of_detection(proposed_name)
			success = self.pick_up(real_name)
			time.sleep(.5)
			success = success and self.validate(proposed_name)
			if not success:
				self.put_in_clutter(real_name)
				self.num_actions+=1
				self.num_mistakes+=1

		elif action[0] == 'pick-from-box':
			if not self.items[action[1]].inbox:
				print('Wrong pick')
				return False
			self.num_pick_from_box+=1
			success = self.pick_up(action[1])
			self.box.remove_item(action[1])

		elif action[0] == 'pick-from':
			self.num_pick_from_box+=1
			success = self.pick_up(action[1])
			self.box.remove_item(action[1])

		elif action[0] == 'put-in-box':
			x,y,z = self.box.add_item(action[1])
			success = self.put_in_box(action[1],x,y,z)
			if not success:
				self.put_in_clutter(action[1])
				self.num_actions+=1
				self.num_mistakes+=1

		elif action[0] == 'put-in-clutter':
			success = self.put_in_clutter(action[1])

		elif action[0] == 'put-on':
			success = self.put_on(action[1], action[2])

		return success


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
		b = Bool()
		b.data = True
		self.startpub.publish(b)
		
		if strategy == 'classical-replanner':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_classical_replanning()
		
		elif strategy == 'pomcp':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_pomcp()
		elif strategy == 'pomcp_er':
			m = String()
			m.data = strategy
			self.method_pub.publish(m)
			self.perform_pomcp_er()
		elif strategy == 'classical-planner':
			m = String()
			m.data = strategy 
			self.method_pub.publish(m)
			self.perform_classical_planner()
		elif strategy == 'fdreplan':
			m = String()
			m.data = strategy 
			self.method_pub.publish(m)
			self.perform_fdreplan()

		self.alive = False
		a = Bool()
		a.data = False


def end_the_prog(signum, frame):
	sys.exit()

if __name__ == '__main__':
	args = sys.argv
	if len(args) != 5:
		print("Arguments should be strategy and order")
	else:        
		rospy.init_node('grocery_packing')
		strategy = args[1]
		difficulty = args[2]
		arrangement_num = args[3]
		run_number = args[4]
		# test_pick_place()
		# order = int(args[2])
		g = Grocery_packing(diff=difficulty, arr=arrangement_num,\
						 run_num=run_number, strat=strategy)

		time.sleep(30)
		signal.signal(signal.SIGTERM, end_the_prog)
		signal.alarm(1800)
		# g.compute_entropy()
		g.run_strategy(strategy)

		# time.sleep(60)

	































