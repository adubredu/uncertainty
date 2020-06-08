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
		self.cube = Grocery_item(3.,3.,0.5, 0,0,0, "cube_small.urdf",0.07,0.07,0.07,'cube','light')
		self.table = Grocery_item(0,0,0, 0,0,0, "table/table.urdf",1,1,1,'table','heavy')
		self.lgripper = Grocery_item(-0.05,0.0,2.0, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_left_finger.urdf",1,1,1,'lgripper','light')
		self.rgripper = Grocery_item(0.05,0.0,2.0, 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_right_finger.urdf",1,1,1,'rgripper','light')
		self.tray = Grocery_item(-.5,.0,0.6, 0,0,0, "container/container.urdf",1,1,1,'tray','heavy')

		self.items = {
						'cube':self.cube,
						'table':self.table,
						'lgripper':self.lgripper,
						'rgripper':self.rgripper,
						'tray': self.tray
		}
		self.delta = 0.01
		self.fps = 60


	def refresh_world(self):
		for key in self.items:
			self.items[key].update_object_position()
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
	






for i in range(1):
	# p.stepSimulation()
	time.sleep(1./420)
	g = Grocery_packing()
	g.pick_up('cube')

	# p.resetBasePositionAndOrientation(boxId, [h,0,1], cubeStartOrientation)
	# h+=.01

(x,y,z), cubeOrn = p.getBasePositionAndOrientation(boxId)
print((x,y,z))
p.disconnect()