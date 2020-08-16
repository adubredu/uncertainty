#! /usr/bin/env python3
#! /usr/bin/env python3

class Grocery_item:
	def __init__(self, urdf_path, object_name, 
				p, x=0, y=0, z=0, orr=0, op=0, oy=0,  width=0.01, 
				breadth=0.01, height=0.01, mass='light',
				dummy=False, urdf=False, texture=None):
		self.x = x
		self.y = y
		self.z = z
		self.width = width 
		self.breadth = breadth 
		self.height = height 
		self.mass = mass 
		self.inbox = False
		self.inclutter = True
		self.dummy = dummy
		self.item_on_top = None 
		self.p = p
		self.orr = orr 
		self.op = op 
		self.oy = oy 
		self.quat = self.p.getQuaternionFromEuler([self.orr,self.op,self.oy])
		self.name = object_name
		if not dummy:
			if object_name=='lgripper' or object_name=='rgripper':
				self.id = self.p.loadURDF(urdf_path)
				self.update_object_position()
			else:
				if urdf:
					self.id = self.p.loadSDF(urdf_path)[0]
					self.update_object_position()
					if texture is not None:
						tid = self.p.loadTexture(texture)
						self.p.changeVisualShape(self.id, -1, textureUniqueId=tid)



				else:
					self.load_item(urdf_path, [self.x,self.y,self.z], self.quat)
		

	def update_object_position(self):
		self.p.resetBasePositionAndOrientation(self.id, \
			[self.x,self.y,self.z], self.p.getQuaternionFromEuler([self.orr,self.op,self.oy]))


	def get_position(self):
		(x,y,z), _ = self.p.getBasePositionAndOrientation(self.id)
		self.x = x; self.y = y; self.z = z;
		return (x,y,z)


	def load_item(self, path, position, orientation):
		vid = self.p.createVisualShape(shapeType=self.p.GEOM_MESH, 
								   fileName=path)
		collisionShapeId = self.p.createCollisionShape(
								shapeType=self.p.GEOM_MESH,
		                        fileName=path)
		self.id = self.p.createMultiBody(
					baseCollisionShapeIndex=collisionShapeId,
					baseVisualShapeIndex=vid,
					basePosition=position,
					baseOrientation=orientation)

# self.lgripper = Grocery_item(0.2,-0.3,1., 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_left_finger.urdf",1,1,1,'lgripper','light',False)
# self.rgripper = Grocery_item(0.23,-0.3,1., 0,3.14,3.14, "gripper/wsg50_one_motor_gripper_right_finger.urdf",1,1,1,'rgripper','light',False)
class Shopping_List:
	def __init__(self, p):
		self.lgripper = Grocery_item(urdf_path="gripper/wsg50_one_motor_gripper_left_finger.urdf", 
			object_name='lgripper', urdf=True, p=p, x=-0.13, y=-.5, z=1.5, orr=0, op=3.14, oy=3.14)
		self.rgripper = Grocery_item(urdf_path="gripper/wsg50_one_motor_gripper_right_finger.urdf",
			object_name='rgripper',urdf=True, p=p,x=-0.1, y=-.5, z=1.5, orr=0, op=3.14, oy=3.14)
		self.baseball = Grocery_item(urdf_path='models/baseball/textured.obj',
			object_name='baseball', height=0.07, width=0.08, urdf=False, p=p)
		self.bottle_beer = Grocery_item(urdf_path='models/bottle_beer/model.sdf',
			object_name='beer', mass='heavy',height=0.25,width=0.08, urdf=True, p=p, x=0.15)
		self.can_coke = Grocery_item(urdf_path='models/can_coke/coke.obj',
			object_name='can_coke', height=0.17,width=0.08, orr=1.57, urdf=False, p=p, x=0.3)
		self.can_pepsi = Grocery_item(urdf_path='models/can_pepsi/pepsi.obj',
			object_name='can_pepsi',height=0.17,width=0.08, orr=1.57, urdf=False, p=p, x=0.45)
		self.can_fanta = Grocery_item(urdf_path='models/can_fanta/fanta.obj',
			object_name='can_fanta', height=0.17,width=0.08,orr=1.57, urdf=False, p=p, x=0.6)
		self.can_sprite = Grocery_item(urdf_path='models/can_sprite/sprite.obj',
			object_name='can_sprite',height=0.17, width=0.08,orr=1.57, urdf=False, p=p, x=0.75)
		self.chips_can = Grocery_item(urdf_path='models/chips_can/textured.obj',
			object_name='chips_can', height=0.3, width=0.08,urdf=False, p=p, x=0.9)
		self.coffee_box = Grocery_item(urdf_path='models/coffee_box/model.sdf',
			object_name='coffee_box', mass='heavy',width=0.08,height=0.17, oy=3.14, urdf=True, p=p, x=1.05,
			texture='models/coffee_box/materials/textures/coffee_box.png')
		self.cracker = Grocery_item(urdf_path='models/cracker/meshes/cracker.obj',
			object_name='cracker',mass='heavy', width=0.2,height=0.3, oy=-1.57, urdf=False, p=p, x=1.2)
		self.cup = Grocery_item(urdf_path='models/cup/textured.obj',
			object_name='cup',mass='heavy', height=0.1,width=0.08, urdf=False, p=p, x=1.35)
		self.donut = Grocery_item(urdf_path='models/donut/model.sdf',
			object_name='donut',height=0.041, width=0.1,urdf=True, p=p, x=1.5)
		self.fork = Grocery_item(urdf_path='models/fork/textured.obj',
			object_name='fork',dummy=True,height=0.1, width=0.05,urdf=False, p=p, x=1.65)
		self.gelatin = Grocery_item(urdf_path='models/gelatin/meshes/gelatin.obj',
			object_name='gelatin', mass='heavy',width=0.08,height=0.11,z=0.005, op=1.9,orr=1.65, urdf=False, p=p, x=1.8)
		self.meat = Grocery_item(urdf_path='models/meat/meshes/meat.obj',
			object_name='meat', mass='heavy',urdf=False,width=0.08, height=0.094, p=p, x=1.95)
		self.mustard = Grocery_item(urdf_path='models/mustard/meshes/mustard.obj',
			object_name='mustard',height=0.25, urdf=False, width=0.12,oy=3.14, p=p, x=2.05)
		self.newspaper = Grocery_item(urdf_path='models/newspaper/model.sdf',
			object_name='newspaper',height=0.005, urdf=True, p=p, x=2.2,
			texture='models/newspaper/materials/textures/news.png')
		self.orange = Grocery_item(urdf_path='models/orange/textured.obj',
			object_name='orange', height=0.076,width=0.08,urdf=False, p=p, x=2.35)
		self.pear = Grocery_item(urdf_path='models/pear/textured.obj',
			object_name='pear', height=0.076, width=0.08,urdf=False, p=p, y=-0.5)
		self.plate = Grocery_item(urdf_path='models/bowl/textured.obj',
			object_name='plate',mass='heavy', width=0.2,height=0.05, urdf=False, p=p, x=0.2, y=-0.5)
		self.soccer_ball = Grocery_item(urdf_path='models/soccer_ball/textured.obj',
			object_name='soccer_ball', dummy=True,mass='heavy',width=0.15,height=0.15, urdf=False, p=p, x=0.35, y=-0.5)
		self.soup = Grocery_item(urdf_path='models/soup/meshes/soup.obj',
			object_name='soup',mass='heavy',height=0.14, width=0.08,oy=3.14, urdf=False, p=p, x=0.5, y=-0.5)
		self.sponge = Grocery_item(urdf_path='models/sponge/textured.obj',
			object_name='sponge',dummy=True,height=0.1, oy=1.57,width=0.08, urdf=False, p=p, x=0.75, y=-0.5)
		self.sugar =  Grocery_item(urdf_path='models/sugar/meshes/sugar.obj',
			object_name='sugar',mass='heavy',width=0.08,height=0.2, oy=-1.57, urdf=False, p=p, x=0.9, y=-0.5)
		self.toy = Grocery_item(urdf_path='models/toy_airplane/textured.obj',
			object_name='toy',dummy=True,height=0.08,width=0.05, oy=2.2, urdf=False, p=p, x=1.05, y=-0.5)
		#################################################################
		

	def get_items_dict(self):
		self.items = {'baseball':self.baseball,
					  'beer':self.bottle_beer,
					  'can_coke':self.can_coke,
					  'can_pepsi':self.can_pepsi,
					  'can_fanta':self.can_fanta,
					  'can_sprite':self.can_sprite,
					  'chips_can':self.chips_can,
					  'coffee_box':self.coffee_box,
					  'cracker':self.cracker,
					  'cup': self.cup,
					  'donut':self.donut,
					  'fork':self.fork,
					  'gelatin':self.gelatin,
					  'meat':self.meat,
					  'mustard':self.mustard,
					  'newspaper':self.newspaper,
					  'orange':self.orange,
					  'pear':self.pear,
					  'plate':self.plate,
					  'soccer_ball':self.soccer_ball,
					  'soup':self.soup,
					  'sponge':self.sponge,
					  'sugar':self.sugar,
					  'toy':self.toy,
					  'lgripper':self.lgripper,
					  'rgripper':self.rgripper}
		return self.items


	def get_items_list(self):
		self.object_list = [self.baseball, self.bottle_beer,
		self.can_coke, self.can_pepsi, self.can_fanta,
		self.can_sprite, self.chips_can,self.coffee_box,self.cracker, 
		self.cup, self.donut,
		self.fork,self.gelatin,self.meat, self.mustard, self.newspaper,
		self.orange, self.pear, self.plate, self.soccer_ball,
		self.soup, self.sponge, self.sugar, self.toy]
		return self.object_list

	def get_item_string_list(self):
		oblist = self.get_items_list()
		slist = [it.name for it in oblist if (not it.dummy) and \
		(not it.name == 'lgripper') and (not it.name == 'rgripper')]
		return slist

if __name__ == '__main__':
	import pybullet as p
	physicsClient = p.connect(p.GUI)
	p.setAdditionalSearchPath('models')
	s = Shopping_List(p)
	print(s.get_item_string_list())