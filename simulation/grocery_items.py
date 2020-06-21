#! /usr/bin/env python3
#! /usr/bin/env python3

class Grocery_item:
	def __init__(self, urdf_path, object_name, 
				p, x=0, y=0, z=0, orr=0, op=0, oy=0,  width=0.01, 
				breadth=0.01, height=0.01, mass=0.01,
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



class Shopping_List:
	def __init__(self, p):
		self.baseball = Grocery_item(urdf_path='models/baseball/textured.obj',
			object_name='baseball', urdf=False, p=p)
		self.bottle_beer = Grocery_item(urdf_path='models/bottle_beer/model.sdf',
			object_name='bottle_beer', urdf=True, p=p, x=0.15)
		self.can_coke = Grocery_item(urdf_path='models/can_coke/coke.obj',
			object_name='can_coke', orr=1.57, urdf=False, p=p, x=0.3)
		self.can_pepsi = Grocery_item(urdf_path='models/can_pepsi/pepsi.obj',
			object_name='can_pepsi', orr=1.57, urdf=False, p=p, x=0.45)
		self.can_fanta = Grocery_item(urdf_path='models/can_fanta/fanta.obj',
			object_name='can_fanta', orr=1.57, urdf=False, p=p, x=0.6)
		self.can_sprite = Grocery_item(urdf_path='models/can_sprite/sprite.obj',
			object_name='can_sprite', orr=1.57, urdf=False, p=p, x=0.75)


	def get_items_dict(self):
		self.items = {'baseball':self.baseball,
					  'bottle_beer':self.bottle_beer,
					  'can_coke':self.can_coke,
					  'can_pepsi':self.can_pepsi}
		return self.items


	def get_items_list(self):
		self.object_list = [self.baseball, self.bottle_beer,
		self.can_coke, self.can_pepsi]
		return self.object_list