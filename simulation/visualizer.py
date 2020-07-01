import rospy
from std_msgs.msg import String, Bool
import pygame
import time

pygame.init()
pygame.display.set_caption("Grocery Packing Visualizer")


class  visualizer:
	def __init__(self):
		
		self.scene_belief = 'None'
		self.current_action = 'None'
		self.plan = 'None' 
		self.box_items = 'None'
		self.method = 'None'
		self.holding = 'None'
		self.stay_alive = True
		self.clock = pygame.time.Clock()

		self.window_width = 800
		self.window_height = 700
		self.duration = ''

		self.fps = 60
		self.win = pygame.display.set_mode((self.window_width,self.window_height))
		self.win.fill((255,255,255))

		self.object_coordinates = [(10,100), (50,100), (90,100), (130,100),(170,100),(210,100),(250,100),(290,100),(330,100),
								   (10,200), (50,200), (90,200), (130,200),(170,200),(210,200),(250,200),(290,200),(330,200), 
								   (10,300), (50,300), (90,300), (130,300),(170,300),(210,300),(250,300),(290,300),(330,300)]
		self.box_coordinates = [(10,400), (50,400), (90,400), (130,400),(170,400),(210,400),(250,400),(290,400),(330,400),
								   (10,500), (50,500), (90,500), (130,500),(170,500),(210,500),(250,500),(290,500),(330,500), 
								   (10,600), (50,600), (90,600), (130,600),(170,600),(210,600),(250,600),(290,600),(330,600)]
		# self.box_items = 'ambrosia_apple_banana_bottle_cereal_coke_lipton_lysol_milk_nutella_orange_oreo'

		# while not rospy.is_shutdown():
		# 	self.refresh_window()
		rospy.Subscriber('/scene_belief', String, self.belief_callback)
		rospy.Subscriber('/plan', String, self.plan_callback)
		rospy.Subscriber('/current_action', String, self.current_action_callback)
		rospy.Subscriber('/method', String, self.method_callback)
		rospy.Subscriber('/box_items', String, self.boxitems_callback)
		rospy.Subscriber('/time', String, self.time_callback)
		rospy.Subscriber('/holding', String, self.holding_callback)


		rospy.spin()


	def belief_callback(self, inputs):
		self.scene_belief = inputs.data
		self.refresh_window()


	def boxitems_callback(self, inputs):
		self.box_items = inputs.data 
		self.refresh_window()


	def plan_callback(self, inputs):
		self.plan = inputs.data
		self.refresh_window()

	def holding_callback(self, inputs):
		self.holding = inputs.data 
		self.refresh_window()

	def time_callback(self, inputs):
		self.duration = inputs.data
		# self.refresh_window()

	def current_action_callback(self, inputs):
		self.current_action = inputs.data
		self.refresh_window()


	def method_callback(self, inputs):
		self.method = inputs.data
		self.refresh_window()


	def display_text(self,textcontent,x,y,color,font):
		font = pygame.font.Font('freesansbold.ttf',font)
		text = font.render(textcontent, True, color)
		self.win.blit(text, (x,y))


	def refresh_window(self):
		# objects = ['pepsi-0.8', 'nutella-0.8_coke-0.8_lipton-0.8',
		# 			'bleach-0.8', 'ambrosia-0.8', 'banana-0.8', 'cereal-0.8', 
		# 			'lysol-0.8_milk-0.8', 'oreo-0.8', 'tangerine-0.8']
		self.win.fill((255,255,255))
		self.display_text('Belief Space Confidence Probabilities', 20, 30, (0,0,255),17)
		pygame.draw.line(self.win, (0,0,0), (350,0), (350,700),2)
		self.display_text('Current Action: ', 360, 30, (0,0,255),17)
		self.display_text(self.current_action, 360, 60, (0,0,0), 15)
		pygame.draw.line(self.win, (0,0,0), (350,90), (800,90),2)
		self.display_text('Current Plan: ', 360, 120, (0,0,255),17)
		y = 150
		plan = self.plan.split('*')[:-1]
		for p in plan:
			if p == self.current_action:
				self.display_text(p, 360, y, (255,0,0), 15)
			else:
				self.display_text(p, 360, y, (0,0,0), 15)
			y+=20

		objects = self.scene_belief.split('*')[:-1]
		# print(objects)

		for name, (x,y) in zip(objects, self.object_coordinates):
			nc = name.split('-')
			nm = nc[0]; cf=nc[1]
			self.win.blit(pygame.image.load('assets/'+nm+'.png'), (x,y))
			self.display_text(cf, x, y-16, (0,0,0),14)


		bobjects = self.box_items.split('*')
		# print(bobjects)


		if not self.box_items == 'None' or not self.box_items == '':
			for name, (x,y) in zip(bobjects, self.box_coordinates):
				if name != '' and name!='None':
					self.win.blit(pygame.image.load('assets/'+name+'.png'), (x,y))

		self.display_text('Items in Grocery Box', 10, 360, (0,0, 255), 17)
		self.display_text('Duration: '+self.duration+' seconds', 400, 600, (100,250,100), 18)
		self.display_text('Method: '+self.method, 400, 620, (0,100,250), 18)

		self.display_text('Gripper holding: '+self.holding, 10, 620, (250,100,250), 18)


		pygame.display.update()
		self.clock.tick(self.fps)




if __name__ == '__main__':
	rospy.init_node('visualizer')		
	v = visualizer()