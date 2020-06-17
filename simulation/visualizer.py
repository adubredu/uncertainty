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
		self.stay_alive = True
		self.clock = pygame.time.Clock()

		self.window_width = 800
		self.window_height = 700
		self.duration = ''

		self.fps = 60
		self.win = pygame.display.set_mode((self.window_width,self.window_height))
		self.win.fill((255,255,255))

		self.object_coordinates = [(50,100), (150,100), (250,100), 
								   (50,200), (150,200), (250,200), 
								   (50,300), (150,300), (250,300), 
								   (50,400), (150,400), (250,400)]
		self.box_coordinates = [(50,400), (150,400), (250,400), (300,400),
							   (50,500), (150,500), (250,500),  (300,500),
							   (50,600), (150,600), (250,600), 	(300, 600)]
		# self.box_items = 'ambrosia_apple_banana_bottle_cereal_coke_lipton_lysol_milk_nutella_orange_oreo'

		# while not rospy.is_shutdown():
		# 	self.refresh_window()
		rospy.Subscriber('/scene_belief', String, self.belief_callback)
		rospy.Subscriber('/plan', String, self.plan_callback)
		rospy.Subscriber('/current_action', String, self.current_action_callback)
		rospy.Subscriber('/method', String, self.method_callback)
		rospy.Subscriber('/box_items', String, self.boxitems_callback)
		rospy.Subscriber('/time', String, self.time_callback)

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
		plan = self.plan.split('_')[:-1]
		for p in plan:
			if p == self.current_action:
				self.display_text(p, 360, y, (255,0,0), 15)
			else:
				self.display_text(p, 360, y, (0,0,0), 15)
			y+=20

		objects = self.scene_belief.split('_')[:-1]

		for name, (x,y) in zip(objects, self.object_coordinates):
			nc = name.split('-')
			nm = nc[0]; cf=nc[1]
			self.win.blit(pygame.image.load('assets/'+nm+'.jpg'), (x,y))
			self.display_text(cf, x, y-16, (0,0,0),14)


		bobjects = self.box_items.split('_')
		# print(bobjects)


		if not self.box_items == 'None' or not self.box_items == '':
			for name, (x,y) in zip(bobjects, self.box_coordinates):
				self.win.blit(pygame.image.load('assets/'+name+'.jpg'), (x,y))

		self.display_text('Items in Grocery Box', 10, 360, (0,0, 255), 17)
		self.display_text('Duration: '+self.duration+' seconds', 400, 600, (100,250,100), 18)
		self.display_text('Method: '+self.method, 400, 620, (0,100,250), 18)

		pygame.display.update()
		self.clock.tick(self.fps)




if __name__ == '__main__':
	rospy.init_node('visualizer')		
	v = visualizer()