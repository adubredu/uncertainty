import rospy
from std_msgs.msg import String, Bool
import pygame

pygame.init()
pygame.display.set_caption("Grocery Packing visualizer")


class  visualizer:
	def __init__(self):
		rospy.Subscriber('/scene_belief', String, self.belief_callback)
		rospy.Subscriber('/plan', String, self.plan_callback)
		rospy.Subscriber('/current_action', String, self.current_action_callback)
		rospy.Subscriber('/stay_alive', Bool, self.alive_callback)
		self.scene_belief = None
		self.current_action = None
		self.plan = None 
		self.stay_alive = True
		self.clock = pygame.time.Clock()

		self.window_width = 800
		self.window_height = 700

		self.fps = 60
		self.win = pygame.display.set_mode((self.window_width,self.window_height))


		self.object_coordinates = [(350,100), (400,100), (450,100), 
								   (350,200), (400,200), (450,200), 
								   (350,300), (400,300), (450,300), 
								   (350,400), (400,400), (450,400), 
								   (350,500), (400,500), (450,500), 
								   (350,600), (400,600), (450,600)]

		rospy.spin()


	def belief_callback(self, inputs):
		self.scene_belief = inputs.data


	def plan_callback(self, inputs):
		self.plan = inputs.data


	def current_action_callback(self, inputs):
		self.current_action = inputs.data


	def alive_callback(self, inputs):
		self.stay_alive = inputs.data


	def display_text(self,textcontent,x,y):
		font = pygame.font.Font('freesansbold.ttf',12)
		text = font.render(textcontent, True, (0,0,0))
		self.win.blit(text, (x,y))


	def run(self):
		objects = ['pepsi', 'nutella','coke','lipton',
					'bleach', 'ambrosia', 'banana', 'cereal', 'lysol',
					'milk', 'oreo', 'tangerine']
		while self.stay_alive:
			for name, (x,y) in zip(objects, self.object_coordinates):
				self.win.blit(pygame.image.load('assets/'+name+'.jpg'), (x,y))
			self.clock.tick(self.fps)




if __name__ == '__main__':
	rospy.init_node('visualizer')		
	v = visualizer()
	v.run()