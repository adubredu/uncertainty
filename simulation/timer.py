import rospy
from std_msgs.msg import String, Bool
import pygame
import time

pygame.init()
pygame.display.set_caption("Timer")


class  visualizer:
	def __init__(self):
		
		self.stay_alive = True
		self.clock = pygame.time.Clock()

		self.window_width = 100
		self.window_height = 50
		self.start = time.time()

		self.fps = 60
		self.win = pygame.display.set_mode((self.window_width,self.window_height))
		self.win.fill((255,255,255))


	def display_text(self,textcontent,x,y,color,font):
		font = pygame.font.Font('freesansbold.ttf',font)
		text = font.render(textcontent, True, color)
		self.win.blit(text, (x,y))

	def refresh_window(self):
		self.win.fill((255,255,255))
		now = str(int(time.time() - self.start))+'s'
		self.display_text(now, 30, 10, (0,0,0), 20)
		
		pygame.display.update()
		self.clock.tick(self.fps)



if __name__ == '__main__':	
	v = visualizer()
	while True:
		v.refresh_window()