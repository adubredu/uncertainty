from viewer import DiscreteTAMPViewer, COLORS
import numpy as np 
from collections import namedtuple
import time

DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])
BLOCK_TEMPLATE = 'b{}'
INITIAL_CONF = np.array([0, -1])
GRASP = np.array([0, 0])

class testviewer:
	def __init__(self):
		n_rows = 5
		n_cols = 9
		self.viewer = DiscreteTAMPViewer(n_rows,n_cols, title='Initial',width=750, height=500)
		n_poses = 9
		n_blocks = 9

		blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
		poses = [np.array([x, n_rows-1]) for x in range(n_poses)]
		block_poses = dict(zip(blocks, poses))
		initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
		goal_poses = {blocks[0]: [1,-1]}

		self.tamp_problem = DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)

		self.colors = dict(zip(self.tamp_problem.initial.block_poses, COLORS))

	def draw_state(self, state):
		self.viewer.clear()
		self.viewer.draw_environment()
		self.viewer.draw_robot(*state.conf[::-1])
		# print(state)
		for block, pose in state.block_poses.items():
			r,c = pose[::-1]
			self.viewer.draw_block(r,c, name=block, color=self.colors[block])
		if state.holding is not None:
			pose = state.conf - GRASP
			r, c = pose[::-1]
			self.viewer.draw_block(r, c, name=state.holding, color=self.colors[state.holding])

	def apply_plan(self, plan):
		state  = self.tamp_problem.initial 
		self.draw_state(state)
		for action in plan:
			raw_input("Continue?")
			state = self.apply_action(state, action) 
			self.draw_state(state)
		raw_input("Finish?")

	def apply_action(self, state, action):
		conf, holding, block_poses = state 

		name, args = action 
		if name == 'move':
			_, conf = args 
		elif name == 'pick':
			holding, _,_ = args 
			del block_poses[holding]
		elif name == 'place':
			block, pose, _ = args 
			holding = None
			block_poses[block] = pose 
		elif name == 'push':
			block, _,_,pose, conf = args 
			holding =  None 
			block_poses[block]
		else:
			raise ValueError(name) 

		return DiscreteTAMPState(conf, holding, block_poses)


if __name__=='__main__':
	t = testviewer()
	# t.apply_plan()