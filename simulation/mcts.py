import numpy as np 
import copy

class State:
	def __init__(self, state_space):
		self.in_clutter = []
		self.in_box = []
		self.holding = None
		self.topfree = []
		self.on = []
		self.handempty = True
		self.state_space = state_space
		self.populate_state(state_space)

	def populate_state(self, state_space):
		if state_space['holding'] is None:
			self.handempty = True
		else:
			self.holding = state_space['holding']
		for item in state_space['items']:
			if item.inclutter:
				self.in_clutter.append(item.name)
			if item.inbox:
				self.in_box.append(item.name)
			it = item.item_on_top
			if it is None:
				self.topfree.append(item.name)
			else:
				self.on.append((it, item.name))


class Node:
	def __init__(self, state, parent=None):
		self.state = state
		self.parent = parent
		self.children = []
		self.value = 0
		self.visits = 0

	def add_child(self, child_node):
		self.children.append(child_node)

	def update(self, reward):
		self.value += reward 
		self.visits += 1

	def __repr__(self):
		s="Node; children: %d; visits: %d; reward: %f"%(len(self.children), self.visits, self.value)
		return s


def get_next_state_node(node, action):
	state = node.state 
	next_state = copy.deepcopy(state)
	if action[0] == 'put-in-box':
		next_state.in_box.append(action[1])
		next_state.holding = None
		next_state.topfree.append(action[1])
		next_state.handempty = True 

	elif action[0] == 'put-in-clutter':
		next_state.in_clutter.append(action[1])
		next_state.holding = None
		next_state.topfree.append(action[1])
		next_state.handempty = True 

	elif action[0] == 'put-on':
		next_state.on.append((action[1], action[2]))
		next_state.handempty = True
		next_state.holding = None 
		next_state.topfree.append(action[1])

	elif action[0] == 'pick-from-clutter':
		next_state.holding = action[1]
		next_state.handempty = False
		next_state.in_clutter.remove(action[1])
		try:
			next_state.topfree.remove(action[1])
		except:
			pass
		try:
			next_state.in_box.remove(action[1])
		except:
			pass

	elif action[0] == 'pick-from-box':
		next_state.holding = action[1]
		next_state.handempty = False
		next_state.in_box.remove(action[1])
		try:
			next_state.topfree.remove(action[1])
		except:
			pass
		try:
			next_state.in_clutter.remove(action[1])
		except:
			pass

	elif action[0] == 'pick-from':
		next_state.holding = action[1]
		next_state.handempty = False
		next_state.on.remove((action[1], action[2]))
		next_state.topfree.append(action[2])
		try:
			next_state.in_box.remove(action[1])
		except:
			pass
		try:
			next_state.in_clutter.remove(action[1])
		except:
			pass
		try:
			next_state.topfree.remove(action[1])
		except:
			pass
	next_node = Node(state=next_state, parent=node)
	return next_node


def get_valid_actions(node):
	state = node.state 
	actions = []

	if state.holding is not None:
		actions.append(('put-in-box', state.holding))
		actions.append(('put-in-clutter', state.holding))

		for item in state.in_box:
			actions.append(('put-on', state.holding, item))

		for item,_ in state.on:
			actions.append(('put-on', state.holding, item))

	else:
		for item in state.in_clutter:
			actions.append(('pick-from-clutter', item))

		for item in state.in_box:
			actions.append(('pick-from-box', item))

		for top,bottom in state.on:
			actions.append(('pick-from', top, bottom))

	return actions


def expand_node(node):
	actions = get_valid_actions(node)
	for action in actions:
		next_state_node = get_next_state_node(node, action)
		node.add_child(next_state_node)

	return node 


def backup(node, value):
	while node != None:
		node.visits += 1
		node.value += value
		node = node.parent
	return


def successful_packing(node):
	for item in node.state.in_box:
		items = node.state.state_space['items']
		if items[item].mass == 'light' and items[items[item].item_on_top] == 'heavy':
			return False
	return True


def rollout_policy(node):
	iteration = 0
	next_node = copy.deepcopy(node)
	while True:
		if len(next_node.state.in_clutter) == 0:
			if successful_packing(next_node):
				return 100
			else:
				return -10
		actions = get_valid_actions(next_node)
		random_action = np.random.choice(actions)
		next_node = get_next_state_node(next_node, random_action)
		iteration += 1
		if iteration > 1000000:
			return -10

#UCT / bestchild
def select_action(node):
	pass






