import numpy as np 
import copy

num_items = 5
success = 0
class State:
	def __init__(self, state_space, scene_belief):
		self.in_clutter = []
		self.in_box = []
		self.holding = None
		self.topfree = []
		self.on = []
		self.handempty = True
		self.heaviness = {}
		self.state_space = state_space
		self.num_mc_samples = 100
		if scene_belief is not None:
			self.belief = scene_belief
		if state_space is not None:
			self.populate_state(state_space)

	def populate_state(self, state_space):
		#perform sampling from scene_belief here
		#this and the other will turn it into pomcp
		#mainly sampling items in clutter
		self.in_clutter = self.monte_carlo_sample()
		if state_space['holding'] is None:
			self.handempty = True
		else:
			self.holding = state_space['holding']
		items = list(state_space['items'].values())
		for item in items[:-2]:
			# if item.inclutter:
			# 	self.in_clutter.append(item.name)
			if item.inbox:
				self.in_box.append(item.name)
			it = item.item_on_top
			if it is None:
				self.topfree.append(item.name)
			else:
				self.on.append((it, item.name))
			self.heaviness[item.name] = item.mass

	def get_current_state(self):
		#mainly sampling items in clutter
		state = State(None,None)
		state.in_clutter = self.monte_carlo_sample()
		state.in_box = copy.deepcopy(self.in_box)
		state.holding = copy.deepcopy(self.holding)
		state.topfree = copy.deepcopy(self.topfree)
		state.on = copy.deepcopy(self.on)
		state.handempty = copy.deepcopy(self.handempty)
		state.heaviness = copy.deepcopy(self.heaviness)
		state.belief = copy.deepcopy(self.belief)

		return state

	def single_sample(self):
		sampled_items=[]
		for bunch in self.belief:
			# print(bunch)
			items = [b[0] for b in bunch]
			weights = [b[1] for b in bunch]
			weights = weights/np.sum(weights)
			sample = np.random.choice(items, size=1, p=weights)
			sampled_items.append(sample[0])

		return sampled_items


	def monte_carlo_sample(self):
		global num_items
		mc_counts={}
		items = list(set(self.in_clutter+self.in_box))
		for t in items: mc_counts[t] = 0
		mc_samples=[]
		for i in range(self.num_mc_samples):
			sampled_items = self.single_sample()
			joined=''
			for it in set(sampled_items):
				joined+= it+'*'
			mc_samples.append(joined[:-1])

		final_sample = max(set(mc_samples), key=mc_samples.count)
		sample = final_sample.split('*')

		for it in self.in_box:
			try:
				sample.remove(it)
			except:
				pass
		num_items = len(sample+self.in_box)
		return sample



class Node:
	def __init__(self, state, parent=None, action=None):
		self.state = state
		self.parent = parent
		self.children = []
		self.value = 0
		self.visits = 0
		self.birth_action = action

	def add_child(self, child_node):
		self.children.append(child_node)

	def update(self, reward):
		self.value += reward 
		self.visits += 1

	def get_num_children(self):
		return len(self.children)

	def __repr__(self):
		s="Node; children: %d; visits: %d; reward: %f"%(len(self.children), self.visits, self.value)
		return s


def get_next_state_node(node, action):
	# state = node.state 
	next_state = node.state.get_current_state()
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
		try:
			next_state.in_clutter.remove(action[1])
		except:
			pass
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
		# print('picking from box: ',action[1])
		# print(next_state.in_box)
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
	next_node = Node(state=next_state, parent=node, action=action)

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
		node.update(value)
		node = node.parent
	return


def successful_packing(node):
	for item in node.state.in_box:
		item_heaviness = node.state.heaviness[item]
		on_top_heaviness = None
		for top,bottom in node.state.on:
			if bottom == item:
				on_top_heaviness = node.state.heaviness[top]
		if item_heaviness == 'light' and on_top_heaviness == 'heavy':
			return False
	return True



def rollout_policy(node, depth=1000):
	global success
	iteration = 0
	next_node = node
	while True:
		if len(next_node.state.in_clutter) == 0 and \
			len(next_node.state.in_box) == num_items:
			if successful_packing(next_node):
				print('*'*50)
				print('success!')
				success += 1
				print(next_node.state.in_box)
				print('*'*50)
				return 100
			else:
				return -10
		actions = get_valid_actions(next_node)
		# print(actions)
		index = np.random.randint(len(actions))
		random_action = actions[index]
		# print(random_action)
		# print(next_node.state.in_box)
		next_node = get_next_state_node(next_node, random_action)
		# print('was successful')
		iteration += 1
		if iteration > depth:
			return -10

#UCT / bestchild
def select_action(node, infer=False):
	bestscore = -np.inf
	bestchildren = []
	for child in node.children:
		if child.visits != 0:
			exploit = child.value/child.visits
			explore = 2*np.sqrt(np.log(node.visits)/float(child.visits))
			score = exploit + explore
		else:
			score = np.inf
		if score == bestscore:
			bestchildren.append(child)
		if score > bestscore:
			bestchildren = [child]
			bestscore = score
	if infer:
		print('Best score is: ',bestscore)
	choice_node = np.random.choice(bestchildren)
	return choice_node


def perform_pomcp(root, num_iterations=100):
	global success
	iterr = 0
	root_node = root
	depth = 0
	while iterr < num_iterations:
		print('iteration %d. Num root children %d. Visits %d'%(iterr, len(root.children), root.visits))
		if root_node.get_num_children() == 0:
			if root_node.visits == 0:
				value = rollout_policy(root_node)
				backup(root_node, value)
				# print(root_node.state.in_box)
				root_node = root
				depth = 0
			else:
				expand_node(root_node)
				child_node = root_node.children[0]
				value = rollout_policy(child_node)
				backup(child_node, value)
				print('expanding')
				root_node = root
				depth = 0
		else:
			resultant_node = select_action(root_node)
			root_node = resultant_node
			depth +=1
			print('depth is ',depth)

		iterr += 1
	print('num successes is ',success)
	return root




# if __name__ == '__main__':



