import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import time
import pybullet_data
import math
import copy
import threading
import numpy as np
from fd import Fast_Downward
from planner import Planner 
import rospy
from std_msgs.msg import String, Bool
from grocery_items import Shopping_List, Grocery_item
import mcts
import pomcp

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
print(pybullet_data.getDataPath())
p.setGravity(0,0,0)
p.setAdditionalSearchPath('models')
planeId = p.loadURDF("plane/plane.urdf") 
tableId = p.loadURDF("table/table.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]))

trayId = p.loadURDF("container/container.urdf", [-.5,.0,0.65],p.getQuaternionFromEuler([0,0,0]))


if __name__ == '__main__':
	shopping_list = Shopping_List(p)
	items = shopping_list.get_items_dict()
	itx = {k: items[k] for k in list(items)[:12]}	
	state_space = {'holding': None, 'items':itx}
	belief = [[('baseball',0.3), ('beer',0.5), ('can_coke',0.0), ('can_fanta',0.2),('can_pepsi',0.4)],
			[('baseball',0.2), ('beer',0.3), ('can_coke',0.6), ('can_fanta',0.5),('can_pepsi',0.6)],
			[('baseball',0.4), ('beer',0.5), ('can_coke',0.4), ('can_fanta',0.1),('can_pepsi',0.7)],
			[('baseball',0.6), ('beer',0.1), ('can_coke',0.1), ('can_fanta',0.3),('can_pepsi',0.9)],
			[('baseball',0.2), ('beer',0.6), ('can_coke',0.3), ('can_fanta',0.4),('can_pepsi',0.2)]
			 ]
	
	# print(len(itx))
	state = pomcp.State(state_space, belief)
	root_node = pomcp.Node(state)
	result_root = pomcp.perform_pomcp(root_node, num_iterations=1000)
	print(result_root)
	select_node = pomcp.select_action(result_root,infer=True)
	print(select_node.birth_action)
