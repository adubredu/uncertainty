import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame
import time
import math
import numpy as np
import random
import copy
from fd import Fast_Downward

# np.random.seed(437)
pygame.init()
pygame.display.set_caption("Grocery Packing")


class Box:
    def __init__(self, bottom_capacity):
        self.cpty = bottom_capacity
        self.index = 0
        self.old_index = 0
        self.lx = 260 
        self.ly = 290
        self.heights = [self.ly for i in range(self.cpty)]
        self.widths = [self.lx for i in range(self.cpty)]
        self.items_added = {}
        self.to_resolve = False
        self.num_items = 0

    def add_item(self, item):
        self.items_added[item.name] = self.index%self.cpty
        self.num_items+=1
        if self.index < self.cpty:
            x = self.widths[self.index%self.cpty] 
            y = self.ly - item.height 
            self.heights[self.index%self.cpty] = y
            if self.index+1 < self.cpty:
                self.widths[(self.index+1)%self.cpty] = x+ item.width
            


        else:
            x = self.widths[self.index%self.cpty]
            y = self.heights[self.index%self.cpty] - item.height
            self.heights[self.index%self.cpty] = y
            self.widths[(self.index+1)%self.cpty] = x+ item.width
        self.index += 1
        if self.to_resolve:
            self.index = copy.deepcopy(self.old_index)
            self.to_resolve = False
        return (x,y)

    def remove_item(self, item):
        if item.name in self.items_added:
            self.old_index = copy.deepcopy(self.index)
            self.index = self.items_added[item.name]
            self.to_resolve = True
            self.items_added.pop(item.name)
            self.num_items-=1






class Grocery_item:
    def __init__(self, x, y, image_path,image_width,image_height,
                object_name,cx, mass):
        self.x = x
        self.y = y
        self.cx = cx
        self.cy= 480-image_height
        self.width = image_width
        self.height = image_height
        self.body = pygame.image.load(image_path)
        self.item_at_left = None
        self.item_at_right = None
        self.item_on_top = None
        self.item_on_bottom = None
        self.inbox = False
        self.on_table = False
        self.on_clutter_or_table = False
        self.onsomething = False
        self.being_held = False
        self.name = object_name
        self.mass = mass
        self.weights = None
        self.holding = None #only for gripper


    def move_to(self, x, y):
        self.x = x
        self.y = y



class environment:
    def __init__(self, uncertain="low", declutter=False, order=0):
        self.window_width = 1000
        self.window_height = 480

        self.table = Grocery_item(150,300,'assets/table.jpg',419,144,"table",0,'heavy')
        self.pepsi = Grocery_item(10, 400,'assets/pepsi.jpg',26,49,"pepsi",200,'medium')
        self.nutella = Grocery_item(15, 400,'assets/nutella.jpg',26,37,"nutella",250,'medium')
        self.coke = Grocery_item(20, 400, 'assets/coke.jpg',28,52,"coke",300,'medium')
        self.lipton = Grocery_item(25, 400, 'assets/lipton.jpg',58,28,"lipton",350,'medium')
        self.bleach = Grocery_item(13, 400, 'assets/bleach.jpg',26,64,"bleach",400, 'heavy')

        self.ambrosia = Grocery_item(13, 400, 'assets/ambrosia.jpg',41,58,"ambrosia",430, 'heavy')
        self.banana = Grocery_item(13, 400, 'assets/banana.jpg',70,58,"banana",460, 'heavy')
        self.cereal = Grocery_item(13, 400, 'assets/cereal.jpg',30,48,"cereal",490, 'medium')
        self.lysol = Grocery_item(13, 400, 'assets/lysol.jpg',42,74,"lysol",520, 'heavy')
        self.milk = Grocery_item(13, 400, 'assets/milk.jpg',45,66,"milk",550, 'heavy')
        self.oreo = Grocery_item(13, 400, 'assets/oreo.jpg',28,17,"oreo",580, 'medium')
        self.tangerine = Grocery_item(13, 400, 'assets/tangerine.jpg',49,39,"tangerine",610, 'heavy')

        self.gripper = Grocery_item(350, 0,'assets/gripper.png',75,75,"gripper",0,'heavy')
        self.logo = Grocery_item(0,0, 'assets/4progress.png',535,78,"logo",0,'heavy')
        self.perceived = None
        self.false_positive_detections = False
        self.box = Box(5)
        self.planning_time = 0

        self.uncertainty = uncertain 
        self.declutter = declutter
        self.init_order = order
        self.declutter_domain = '/home/developer/uncertainty/pddl/dom.pddl'
        self.domain_path='/home/developer/uncertainty/pddl/belief_domain.pddl'
        self.problem_path='/home/developer/uncertainty/pddl/prob.pddl'
        self.definition = "(define (problem PACKED-GROCERY) \n (:domain GROCERY) \
                            \n (:objects bleach nutella coke pepsi lipton ambrosia banana cereal lysol milk tangerine oreo - item) \n"
        self.goal_def = "\n(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))\n"
        self.mid_matrix = {
                "pepsi":[0.4, 0.1,0.3,0.1,0.1,0,0,0,0,0,0,0],
                "nutella":[0.2,0.4,0.2,0.1,0.1,0,0,0,0,0,0,0],
                "coke":[0.3,0.2,0.4,0.05,0.05,0,0,0,0,0,0,0],
                "lipton":[0.2,0.1,0.1,0.4,0.2,0,0,0,0,0,0,0],
                "bleach":[0.1,0.1,0.1,0.1,0.6,0,0,0,0,0,0,0],
                "ambrosia":[0.1,0.1,0.1,0.1,0,0.6,0,0,0,0,0,0],
                "banana":[0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0,0],
                "cereal":[0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0],
                "lysol":[0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0],
                "milk":[0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0],
                "oreo":[0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0],
                "tangerine":[0,0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6]
            }

        self.low_matrix = {
                "pepsi":[0.9, 0.025,0.025,0.025,0.025,0,0,0,0,0,0,0],
                "nutella":[0.025,0.9,0.025,0.025,0.025,0,0,0,0,0,0,0],
                "coke":[0.025,0.025,0.9,0.025,0.025,0,0,0,0,0,0,0],
                "lipton":[0.025,0.025,0.025,0.9,0.025,0,0,0,0,0,0,0],
                "bleach":[0.025,0.025,0.025,0.025,0.9,0,0,0,0,0,0,0],
                "ambrosia":[0.1,0.1,0.1,0.1,0,0.6,0,0,0,0,0,0],
                "banana":[0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0,0],
                "cereal":[0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0],
                "lysol":[0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0],
                "milk":[0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0],
                "oreo":[0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0],
                "tangerine":[0,0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6]

        }

        self.high_matrix = {
                "pepsi":[0.25, 0.2,0.2,0.2,0.15,0,0,0,0,0,0,0],
                "nutella":[0.2,0.25,0.2,0.2,0.15,0,0,0,0,0,0,0],
                "coke":[0.2,0.2,0.25,0.15,0.2,0,0,0,0,0,0,0],
                "lipton":[0.2,0.15,0.2,0.25,0.2,0,0,0,0,0,0,0],
                "bleach":[0.2,0.2,0.2,0.15,0.25,0,0,0,0,0,0,0],
                "ambrosia":[0.1,0.1,0.1,0.1,0,0.6,0,0,0,0,0,0],
                "banana":[0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0,0],
                "cereal":[0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0,0],
                "lysol":[0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0,0],
                "milk":[0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0,0],
                "oreo":[0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6,0],
                "tangerine":[0,0,0,0,0,0,0,0.1,0.1,0.1,0.1,0.6]

            }

        self.items_in_clutter = 12

        self.items = {"pepsi":self.pepsi, "nutella":self.nutella,
                      "coke": self.coke, "lipton": self.lipton,
                      "bleach": self.bleach, "table":self.table,
                      "ambrosia":self.ambrosia, "banana":self.banana,
                      "cereal":self.cereal, "lysol":self.lysol,
                      "milk":self.milk, "oreo":self.oreo,
                      "tangerine":self.tangerine}

        self.belief_space = {"pepsi":{"belief":"pepsi", "weights":[], "un":''}, 
                            "nutella":{"belief":"nutella", "weights":[], "un":''},
                            "coke": {"belief":"coke", "weights":[], "un":''}, 
                            "lipton": {"belief":"lipton", "weights":[], "un":''},
                            "bleach": {"belief":"bleach", "weights":[], "un":''},
                            "ambrosia": {"belief":"ambrosia", "weights":[], "un":''},
                            "banana": {"belief":"banana", "weights":[], "un":''},
                            "cereal": {"belief":"cereal", "weights":[], "un":''},
                            "lysol": {"belief":"lysol", "weights":[], "un":''},
                            "milk": {"belief":"milk", "weights":[], "un":''},
                            "oreo": {"belief":"oreo", "weights":[], "un":''},
                            "tangerine": {"belief":"tangerine", "weights":[], "un":''}}

        self.objects_list = [self.pepsi, self.nutella,self.coke,self.lipton,
                    self.bleach, self.ambrosia, self.banana, self.cereal, self.lysol,
                    self.milk, self.oreo, self.tangerine]

        self.clock = pygame.time.Clock()
        self.current_action = "Action: (pick-up-from-on nutella bleach)"
        self.certainty_level = "Uncertainty Level: "+uncertain
        self.clutter_strategy = "Clutter Strategy: Declutter first" if declutter else "Clutter Strategy: Optimistic"
        
        self.win = pygame.display.set_mode((self.window_width,self.window_height))
        # self.populate_belief_space()
        self.start_time = time.time()
        self.initialize_clutter()   
        
        self.rate = 120

    def update_items_left(self):
        inbox = []
        for item in self.objects_list:
            if item.on_table:
                inbox.append(item.name)
        for item in self.objects_list:
            if item.item_on_bottom in inbox:
                inbox.append(item.name)
        for item in self.objects_list:
            if item.item_on_bottom in inbox:
                inbox.append(item.name)
        for item in self.objects_list:
            if item.item_on_bottom in inbox:
                inbox.append(item.name)
        inbox = list(set(inbox))
        return self.items_in_clutter == len(inbox)




    def populate_belief_space(self):
        for key  in self.belief_space:
            if self.items[key].item_on_top == None:
                self.belief_space[key]['weights'] =  self.low_matrix[key]
                self.belief_space[key]['un'] = 'low'
            else:
                self.belief_space[key]['weights'] =  self.mid_matrix[key]
                self.belief_space[key]['un'] = 'mid'
           
        seen_objects=[]
        for key in self.belief_space:
            while True:
                choice = np.random.choice(self.objects_list,
                    size=1, p=self.belief_space[key]['weights'])
                name = choice[0].name

                if name not in seen_objects:
                    seen_objects.append(name)
                    self.belief_space[key]['belief'] = name
                    break
            print(key+":"+self.belief_space[key]['belief']+':'+self.belief_space[key]['un'])
        

    def display_icon(self, body, x,y):
        self.win.blit(pygame.transform.scale(body,(15,30)),(x,y))


    def display_belief_space(self):
        ix = 410+30
        iy = 200+30
        self.display_text('GroundTruth --> Belief',200,150,12)
        y=96

        d = self.belief_space
        d1 = dict(d.items()[len(d)/2:])
        d2 = dict(d.items()[:len(d)/2])

        for true_state in d1:
            # print(d1)
            ts = true_state
            bf = self.belief_space[true_state]['belief']
            self.display_icon(self.items[ts].body, ix+160, iy+y)
            self.display_icon(self.items[bf].body, ix+160+50, iy+y)
            y+=30

        for true_state in d2:
            ts = true_state
            bf = self.belief_space[true_state]['belief']
            self.display_icon(self.items[ts].body, ix+100+160, iy+y)
            self.display_icon(self.items[bf].body, ix+160+100+50, iy+y)
            y+=30




    def run(self):
        if self.declutter:
            self.declutter_before_clutter_planning()
        else:
            self.clutter_optimistic_planning()

    def sample_object(self, object_name):
        if self.uncertainty == "low":
            return self.items[object_name]

        probabilities = self.mid_matrix if self.uncertainty=="medium" else self.high_matrix

        choice = np.random.choice(self.objects_list, size=1, 
                p=probabilities[object_name])

        decision = choice[0]

        count = 0
        while decision.onsomething:
            # print("not choosing "+choice[0].name+". Choosing from clutter")
            choice = np.random.choice(self.objects_list, size=1, 
                p=probabilities[object_name])
            count+=1
            if count > 20:
                return self.items[object_name]


        return decision


    def initialize_clutter(self):
        # choice = 0#np.random.randint(4)
        choice = self.init_order

        if choice == 0:
            self.bleach.item_at_left = "lipton"
            self.lipton.item_at_right = "bleach"
            self.bleach.item_on_top = "pepsi"
            self.pepsi.onsomething = True
            self.bleach.item_at_right = "coke"
            self.coke.item_at_left = "bleach"
            self.coke.item_on_top = "nutella"
            self.nutella.onsomething = True
            self.pepsi.item_at_right = "nutella"
            self.nutella.item_at_left = "coke"
            self.coke.item_at_right = "tangerine"
            self.tangerine.item_on_top = "oreo"
            self.oreo.item_on_top = "ambrosia"
            self.nutella.item_on_top = "lysol"
            self.pepsi.item_on_top = "cereal"
            self.lipton.item_on_top = "milk"
            self.milk.item_on_top = "banana"
            self.lipton.on_clutter_or_table = True
            self.bleach.on_clutter_or_table = True
            self.coke.on_clutter_or_table = True
            self.tangerine.on_clutter_or_table = True 

        elif choice == 1:
            self.lipton.item_at_left = "coke"
            self.coke.item_at_right = "lipton"
            self.lipton.item_on_top = "nutella"
            self.nutella.onsomething = True
            self.lipton.item_at_right = "bleach"
            self.bleach.item_at_left = "lipton"
            self.bleach.item_at_right = "pepsi"
            self.pepsi.item_at_left = "bleach"
            self.coke.item_on_top = "tangerine"
            self.tangerine.item_on_top = "banana"
            self.nutella.item_on_top = "ambrosia"
            self.bleach.item_on_top = "milk"
            self.milk.item_on_top = "lysol"
            self.pepsi.item_on_top = "oreo"
            self.oreo.item_on_top = "cereal"
            self.coke.on_clutter_or_table = True
            self.lipton.on_clutter_or_table = True
            self.bleach.on_clutter_or_table = True
            self.pepsi.on_clutter_or_table = True

        elif choice == 2:
            self.lipton.item_at_left = "nutella"
            self.nutella.item_at_right = "lipton"

            self.lipton.item_at_right = "pepsi"
            self.pepsi.item_at_left = "lipton"

            self.lipton.item_on_top = "coke"
            self.coke.onsomething = True
            self.coke.item_on_top = "bleach"
            self.bleach.onsomething  =True

            self.nutella.item_on_top = "tangerine"
            self.tangerine.item_on_top = "milk"
            self.bleach.item_on_top = "banana"
            self.pepsi.item_on_top = "lysol"
            self.cereal.item_on_top = "ambrosia"
            self.oreo.on_clutter_or_table = True 
            self.cereal.on_clutter_or_table = True 

            self.nutella.on_clutter_or_table = True
            self.lipton.on_clutter_or_table = True 
            self.pepsi.on_clutter_or_table = True

        elif choice == 3:
            self.lipton.item_on_top = "pepsi"
            self.pepsi.onsomething = True

            self.lipton.item_at_right = "coke"
            self.coke.item_at_left = "lipton"

            self.coke.item_on_top = "bleach"
            self.bleach.onsomething = True

            self.coke.item_at_right = "nutella"
            self.nutella.item_at_left = "coke"

            self.pepsi.item_on_top = "oreo"
            self.oreo.item_on_top = "milk"
            self.bleach.item_on_top = "tangerine"
            self.nutella.item_on_top = "lysol"
            self.oreo.on_clutter_or_table = True 
            self.cereal.on_clutter_or_table = True 
            self.cereal.item_on_top = "banana"

            self.lipton.on_clutter_or_table = True
            self.coke.on_clutter_or_table = True
            self.nutella.on_clutter_or_table = True

        elif choice == 4:
            self.milk.item_on_top = "bleach"
            self.bleach.item_on_top = "oreo"
            self.milk.on_clutter_or_table = True
            self.milk.item_at_right = "tangerine"
            self.tangerine.item_on_top = "lysol"
            self.lysol.item_on_top = "pepsi"
            self.tangerine.on_clutter_or_table = True 
            self.tangerine.item_at_right = "ambrosia"
            self.ambrosia.on_clutter_or_table = True 
            self.ambrosia.item_on_top = "cereal"
            self.cereal.item_on_top = "coke"
            self.ambrosia.item_at_right = "banana"
            self.banana.on_clutter_or_table = True 
            self.banana.item_on_top = "nutella"
            self.nutella.item_on_top = "lipton"

        self.draw_init_clutter(choice)


    def draw_init_clutter(self, choice):
        if choice == 0:
            self.lipton.x = 10
            self.lipton.y = self.window_height-self.lipton.height
            self.milk.x = 10 
            self.milk.y = self.window_height-self.lipton.height-self.milk.height
            self.banana.x = 10
            self.banana.y = self.window_height-self.lipton.height-self.milk.height - self.banana.height
            self.bleach.x = 10+self.lipton.width
            self.bleach.y = self.window_height-self.bleach.height
            self.coke.x = 10+self.lipton.width+self.bleach.width
            self.coke.y = self.window_height-self.coke.height
            self.pepsi.x = 10+self.milk.width
            self.pepsi.y = self.window_height-self.bleach.height-self.pepsi.height
            self.cereal.x = 10+self.banana.width
            self.cereal.y = self.window_height-self.bleach.height-self.pepsi.height-self.cereal.height
            self.nutella.x = 10+self.milk.width+self.pepsi.width
            self.nutella.y = self.window_height - self.coke.height-self.nutella.height
            self.lysol.x = 10+self.banana.width+self.cereal.width
            self.lysol.y = self.window_height - self.coke.height-self.nutella.height-self.lysol.height 
            self.tangerine.x = 10+self.lipton.width+self.bleach.width+self.coke.width
            self.tangerine.y = self.window_height-self.tangerine.height
            self.oreo.x = 10+self.milk.width+self.pepsi.width+self.nutella.width 
            self.oreo.y = self.window_height-self.tangerine.height-self.oreo.height 
            self.ambrosia.x = 10+self.banana.width+self.cereal.width+self.lysol.width 
            self.ambrosia.y = self.window_height-self.tangerine.height-self.oreo.height-self.ambrosia.height

        elif choice ==1:
            self.coke.x = 10
            self.tangerine.x = 10
            self.banana.x = 10
            self.lipton.x = 10+self.coke.width 
            self.ambrosia.x = 10+self.banana.width
            self.bleach.x = 10+self.coke.width+self.lipton.width
            self.pepsi.x = 10+self.coke.width+self.lipton.width+self.bleach.width
            self.nutella.x = 10+self.tangerine.width 
            self.milk.x = 10+self.tangerine.width+self.nutella.width 
            self.lysol.x = 10+self.banana.width+self.ambrosia.width
            self.oreo.x = 10+self.tangerine.width+self.nutella.width+self.milk.width
            self.cereal.x = 10+self.banana.width+self.ambrosia.width+self.lysol.width

            self.coke.y = self.window_height-self.coke.height 
            self.tangerine.y = self.window_height-self.coke.height-self.tangerine.height
            self.banana.y = self.window_height-self.coke.height-self.tangerine.height-self.banana.height 
            self.lipton.y = self.window_height-self.lipton.height 
            self.bleach.y = self.window_height-self.bleach.height 
            self.milk.y = self.window_height-self.bleach.height-self.milk.height
            self.lysol.y = self.window_height-self.bleach.height-self.milk.height-self.lysol.height
            self.pepsi.y = self.window_height-self.pepsi.height 
            self.oreo.y = self.window_height-self.pepsi.height - self.oreo.height
            self.cereal.y = self.window_height-self.pepsi.height - self.oreo.height-self.cereal.height
            self.nutella.y = self.window_height-self.lipton.height-self.nutella.height
            self.ambrosia.y = self.window_height-self.lipton.height-self.nutella.height-self.ambrosia.height


        elif choice == 2:
            self.nutella.x = 10
            self.tangerine.x = 10 
            self.milk.x = 10
            self.lipton.x = 10+self.nutella.width 
            self.pepsi.x = 10+self.nutella.width +self.lipton.width
            self.cereal.x = 10+self.nutella.width +self.lipton.width+self.pepsi.width 
            self.oreo.x = 10+self.nutella.width +self.lipton.width+self.pepsi.width+self.cereal.width
            self.coke.x = 10+self.tangerine.width
            self.lysol.x = 10+self.tangerine.width+self.coke.width 
            self.ambrosia.x = 10+self.tangerine.width+self.coke.width+self.lysol.width
            self.bleach.x = 10+self.milk.width
            self.banana.x = 10+self.milk.width

            self.nutella.y = self.window_height-self.nutella.height 
            self.tangerine.y = self.window_height-self.nutella.height-self.tangerine.height
            self.milk.y = self.window_height-self.nutella.height-self.tangerine.height-self.milk.height
            self.lipton.y = self.window_height-self.lipton.height
            self.pepsi.y = self.window_height-self.pepsi.height 
            self.coke.y = self.window_height-self.lipton.height-self.coke.height 
            self.bleach.y = self.window_height-self.lipton.height-self.coke.height-self.bleach.height
            self.banana.y = self.bleach.y - self.banana.height
            self.lysol.y = self.pepsi.y-self.lysol.height
            self.cereal.y = self.window_height-self.cereal.height
            self.ambrosia.y = self.cereal.y-self.ambrosia.height
            self.oreo.y = self.window_height-self.oreo.height

        elif choice == 3:
            self.lipton.x = 10
            self.milk.x=10
            self.oreo.x=10
            self.tangerine.x = 10+self.oreo.width
            self.coke.x = 10+self.lipton.width
            self.nutella.x = 10+self.lipton.width+self.coke.width 
            self.pepsi.x = 10
            self.bleach.x = 10+self.pepsi.width
            self.lysol.x = 10+self.pepsi.width+self.bleach.width 
            self.ambrosia.x = 10+self.lipton.width+self.coke.width+self.nutella.width
            self.cereal.x = self.ambrosia.x+self.cereal.width
            self.banana.x = self.cereal.x

            self.lipton.y = self.window_height - self.lipton.height
            self.coke.y = self.window_height-self.coke.height
            self.nutella.y = self.window_height-self.nutella.height
            self.lysol.y = self.window_height-self.nutella.height-self.lysol.height
            self.ambrosia.y = self.window_height-self.ambrosia.height
            self.cereal.y = self.window_height-self.cereal.height
            self.banana.y = self.window_height-self.cereal.height-self.banana.height
            self.pepsi.y = self.window_height - self.lipton.height-self.pepsi.height 
            self.bleach.y = self.window_height-self.coke.height - self.bleach.height
            self.tangerine.y = self.window_height-self.coke.height - self.bleach.height-self.tangerine.height
            self.oreo.y = self.pepsi.y - self.oreo.height 
            self.milk.y = self.oreo.y - self.milk.height 

        elif choice == 4:
            self.milk.x = 10
            self.tangerine.x = self.milk.x + self.milk.width 
            self.ambrosia.x = self.tangerine.x + self.tangerine.width
            self.banana.x =self.ambrosia.x + self.ambrosia.width
            self.bleach.x = 10
            self.lysol.x = self.bleach.x + self.bleach.width
            self.cereal.x = self.lysol.x + self.lysol.width
            self.nutella.x = self.cereal.x + self.cereal.width
            self.oreo.x = 10
            self.pepsi.x = self.oreo.x + self.oreo.width 
            self.coke.x = self.pepsi.x + self.pepsi.width 
            self.lipton.x = self.coke.x + self.coke.width 

            self.milk.y = self.window_height - self.milk.height 
            self.bleach.y = self.milk.y - self.bleach.height 
            self.oreo.y = self.bleach.y - self.oreo.height 
            self.tangerine.y = self.window_height - self.tangerine.height 
            self.lysol.y = self.tangerine.y - self.lysol.height 
            self.pepsi.y = self.lysol.y - self.pepsi.height 
            self.ambrosia.y = self.window_height - self.ambrosia.height 
            self.cereal.y = self.ambrosia.y - self.cereal.height 
            self.coke.y = self.cereal.y - self.coke.height 
            self.banana.y = self.window_height - self.banana.height 
            self.nutella.y = self.banana.y - self.nutella.height 
            self.lipton.y = self.nutella.y - self.lipton.height



    def display_text(self,textcontent,w,h,font):
        font = pygame.font.Font('freesansbold.ttf',font)
        text = font.render(textcontent, True, (0,0,0))
        self.win.blit(text, (410+h,200+w))



    def redrawGameWindow(self):
        self.win.fill((255,255,255))
        self.win.blit(pygame.image.load('assets/box.jpg'), (250,160))
        self.win.blit(pygame.image.load('assets/box.jpg'), (493,160))
        self.win.blit(pygame.transform.scale(pygame.image.load('assets/box_lat.png'),(243,11)), (260,290))
        self.win.blit(self.logo.body, (self.logo.x, self.logo.y))
        self.win.blit(pygame.image.load('assets/rt.jpg'), (550,10))
        self.win.blit(self.table.body,(self.table.x+20, self.table.y))

        self.win.blit(self.pepsi.body,(self.pepsi.x, self.pepsi.y))
        self.win.blit(self.nutella.body,(self.nutella.x, self.nutella.y))
        self.win.blit(self.coke.body,(self.coke.x, self.coke.y))
        self.win.blit(self.lipton.body,(self.lipton.x, self.lipton.y))
        self.win.blit(self.bleach.body,(self.bleach.x, self.bleach.y))
        self.win.blit(self.ambrosia.body,(self.ambrosia.x, self.ambrosia.y))
        self.win.blit(self.banana.body,(self.banana.x, self.banana.y))
        self.win.blit(self.cereal.body,(self.cereal.x, self.cereal.y))
        self.win.blit(self.lysol.body,(self.lysol.x, self.lysol.y))
        self.win.blit(self.oreo.body,(self.oreo.x, self.oreo.y))
        self.win.blit(self.tangerine.body,(self.tangerine.x, self.tangerine.y))
        self.win.blit(self.milk.body,(self.milk.x, self.milk.y))

        self.win.blit(self.gripper.body,(self.gripper.x, self.gripper.y))
        
        self.display_text(self.current_action,0,200,14)
        self.duration = int(time.time()-self.start_time)
        self.duration_in_sec = "Duration: "+str(self.duration)+ " seconds"
        self.display_text(self.duration_in_sec, 20,200,14)
        self.display_text("Uncertainty Level: "+self.uncertainty, 40,200,14)
        self.display_text(self.clutter_strategy, 60,200,14)
        # self.display_belief_space()

        if self.perceived is not None:
            self.win.blit(pygame.transform.scale(self.perceived.body,(15,30)),(585,20))        
        
        pygame.display.update()

    def execute_action(self, action):
        if action[0] == 'pick-up':
            self.pick_up(action[1])
        elif action[0] == 'pick-up-from-on':
            self.pick_up(action[1])
        elif action[0] == 'put-on-table':
            self.put_in_box(action[1])
        elif action[0] == 'put-on':
            self.put_on(action[1], action[2])
        elif action[0] == 'put-left':
            self.put_left(action[1],action[2])
        elif action[0] == 'put-right':
            self.put_right(action[1], action[2])
        elif action[0] == 'drop-in-clutter':
            self.drop_in_clutter(action[1])


    

    def inspect_scene(self, action):
        result = True
        # for action in progress:
        if action[0] == 'put-on-table':
            result = result and self.check_on_table(action[1])
            if not result:
                print("ERROR in put-on-table")
        elif action[0] == 'put-on':
            result = result and self.check_on(action[1],action[2])
            if not result:
                print("ERROR in put-on")
        elif action[0] == 'put-left':
            result = result and self.check_left(action[1],action[2])
            if not result:
                print("ERROR in put-left")
        elif action[0] == 'put-right':
            result = result and self.check_right(action[1],action[2])
            if not result:
                print("ERROR in put-right")
        elif action[0] == 'pick-up':
            result = result and (self.gripper.holding == action[1])
            if not result:
                print("ERROR in holding")
        elif action[0] == 'pick-up-from-on':
            result = result and (self.gripper.holding == action[1])
            if not result:
                print("ERROR in pick-up-from")
        return result


    def check_on_table(self, item_name):
        item = self.items[item_name]
        return item.on_table
        # if item.y == 260:
        #     return True
        # else:
        #     return False


    def check_left(self, left_item_name, right_item_name):
        left_item = self.items[left_item_name]
        right_item = self.items[right_item_name]

        return (right_item.item_at_left == left_item_name)
        # if (right_item.x - left_item.x) == 50:
        #     return True
        # else:
        #     return False


    def check_right(self, left_item_name, right_item_name):
        left_item = self.items[left_item_name]
        right_item = self.items[right_item_name]

        return (right_item.item_at_right == left_item_name)

        # if (left_item.x - right_item.x) == 50:
        #     return True
        # else:
        #     return False


    def check_on(self, top_item_name, bot_item_name):
        top_item = self.items[top_item_name]
        bot_item = self.items[bot_item_name]

        return (bot_item.item_on_top == top_item_name)

        # if (bot_item.y - top_item.y) == 64:
        #     return True
        # else:
        #     return False


    def get_current_packing_state(self):
        init = "\n(:init "
        for item in self.objects_list:
            if item.item_at_left == None:
                init+=" (clearleft "+self.belief_space[item.name]['belief']+")"
            else:
                init+=" (toleft "+self.belief_space[item.item_at_left]['belief']+" "+ self.belief_space[item.name]['belief']+")"

            if item.item_at_right == None:
                init+=" (clearright "+self.belief_space[item.name]['belief']+")"
            else:
                init+=" (toright "+self.belief_space[item.item_at_right]['belief']+" "+self.belief_space[item.name]['belief']+")"

            
            if item.item_on_top == None:
                init+=" (topfree "+self.belief_space[item.name]['belief']+")"
            else:
                init+=" (on "+self.belief_space[item.item_on_top]['belief']+" "+self.belief_space[item.name]['belief']+")"

            if item.on_table:
                init+=" (ontable "+self.belief_space[item.name]['belief']+")"

            if item.on_clutter_or_table:
                init+=" (onclutterortable "+self.belief_space[item.name]['belief']+")"


            if item.onsomething:
                init+=" (onsomething "+self.belief_space[item.name]['belief']+")"

        if self.gripper.holding == None:
            init+=" (handempty) "
        else:
            init+=" (holding "+self.gripper.holding+") "
        init+=")\n"
        return init


    def get_scene_for_declutter(self):
        init = "\n(:init (handempty)"
        for item in self.objects_list:
            if item.item_on_top == None:
                init+=" (topfree "+self.belief_space[item.name]['belief']+")"
            else:
                init+=" (on "+self.belief_space[item.item_on_top]['belief']+" "+self.belief_space[item.name]['belief']+")"
            init += " (inclutter "+self.belief_space[item.name]['belief']+")"
        init += ")\n"
        return init 

    def form_problem_from_current_scene(self):
        init = self.get_current_packing_state()
        prob = self.definition+init+self.goal_def
        f = open("newprob.pddl","w")
        f.write(prob)
        f.close()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        prob_path = dir_path+"/"+"newprob.pddl"
        
        return prob_path

    def form_dec_problem_from_current_scene(self):
        init = self.get_current_packing_state()
        goal = "\n(:goal (and (topfree coke) \
        (topfree lipton) (topfree nutella) \
        (topfree pepsi) (topfree bleach))))"
        prob = self.definition+init+goal
        f = open("newprob.pddl","w")
        f.write(prob)
        f.close()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        prob_path = dir_path+"/"+"newprob.pddl"
        
        return prob_path


    def clutter_optimistic_planning(self):
        start_time = time.time()
        self.initialize_clutter()
        problem = self.form_problem_from_current_scene()
        self.run_grocery_packing(self.domain_path, problem) 
        duration = time.time() - start_time
        print("\n\n DURATION OF OPTIMISTIC IS "+str(duration)+" seconds")
        time.sleep(3)
        pygame.quit()


    def perform_declutter(self):
        init = self.get_scene_for_declutter()
        goal = "\n(:goal (and (topfree coke) \
        (topfree lipton) (topfree nutella) \
        (topfree pepsi) (topfree bleach) (topfree ambrosia)\
        (topfree banana) (topfree cereal) (topfree lysol)\
        (topfree milk) (topfree oreo) (topfree tangerine)\
        (inclutter nutella) (inclutter pepsi) (inclutter bleach) \
        (inclutter coke) (inclutter lipton)\
        (inclutter ambrosia) (inclutter banana) (inclutter milk)\
        (inclutter cereal) (inclutter oreo)\
        (inclutter tangerine) (inclutter lysol))))"
        problem = self.definition+init+goal
        file = open("declutterprob.pddl",'w')
        file.write(problem)
        file.close()
        clutter_prob_path = os.path.dirname(os.path.realpath(__file__))+\
                    "/"+"declutterprob.pddl" 
        uncert = copy.deepcopy(self.uncertainty)
        self.uncertainty = 'low'
        self.run_dec_grocery_packing(self.domain_path, clutter_prob_path)
        print("***Declutter Complete***")
        self.uncertainty = uncert


    def declutter_before_clutter_planning(self):
        start_time = time.time()
        self.initialize_clutter()
        init = self.get_current_packing_state()
        goal = "\n(:goal (and (topfree coke) \
        (topfree lipton) (topfree nutella) \
        (topfree pepsi) (topfree bleach))))"
        problem = self.definition+init+goal
        file = open("declutterprob.pddl",'w')
        file.write(problem)
        file.close()
        clutter_prob_path = os.path.dirname(os.path.realpath(__file__))+\
                    "/"+"declutterprob.pddl" 
        uncert = copy.deepcopy(self.uncertainty)
        self.uncertainty = 'low'
        self.run_dec_grocery_packing(self.domain_path, clutter_prob_path)
        print("***Declutter Complete***")
        self.uncertainty = uncert

        inits = self.get_current_packing_state()
        problems = self.definition+inits+self.goal_def
        file = open("probs.pddl",'w')
        file.write(problems)
        file.close()
        probs_path = os.path.dirname(os.path.realpath(__file__))+\
                    "/"+"probs.pddl"
        self.run_grocery_packing(self.domain_path, probs_path)
        print("***GROCERY PACKING COMPLETE***")
        duration = time.time() - start_time
        print("\n\nDURATION OF DECLUTTER IS "+str(duration)+" seconds")
        pygame.quit()

        

    def run_grocery_packing(self,domain_path, problem_path):
        # action_progress=[] 
        f = Fast_Downward()
        plan = f.plan(domain_path, problem_path)
        self.populate_belief_space()
        # print(plan)
        if plan is None or len(plan)==0:
            print('No valid plan found')
            return
        else:
            for action in plan:
                self.redrawGameWindow()               
                print('Performing action: '+str(action))
                self.current_action = "Action: "+str(action)
                self.execute_action(action)
                inspection_result = self.inspect_scene(action)
                if not inspection_result:
                    plan = None
                    self.current_action = "Action: REPLANNING..."                   
                    self.redrawGameWindow()
                    print('****************')
                    print('REPLANNING...')
                    print('****************')
                    time.sleep(3)
                    prob_path = self.form_problem_from_current_scene()
                    self.run_grocery_packing(domain_path, prob_path)
                    break
            return


        
        time.sleep(2)

    def run_dec_grocery_packing(self,domain_path, problem_path):
        # action_progress=[] 
        f = Fast_Downward()
        plan = f.plan(domain_path, problem_path)
        # print(plan)
        if plan is None or len(plan)==0:
            print('No valid plan found')
            return
        else:
            for action in plan:
                self.redrawGameWindow()               
                print('Performing action: '+str(action))
                self.current_action = "Action: "+str(action)
                self.declutter_belief_execute_action(action)
                inspection_result = self.inspect_scene(action)
                if not inspection_result:
                    self.current_action = "Action: REPLANNING..."
                    self.redrawGameWindow()
                    print('****************')
                    print('REPLANNING...')
                    print('****************')
                    time.sleep(3)
                    prob_path = self.form_dec_problem_from_current_scene()
                    self.run_dec_grocery_packing(domain_path, prob_path)
                    break


    def pick_motion(self, item):
        orig_x = self.gripper.x
        orig_y = self.gripper.y 
        while math.fabs(item.x - (self.gripper.x+26)) > 0:
            if item.x - (self.gripper.x+26) > 0:
                self.gripper.x += 1
            elif item.x - (self.gripper.x+26) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)
        # print('done moving left')

        while math.fabs(item.y - (self.gripper.y+90)) > 0:
            if item.y - (self.gripper.y+90) > 0:
                self.gripper.y += 1
            elif item.y - (self.gripper.y+90) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('done moving right')
        # time.sleep(2)
        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
                item.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1
                item.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
                item.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
                item.x -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def pick_up(self, item):
        if self.false_positive_detections:
            fp = np.random.randint(20)
            if fp == 10:  #5% probability of false positive localization
                self.missed_pick()
                print("false detection")
                return

        s_item = self.items[self.belief_space[item]['belief']]
        self.perceived = self.items[s_item.name]
        if not (s_item.item_on_top == None):
            print("won't pick "+s_item.name)
            return
        
        self.box.remove_item(s_item)
        s_item.item_on_bottom=None
        s_item.item_on_top=None
        s_item.item_at_left=None
        s_item.item_at_right=None
        s_item.on_table=False
        s_item.onsomething=False
        s_item.on_clutter_or_table=False
        s_item.being_held = True

        item = s_item.name

        for it in self.objects_list:
            if it.item_on_top == item:
                it.item_on_top = None
            elif it.item_on_bottom == item:
                it.item_on_bottom = None
            elif it.item_at_right == item:
                it.item_at_right = None
            elif it.item_at_left == item:
                it.item_at_left = None
        self.gripper.holding = s_item.name
        self.pick_motion(s_item)


        #put top on botton
    def put_on(self, topitem, bottomitem):
        if topitem == bottomitem:
            return
        top = self.items[topitem]
        bot = self.items[bottomitem]
        if (not bot.onsomething) and (bot.item_on_bottom == None) :
            return
        self.gripper.holding = None
        bot.item_on_top = topitem
        top.inbox = True

        orig_x = self.gripper.x
        orig_y = self.gripper.y
        while math.fabs(top.x - bot.x) > 0:
            if (top.x - bot.x) > 0:
                top.x-=1
                self.gripper.x -=1

            elif (top.x - bot.x) < 0:
                top.x += 1
                self.gripper.x +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        ymargin = top.y + top.height
        while math.fabs(top.y + top.height - bot.y) > 0:
            if (top.y + top.height - bot.y) > 0:
                top.y-=1
                self.gripper.y -=1

            elif (top.y + top.height - bot.y) < 0:
                top.y += 1
                self.gripper.y +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)
        top.item_on_bottom = bot.name
        top.onsomething=True


    def put_in_box(self, topitem, gx, gy):
        top = self.items[topitem]
        if not (self.gripper.holding == topitem):
            return
        self.gripper.holding = None
        top.on_table = True
        top.onsomething=True
        top.inbox = True

        bot = self.items['table']
        orig_x = self.gripper.x
        orig_y = self.gripper.y
        while math.fabs(top.x - gx) > 0:
            if (top.x - gx) > 0:
                top.x-=1
                self.gripper.x -=1

            elif (top.x - gx) < 0:
                top.x += 1
                self.gripper.x +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        ymargin = gy
        while math.fabs(top.y - ymargin) > 0:
            if (top.y - ymargin) > 0:
                top.y-=1
                self.gripper.y -=1

            elif (top.y - ymargin) < 0:
                top.y += 1
                self.gripper.y +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def drop_in_clutter(self, topitem):
        top = self.items[topitem]
        if not (self.gripper.holding == topitem):
            return
        self.gripper.holding = None
        top.item_on_bottom=None
        top.item_on_top=None
        top.item_at_left=None
        top.item_at_right=None
        top.on_table=False
        top.onsomething=False
        top.on_clutter_or_table=True

        for it in self.objects_list:
            if it.item_on_top == topitem:
                it.item_on_top = None
            elif it.item_on_bottom == topitem:
                it.item_on_bottom = None
            elif it.item_at_right == topitem:
                it.item_at_right = None
            elif it.item_at_left == topitem:
                it.item_at_left = None


        
        orig_x = self.gripper.x
        orig_y = self.gripper.y
        while math.fabs(top.x - top.cx) > 0:
            if (top.x - top.cx) > 0:
                top.x-=1
                self.gripper.x -=1

            elif (top.x - top.cx) < 0:
                top.x += 1
                self.gripper.x +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        while math.fabs(top.y - top.cy) > 0:
            if (top.y - top.cy) > 0:
                top.y-=1
                self.gripper.y -=1

            elif (top.y - top.cy) < 0:
                top.y += 1
                self.gripper.y +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def put_left(self, focusitem, staticitem):
        if focusitem==staticitem:
            return
        focus = self.items[focusitem]
        static = self.items[staticitem]
        if not static.on_table:
            return
        self.gripper.holding = None
        static.item_at_left = focusitem
        focus.item_at_right = staticitem
        focus.onsomething=True
        focus.on_table = True
        orig_x = self.gripper.x
        orig_y = self.gripper.y

        margin = (static.x - focus.width)
        while math.fabs(focus.x - (static.x - focus.width)) > 0:
            if (focus.x - (static.x - focus.width)) > 0:
                focus.x -= 1
                self.gripper.x -=1

            elif (focus.x - (static.x - focus.width)) < 0:
                focus.x += 1
                self.gripper.x +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        ymargin = self.table.y - focus.height-10
        while math.fabs(focus.y - ymargin) > 0:
            if (focus.y - ymargin) > 0:
                focus.y -= 1
                self.gripper.y -=1

            elif (focus.y - ymargin) < 0:
                focus.y += 1
                self.gripper.y +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def put_right(self, focusitem, staticitem):
        if focusitem==staticitem:
            return
        focus = self.items[focusitem]
        static = self.items[staticitem]
        if not static.on_table:
            return
        self.gripper.holding = None
        static.item_at_right = focusitem
        focus.item_at_left = staticitem
        focus.onsomething=True
        focus.on_table=True
        orig_x = self.gripper.x
        orig_y = self.gripper.y

        xmargin = (static.x + static.width)
        while math.fabs(focus.x - (static.x + static.width)) > 0:
            if (focus.x - (static.x + static.width)) > 0:
                focus.x -= 1
                self.gripper.x -=1

            elif (focus.x - (static.x + static.width)) < 0:
                focus.x += 1
                self.gripper.x +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        ymargin = (self.table.y - focus.height-10)
        while math.fabs(focus.y - ymargin) > 0:
            if (focus.y - ymargin) > 0:
                focus.y -= 1
                self.gripper.y -=1

            elif (focus.y - ymargin) < 0:
                focus.y += 1
                self.gripper.y +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1
            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def missed_pick(self):
        orig_x = self.gripper.x
        orig_y = self.gripper.y 
        while math.fabs(150 - (self.gripper.x+26)) > 0:
            if 150 - (self.gripper.x+26) > 0:
                self.gripper.x += 1
            elif 150 - (self.gripper.x+26) < 0:
                self.gripper.x -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)
        # print('done moving left')
        self.current_action='FALSE POSITIVE DETECTION!'
        time.sleep(2)

        while math.fabs(400 - (self.gripper.y+90)) > 0:
            if 400 - (self.gripper.y+90) > 0:
                self.gripper.y += 1
            elif 400 - (self.gripper.y+90) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        # print('done moving right')
        # time.sleep(2)
        # print('moving back')
        while math.fabs(orig_y - self.gripper.y) > 0:
            if (orig_y - self.gripper.y) > 0:
                self.gripper.y += 1
            elif (orig_y - self.gripper.y) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


        while math.fabs(orig_x - self.gripper.x) > 0:
            if (orig_x - self.gripper.x) > 0:
                self.gripper.x += 1
            elif (orig_y - self.gripper.x) < 0:
                self.gripper.x -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)


    def put_in_box_test(self, item_list):
        box = Box(3)
        for it in item_list:
            item = self.items[it]
            x,y = box.add_item(item)
            self.pick_up(item.name)
            self.put_in_box(item.name,x,y)
            time.sleep(0.5)


    def test_box(self):
        box = Box(5)
        x,y = box.add_item(self.banana)
        self.pick_up('banana')
        self.put_in_box('banana', x, y)
        x2,y2 = box.add_item(self.cereal)
        self.pick_up('cereal')
        self.put_in_box('cereal', x2, y2)
        x3,y3 = box.add_item(self.lysol)
        self.pick_up('lysol')
        self.put_in_box('lysol', x3, y3)
        # self.pick_up('banana')
        # box.remove_item(self.banana)
        # self.drop_in_clutter('banana')
        x4,y4 = box.add_item(self.ambrosia)
        self.pick_up('ambrosia')
        self.put_in_box('ambrosia', x4, y4)
        x5,y5 = box.add_item(self.milk)
        self.pick_up('milk')
        self.put_in_box('milk', x5, y5)
        x5,y5 = box.add_item(self.pepsi)
        self.pick_up('pepsi')
        self.put_in_box('pepsi', x5, y5)
        x5,y5 = box.add_item(self.nutella)
        self.pick_up('nutella')
        self.put_in_box('nutella', x5, y5)
        x5,y5 = box.add_item(self.oreo)
        self.pick_up('oreo')
        self.put_in_box('oreo', x5, y5)
        x5,y5 = box.add_item(self.lipton)
        self.pick_up('lipton')
        self.put_in_box('lipton', x5, y5)


    def select_perceived_objects_and_classify_weights(self):
        #1
        inbox = []
        topfree = []
        
        for item in self.objects_list:
            if item.on_table:
                inbox.append(item.name)

            else:
                if item.item_on_top == None:
                    topfree.append(item.name)
        #2
        heavylist = []
        mediumlist = []
        for itname in inbox:
            if self.items[itname].mass == 'heavy':
                heavylist.append(itname)

            elif self.items[itname].mass == 'medium':
                mediumlist.append(itname)

        for itname in topfree:
            if self.items[itname].mass == 'heavy':
                heavylist.append(itname)

            elif self.items[itname].mass == 'medium':
                mediumlist.append(itname)


        return (inbox, topfree, mediumlist, heavylist)

    
    def create_pddl_problem(self, inbox, topfree, mediumlist, heavylist):
        itlist = heavylist+mediumlist
        alias = {}
        hc = 0
        for item in heavylist:
            alias[item] = 'h'+str(hc)
            hc+=1

        mc = 0
        for item in mediumlist:
            alias[item] = 'm'+str(mc)
            mc+=1

        init = "(:init (handempty) "
        for item in inbox:
            init += "(inbox "+alias[item]+") "
            it = self.items[item].item_on_top
            if it != None:
                init+= "(on "+alias[it]+" "+alias[item]+") "
            else:
                init += "(topfree "+alias[item]+") "


        for item in topfree:
            init += "(topfree "+alias[item]+") "
            init += "(inclutter "+alias[item]+") "

        if self.box.num_items >= 3:
            init += "(boxfull)"

        init +=  ")\n"    

        goal = "(:goal (and "
        for h in heavylist:
            goal += "(inbox "+alias[h]+") "
            
        mlen=len(mediumlist)
        hlen=len(heavylist)
        stop = hlen
        if mlen > 1:
            goal += " (and "
        if hlen > 2:
            for m in mediumlist[:stop]:
                goal += "(or "
                for h in heavylist:
                    goal += "(on "+alias[m]+" "+alias[h]+") "
                goal +=") "
            # goal+=")"
            for m in mediumlist[stop:]:
                goal+="(or "
                for mm in mediumlist[:stop]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +="))))"

        elif hlen == 1:
            for m in mediumlist[:stop+1]:
                h = heavylist[0]
                goal += "(inbox "+alias[m]+") "
            for m in mediumlist[stop+1:]:
                goal+="(or "
                for mm in heavylist+mediumlist[:stop+1]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +="))))"

        elif hlen == 2:
            for m in mediumlist[:stop-1]:
                goal += "(inbox "+alias[m]+") "
            for m in mediumlist[stop-1:]:
                goal+="(or "
                for mm in heavylist+mediumlist[:stop-1]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +="))))"

        else:
            for m in mediumlist[:3]:
                goal += "(inbox "+alias[m]+") "
            for m in mediumlist[3:]:
                goal+="(or "
                for mm in mediumlist[:3]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +="))))"

        if mlen < 2:
            goal = goal[:-1]



        definition = "(define (problem PACKED-GROCERY) \n(:domain GROCERY) \n (:objects "
        for al in alias.values():
            definition += al+" "
        definition += "- item)\n"

        problem = definition + init + goal

        f = open("newprob.pddl","w")
        f.write(problem)
        f.close()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        prob_path = dir_path+"/"+"newprob.pddl"
        
        swapped_alias  = dict([(value, key) for key, value in alias.items()]) 
        return prob_path, swapped_alias


    def perform_optimistic_belief_grocery_packing(self):
        empty_clutter = self.update_items_left()

        while not empty_clutter:
            inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
            problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
            self.plan_and_run_belief_space_planning(self.domain_path, 
                                                        problem_path, alias)
            empty_clutter = self.update_items_left()

    def perform_optimistic(self):
        start = time.time()
        self.perform_optimistic_belief_grocery_packing()
        end = time.time()
        total = end-start
        print('PLANNING TIME FOR OPTIMISTIC: '+str(self.planning_time))
        print('EXECUTION TIME FOR OPTIMISTIC: '+str(total - self.planning_time))


    def perform_declutter_belief_grocery_packing(self):
        start = time.time()
        self.perform_declutter()
        self.perform_optimistic_belief_grocery_packing()
        end = time.time()
        total = end - start
        print('PLANNING TIME FOR DECLUTTER: '+str(self.planning_time))
        print('EXECUTION TIME FOR DECLUTTER: '+str(total - self.planning_time))


    def execute_plan(self, plan, alias):
        if plan is None or len(plan) == 0:
            print('NO  VALID PLAN FOUND')
            return

        for action in plan:
            self.redrawGameWindow()
            print('Performing action: '+str(action))
            self.current_action = "Action: "+str(action)
            self.belief_execute_action(action, alias)


    def plan_and_run_belief_space_planning(self, domain_path, problem_path, alias):
        f = Fast_Downward()
        start = time.time()
        plan = f.plan(domain_path, problem_path)
        self.planning_time += time.time()-start
        self.execute_plan(plan, alias)

    def belief_execute_action(self, action, alias):
        if action[0] == 'pick-from-clutter':
            self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'pick-from-box':
            self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'pick-from':
            self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'put-in-box':
            x,y = self.box.add_item(self.items[alias[action[1]]])
            self.put_in_box(alias[action[1]],x,y)

        elif action[0] == 'put-in-clutter':
            self.drop_in_clutter(alias[action[1]])

        elif action[0] == 'put-on':
            self.put_on(alias[action[1]], alias[action[2]])


    def declutter_belief_execute_action(self, action):
        if action[0] == 'pick-from-clutter':
            self.pick_up(action[1])
            self.box.remove_item(self.items[action[1]])

        elif action[0] == 'pick-from-box':
            self.pick_up(action[1])
            self.box.remove_item(self.items[action[1]])

        elif action[0] == 'pick-from':
            self.pick_up(action[1])
            self.box.remove_item(self.items[action[1]])

        elif action[0] == 'put-in-box':
            x,y = self.box.add_item(self.items[action[1]])
            self.put_in_box(action[1],x,y)

        elif action[0] == 'put-in-clutter':
            self.drop_in_clutter(action[1])

        elif action[0] == 'put-on':
            self.put_on(action[1], action[2])


    def get_num_declutter_actions(self):
        num_surface = 0
        surface_items=[]
        for item in self.objects_list:
            if not item.inbox:
                name = item.name 
                if item.item_on_top == None:
                    for it in self.objects_list:
                        if it.item_on_top == name:
                            num_surface+=1
                            surface_items.append(name)
                            break
        return 2*num_surface, surface_items


    def declutter_surface_items(self, itemslist):
        for name in itemslist:
            self.pick_up(name)
            self.drop_in_clutter(name)


    def perform_dynamic_grocery_packing(self):
        st = time.time()
        inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
        problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
                                            mediumlist, heavylist)
        self.plan_and_run_belief_space_planning(self.domain_path, 
                                                        problem_path, alias)

        empty_clutter = self.update_items_left()

        while not empty_clutter:
            inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
            problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
            f = Fast_Downward()
            start = time.time()
            plan = f.plan(self.domain_path, problem_path)
            self.planning_time += time.time() - start

            N_o = len(plan)
            N_d, surface_items = self.get_num_declutter_actions()

            if (N_o < N_d) or N_d<6:
                print('EXECUTING OPTIMISTIC')
                self.execute_plan(plan, alias)

            else:
                print('PERFORMING SURFACE DECLUTTER: '+str(N_d)+'since opt is: '+str(N_o))
                self.declutter_surface_items(surface_items)

            
            empty_clutter = self.update_items_left()
        end = time.time()
        total = end-st
        print('PLANNING TIME FOR DYNAMIC: '+str(self.planning_time))
        print('EXECUTION TIME FOR DYNAMIC: '+str(total-self.planning_time))



































if __name__ == '__main__':
    # g = environment(uncertain="low", 
    #                     declutter="optimistic", 
    #                     order=0)
    # g.populate_belief_space()
    # for key in g.belief_space:
    #     print(key+" "+g.belief_space[key].name)
    args = sys.argv
    if len(args) != 3:
        print("Arguments should be level_of_certainty, clutter_strategy and init_order_num")
    else:        
        uncertainty = args[1]
        clutter_strategy = False if args[2]=="optimistic" else True
        order = np.random.randint(4)#int(args[3])
        g = environment(uncertain=uncertainty, 
                        declutter=clutter_strategy, 
                        order=4)
        # g.test_box()
        # g.perform_declutter_belief_grocery_packing()
        # g.perform_optimistic()
        g.perform_dynamic_grocery_packing()
        # g.run()
        # self, inbox, topfree, mediumlist, heavylist
        # print(g.create_pddl_problem(['pepsi','coke'], ['lipton','bleach','nutella'],
        #                          ['pepsi','nutella','bleach','lipton','coke'],[]))
        # g.run_simulation(g.domain_path, g.problem_path)
        # g.clutter_optimistic_planning()
        # g.declutter_before_clutter_planning()
        # while True:
        #     g.redrawGameWindow()









