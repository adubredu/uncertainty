import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame
import time
import math
import numpy as np
import random
import copy
import operator
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
        self.widths = [self.lx, self.lx+47, self.lx+2*47, self.lx+3*47, self.lx+4*47]
        self.occupancy = [0 for i in range(self.cpty)]
        self.items_added = {}
        self.to_resolve = False
        self.num_items = 0
        self.cascade = False

    def add_item(self, item):
        self.items_added[item.name] = self.index%self.cpty
        self.num_items+=1
        xind = 99
        for i in range(self.cpty):
            if self.occupancy[i] == 0:
                xind = i 
                break
        if xind != 99:
            x = self.widths[xind]
            y = self.ly - item.height 
            self.heights[xind] = y
            self.occupancy[xind] = 1
            self.items_added[item.name] = xind
        else:
            x = self.widths[self.index%self.cpty]
            if self.cascade: 
                y = self.heights[self.index%self.cpty]- item.height                 
            else:
                y = self.ly - item.height 

            self.heights[self.index%self.cpty] = y
            self.index +=1
        return x,y

    def remove_item(self, item):
        if item.name in self.items_added:
            index = self.items_added[item.name]
            self.occupancy[index] = 0
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
    def __init__(self, strategy='optimistic', order=1):
        self.window_width = 1000
        self.window_height = 480
        self.num_mc_samples = 10

        self.table = Grocery_item(150,300,'assets/table.jpg',419,144,"table",0,'heavy')
        self.pepsi = Grocery_item(10, 400,'assets/pepsi.jpg',26,49,"pepsi",200,'medium')
        self.nutella = Grocery_item(15, 400,'assets/nutella.jpg',26,37,"nutella",250,'medium')
        self.coke = Grocery_item(20, 400, 'assets/coke.jpg',28,52,"coke",300,'medium')
        self.lipton = Grocery_item(25, 400, 'assets/lipton.jpg',58,28,"lipton",350,'medium')
        self.bleach = Grocery_item(13, 400, 'assets/bleach.jpg',26,64,"bleach",400, 'heavy')

        self.ambrosia = Grocery_item(13, 400, 'assets/ambrosia.jpg',41,58,"ambrosia",430, 'heavy')
        self.banana = Grocery_item(13, 400, 'assets/banana.jpg',32,32,"banana",460, 'medium')
        self.cereal = Grocery_item(13, 400, 'assets/cereal.jpg',30,48,"cereal",490, 'medium')
        self.lysol = Grocery_item(13, 400, 'assets/lysol.jpg',42,74,"lysol",520, 'heavy')
        self.milk = Grocery_item(13, 400, 'assets/milk.jpg',45,66,"milk",550, 'heavy')
        self.oreo = Grocery_item(13, 400, 'assets/oreo.jpg',28,17,"oreo",580, 'medium')
        self.tangerine = Grocery_item(13, 400, 'assets/tangerine.jpg',49,39,"tangerine",610, 'medium')

        self.gripper = Grocery_item(350, 0,'assets/gripper.png',75,75,"gripper",0,'heavy')
        self.logo = Grocery_item(0,0, 'assets/4progress.png',535,78,"logo",0,'heavy')
        self.perceived = None
        self.false_positive_detections = False
        self.box = Box(5)
        self.planning_time = 0
        self.should_declutter = False

        self.uncertainty = "low" 
        # self.declutter = declutter
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
                "pepsi":[0.12, 0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08],
                "nutella":[0.08,0.12,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08],
                "coke":[0.08,0.08,0.12,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08],
                "lipton":[0.08,0.08,0.08,0.12,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08],
                "bleach":[0.08,0.08,0.08,0.08,0.12,0.08,0.08,0.08,0.08,0.08,0.08,0.08],
                "ambrosia":[0.08,0.08,0.08,0.08,0.08,0.12,0.08,0.08,0.08,0.08,0.08,0.08],
                "banana":[0.08,0.08,0.08,0.08,0.08,0.08,0.12,0.08,0.08,0.08,0.08,0.08],
                "cereal":[0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.12,0.08,0.08,0.08,0.08],
                "lysol":[0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.12,0.08,0.08,0.08],
                "milk":[0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.12,0.08,0.08],
                "oreo":[0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.12,0.08],
                "tangerine":[0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.12]

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
        level = "Easy"
        if order == 1:
            level = "Easy"
        elif order == 2:
            level = "Medium"
        elif order == 3:
            level = "Hard"
        else:
            level = "Brutal"
        self.clock = pygame.time.Clock()
        self.current_action = "Action: "
        self.difficulty_level = "Difficulty Level: "+level
        self.strategy = "Strategy: "+strategy
        
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

    def high_uncertainty_sample(self, object_name):
        choice = np.random.choice(self.objects_list, size=1, 
                p=self.high_matrix[object_name])
        name = choice[0].name
        return name


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

        if choice == 1:
            self.oreo.on_clutter_or_table = True
            self.tangerine.on_clutter_or_table = True
            self.banana.on_clutter_or_table = True
            self.coke.on_clutter_or_table = True
            self.cereal.on_clutter_or_table = True
            self.pepsi.on_clutter_or_table = True
            self.lipton.on_clutter_or_table = True
            self.nutella.on_clutter_or_table = True
            self.oreo.item_on_top = "milk"
            self.tangerine.item_on_top = "lysol"
            self.banana.item_on_top = "bleach"
            self.coke.item_on_top = "ambrosia"


        elif choice == 2:
            self.coke.on_clutter_or_table = True 
            self.pepsi.on_clutter_or_table = True
            self.lipton.on_clutter_or_table = True
            self.cereal.on_clutter_or_table = True
            self.ambrosia.on_clutter_or_table = True
            self.nutella.on_clutter_or_table = True
            self.coke.item_on_top = "milk"
            self.milk.item_on_top = "oreo"
            self.pepsi.item_on_top = "lysol"
            self.lysol.item_on_top = "tangerine"
            self.lipton.item_on_top = "bleach"
            self.bleach.item_on_top = "banana"

        elif choice == 3:
            self.milk.on_clutter_or_table = True
            self.lysol.on_clutter_or_table = True
            self.bleach.on_clutter_or_table = True
            self.ambrosia.on_clutter_or_table = True
            self.milk.item_on_top = "cereal"
            self.cereal.item_on_top = "oreo"
            self.lysol.item_on_top = "nutella"
            self.nutella.item_on_top = "tangerine"
            self.bleach.item_on_top = "pepsi"
            self.pepsi.item_on_top = "banana"
            self.ambrosia.item_on_top = "lipton"
            self.lipton.item_on_top = "coke"

        elif choice == 4:
            self.milk.on_clutter_or_table = True
            self.lysol.on_clutter_or_table = True
            self.milk.item_on_top = "bleach"
            self.lysol.item_on_top = "ambrosia"
            self.bleach.item_on_top = "pepsi"
            self.ambrosia.item_on_top = "lipton"
            self.pepsi.item_on_top = "cereal"
            self.lipton.item_on_top = "nutella"
            self.cereal.item_on_top = "banana"
            self.nutella.item_on_top = "coke"
            self.banana.item_on_top = "oreo"
            self.coke.item_on_top = "tangerine"

        self.draw_init_clutter(choice)


    def draw_init_clutter(self, choice):
        if choice ==1:
            self.oreo.x = 10
            self.tangerine.x = 10 + self.oreo.width 
            self.banana.x = self.tangerine.x + self.tangerine.width 
            self.coke.x = self.banana.x + self.banana.width 
            self.cereal.x = self.coke.x + self.coke.width 
            self.pepsi.x = self.cereal.x + self.cereal.width 
            self.lipton.x = self.pepsi.x + self.pepsi.width 
            self.nutella.x = self.lipton.x + self.lipton.width 
            self.milk.x = self.oreo.x
            self.lysol.x = self.tangerine.x 
            self.bleach.x = self.banana.x 
            self.ambrosia.x = self.coke.x

            self.oreo.y = self.window_height - self.oreo.height  
            self.tangerine.y = self.window_height - self.tangerine.height
            self.banana.y = self.window_height - self.banana.height
            self.coke.y = self.window_height - self.coke.height
            self.cereal.y = self.window_height - self.cereal.height 
            self.pepsi.y = self.window_height - self.pepsi.height
            self.lipton.y = self.window_height - self.lipton.height 
            self.nutella.y = self.window_height - self.nutella.height 
            self.milk.y = self.oreo.y - self.milk.height 
            self.lysol.y = self.tangerine.y - self.lysol.height 
            self.bleach.y = self.banana.y - self.bleach.height 
            self.ambrosia.y = self.coke.y - self.ambrosia.height

        elif choice == 2:
            self.coke.x = 10
            self.pepsi.x = 10 + self.coke.width 
            self.lipton.x = self.pepsi.x + self.pepsi.width 
            self.cereal.x = self.lipton.x + self.lipton.width
            self.ambrosia.x = self.cereal.x + self.cereal.width 
            self.nutella.x = self.ambrosia.x + self.ambrosia.width 
            self.milk.x = self.coke.x 
            self.oreo.x = self.coke.x 
            self.lysol.x = self.pepsi.x 
            self.tangerine.x = self.pepsi.x 
            self.bleach.x = self.lipton.x
            self.banana.x = self.lipton.x 

            self.coke.y = self.window_height - self.coke.height 
            self.pepsi.y = self.window_height - self.pepsi.height 
            self.lipton.y = self.window_height - self.lipton.height 
            self.cereal.y = self.window_height - self.cereal.height 
            self.ambrosia.y = self.window_height - self.ambrosia.height 
            self.nutella.y = self.window_height - self.nutella.height 
            self.milk.y = self.coke.y - self.milk.height 
            self.oreo.y = self.milk.y - self.oreo.height 
            self.lysol.y = self.pepsi.y - self.lysol.height
            self.tangerine.y = self.lysol.y - self.tangerine.height 
            self.bleach.y = self.lipton.y - self.bleach.height 
            self.banana.y = self.bleach.y - self.banana.height

        elif choice == 3:
            self.milk.x = 10
            self.lysol.x = 10 + self.milk.width
            self.bleach.x = self.lysol.x + self.lysol.width 
            self.ambrosia.x = self.bleach.x + self.bleach.width
            self.cereal.x = self.milk.x 
            self.oreo.x = self.milk.x 
            self.nutella.x = self.lysol.x 
            self.tangerine.x = self.lysol.x 
            self.pepsi.x = self.bleach.x 
            self.banana.x = self.bleach.x 
            self.lipton.x = self.ambrosia.x 
            self.coke.x =  self.ambrosia.x 

            self.milk.y = self.window_height - self.milk.height 
            self.lysol.y = self.window_height - self.lysol.height
            self.bleach.y = self.window_height -self.bleach.height
            self.ambrosia.y = self.window_height -self.ambrosia.height 
            self.cereal.y = self.milk.y - self.cereal.height 
            self.oreo.y = self.cereal.y - self.oreo.height 
            self.nutella.y = self.lysol.y - self.nutella.height 
            self.tangerine.y = self.nutella.y - self.tangerine.height
            self.pepsi.y = self.bleach.y - self.pepsi.height 
            self.banana.y = self.pepsi.y - self.banana.height 
            self.lipton.y = self.ambrosia.y - self.lipton.height
            self.coke.y = self.lipton.y - self.coke.height 

        elif choice == 4:
            self.milk.x = 10
            self.lysol.x = 10 + self.milk.width
            self.bleach.x = self.milk.x 
            self.pepsi.x = self.milk.x 
            self.cereal.x = self.milk.x 
            self.banana.x = self.milk.x 
            self.oreo.x = self.milk.x 
            self.ambrosia.x = self.lysol.x 
            self.lipton.x = self.lysol.x 
            self.nutella.x = self.lysol.x 
            self.coke.x = self.lysol.x 
            self.tangerine.x = self.lysol.x 

            self.milk.y = self.window_height - self.milk.height 
            self.lysol.y = self.window_height - self.lysol.height 
            self.bleach.y = self.milk.y - self.bleach.height 
            self.pepsi.y = self.bleach.y - self.pepsi.height 
            self.cereal.y = self.pepsi.y - self.cereal.height
            self.banana.y = self.cereal.y - self.banana.height 
            self.oreo.y = self.banana.y - self.oreo.height 
            self.ambrosia.y = self.lysol.y - self.ambrosia.height
            self.lipton.y = self.ambrosia.y - self.lipton.height 
            self.nutella.y = self.lipton.y - self.nutella.height 
            self.coke.y = self.nutella.y - self.coke.height 
            self.tangerine.y = self.coke.y - self.tangerine.height 


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
        self.display_text(self.difficulty_level, 40,200,14)
        self.display_text(self.strategy, 60,200,14)
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
        if self.should_declutter:
            orig_x = 350; orig_y = 300;
        else:
            orig_x = 350; orig_y = 0;
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
                return False

        s_item = self.items[self.belief_space[item]['belief']]
        self.perceived = self.items[s_item.name]
        if not (s_item.item_on_top == None):
            print("won't pick "+s_item.name)
            return False
        
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
        return True


    

        #put top on botton
    def put_on(self, topitem, bottomitem):
        if topitem == bottomitem:
            return False
        top = self.items[topitem]
        bot = self.items[bottomitem]
        if (not bot.onsomething) and (bot.item_on_bottom == None) :
            return False
        self.gripper.holding = None
        bot.item_on_top = topitem
        top.inbox = True
        bot.inbox = True

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
        return True


    def put_in_box(self, topitem, gx, gy):
        top = self.items[topitem]
        if not (self.gripper.holding == topitem):
            return False
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
        return True


    def drop_in_clutter(self, topitem):
        top = self.items[topitem]
        if not (self.gripper.holding == topitem):
            return False
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
        return True


    def put_to_the_left(self, focusitem):
        self.items[focusitem].cx = 200
        self.items[focusitem].cy = self.window_height - self.items[focusitem].height
        self.pick_up(focusitem)
        self.drop_in_clutter(focusitem)


    def put_to_the_right(self, focusitem):
        self.items[focusitem].cx = 500
        self.items[focusitem].cy = self.window_height- self.items[focusitem].height
        self.pick_up(focusitem)
        self.drop_in_clutter(focusitem)


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
        self.pick_up('banana')
        box.remove_item(self.banana)
        self.drop_in_clutter('banana')
        self.pick_up('lysol')
        box.remove_item(self.lysol)
        self.drop_in_clutter('lysol')
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
            if item.on_table or item.inbox:
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


    def get_objects_in_order(self):
        items_in_order = []
        all_items = [x.name for x in self.objects_list]

        for item in self.objects_list:
            if item.item_on_top == None:
                items_in_order.append(item.name)
                all_items.remove(item.name)       

        while len(all_items) != 0:
            to_add=[]
            for item in all_items:
                if self.items[item].item_on_top in items_in_order:
                    to_add.append(item)

            items_in_order+=to_add
            for it in to_add:
                all_items.remove(it)

        return items_in_order



    def create_sbp_problem(self, inbox, topfree, mediumlist, heavylist):
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

        #generating scene hypotheses and choosing the hypothesis with
        #highest weight
        all_items = [x.name for x in self.objects_list]
        possibly_occluded_items = [x for x in all_items if (x not in topfree) and (x not in inbox)]
        N_o = len(possibly_occluded_items)
        if N_o > 0:
            sigma = 2
            scene_hypotheses = []
            scene_actual_sampled = []
            num_hypotheses = 5
            hyp_scores = [0 for i in range(num_hypotheses)]

            for i in range(num_hypotheses):
                N_o_s = 0
                while N_o_s == 0:
                    N_o_s = np.abs(np.random.randint(low=N_o-sigma, high=N_o+1))
                prob_occluded_items = np.random.choice(possibly_occluded_items, 
                                                    size=N_o_s, replace=False)
                perceived_occluded_objects = [self.high_uncertainty_sample(object_name) \
                                         for object_name in prob_occluded_items]
                scene_hypotheses.append(perceived_occluded_objects)
                scene_actual_sampled.append(prob_occluded_items)

            #score hypotheses and chose the one with the highest score
            for i in range(num_hypotheses):
                for j in range(len(scene_hypotheses[i])):
                    if scene_hypotheses[i][j] == scene_actual_sampled[i][j]:
                        hyp_scores[i]+=0.12
                    else:
                        hyp_scores[i]+=0.08

            arg_scene = np.argmax(hyp_scores)
            occluded_scene = scene_hypotheses[arg_scene]
            print('\nHYPOTHESIS: '),
            print(occluded_scene)
            print(" ")

            #generate ids for occluded objects and assign them predicates
            for obj in occluded_scene:
                if not obj in alias:
                    if self.items[obj].mass == 'heavy':
                        alias[obj] = 'h'+str(hc)
                        heavylist.append(obj)
                        top = np.random.choice(topfree, size=1)
                        init+= "(on "+alias[top[0]]+" "+alias[obj]+") "
                        init += "(inclutter "+alias[obj]+") "
                        hc+=1
                    else:
                        alias[obj] = 'm'+str(mc)
                        mediumlist.append(obj)
                        top = np.random.choice(topfree, size=1)
                        init+= "(on "+alias[top[0]]+" "+alias[obj]+") "
                        init += "(inclutter "+alias[obj]+") "
                        mc+=1

        init +=  ")\n"

        goal = "(:goal (and "
        for h in heavylist:
            goal += "(inbox "+alias[h]+") "
            
        mlen=len(mediumlist)
        hlen=len(heavylist)
        stop = self.box.cpty - hlen

        if hlen == self.box.cpty and mlen > hlen:

            for m in mediumlist[:hlen]:
                goal += "(or "
                for h in heavylist:
                    goal += "(on "+alias[m]+" "+alias[h]+") "
                goal+=") "

            for m in mediumlist[hlen:]:
                goal += "(or "
                for mm in mediumlist[:hlen]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +=")))"

        else:
            for m in mediumlist[:stop]:
                goal += "(inbox "+alias[m]+") "
            for m in mediumlist[stop:stop+self.box.cpty]:
                goal+="(or "
                for mm in heavylist+mediumlist[:stop]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            for m in mediumlist[stop+self.box.cpty:]:
                goal += "(or "
                for mm in mediumlist[stop:self.box.cpty]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +=")))"

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

        # if self.box.num_items >= 5:
        #     init += "(boxfull)"

        init +=  ")\n"    

        goal = "(:goal (and "
        for h in heavylist:
            goal += "(inbox "+alias[h]+") "
            
        mlen=len(mediumlist)
        hlen=len(heavylist)
        stop = self.box.cpty - hlen

        if hlen == self.box.cpty and mlen > hlen:

            for m in mediumlist[:hlen]:
                goal += "(or "
                for h in heavylist:
                    goal += "(on "+alias[m]+" "+alias[h]+") "
                goal+=") "

            for m in mediumlist[hlen:]:
                goal += "(or "
                for mm in mediumlist[:hlen]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +=")))"

        else:
            for m in mediumlist[:stop]:
                goal += "(inbox "+alias[m]+") "
            for m in mediumlist[stop:stop+self.box.cpty]:
                goal+="(or "
                for mm in heavylist+mediumlist[:stop]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            for m in mediumlist[stop+self.box.cpty:]:
                goal += "(or "
                for mm in mediumlist[stop:self.box.cpty]:
                    goal += "(on "+alias[m]+" "+alias[mm]+") "
                goal+=") "
            goal +=")))"


        # if mlen < 2:
        #     goal = goal[:-1]

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
        self.should_declutter = True
        self.perform_declutter()
        self.should_declutter = False
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


    def monte_carlo_sample(self, object_name):
        mc_counts={}
        items = [x.name for x in self.objects_list]
        for t in items: mc_counts[t] = 0
        for i in range(self.num_mc_samples):
            sample = self.high_uncertainty_sample(object_name)
            mc_counts[sample]+=1
        choice = max(mc_counts.iteritems(), key=operator.itemgetter(1))[0]

        return choice


    def divergent_set_sample(self, possibly_occluded_items):
        num_samples = 10
        N_o = len(possibly_occluded_items)
        sigma = 2
        divergent_samples = []
        sample_scores = [0 for i in range(num_samples)]
        #generate set of samples
        for i in range(num_samples):
            N_o_s = 0
            while N_o_s == 0:
                N_o_s = np.abs(np.random.randint(low=N_o-sigma, high=N_o+1))
            prob_occluded_items = np.random.choice(possibly_occluded_items, 
                                                size=N_o_s, replace=False)
            perceived_occluded_objects = [self.high_uncertainty_sample(object_name) \
                                     for object_name in prob_occluded_items]
            divergent_samples.append(perceived_occluded_objects)

        #generate mc sample
        mc_sample = [self.monte_carlo_sample(object_name) \
                                     for object_name in possibly_occluded_items]

        #score each sample in set
        for i in range(num_samples):
            for obj in divergent_samples[i]:
                if obj in mc_sample:
                    sample_scores[i] += 1

        #get max sample and min sample and choose one with 0.5 probability
        arg_max_samp = np.argmax(sample_scores)
        arg_min_samp = np.argmin(sample_scores)

        max_sample = divergent_samples[arg_max_samp]
        min_sample = divergent_samples[arg_min_samp]

        toss = np.random.randint(2)

        if toss == 1:
            return max_sample
        else:
            return min_sample


    def estimate_clutter_content(self, surface_items, inbox_items,sample_procedure):
        sigma = 2
        N_o = self.items_in_clutter - (len(surface_items)+len(inbox_items))
        N_o_s = np.random.randint(low=N_o-sigma, high=N_o+1)
        N_o_s = np.abs(N_o_s)
        all_items = [x.name for x in self.objects_list]
        possibly_occluded_items = [x for x in all_items if (x not in surface_items) and (x not in inbox_items)]
        if len(possibly_occluded_items) == 0 or N_o_s == 0:
            return 0.0,1.0
        prob_occluded_items = np.random.choice(possibly_occluded_items, size=N_o_s, replace=False)

        #one-time weighted sample. To change sampling strategy, alter 
        #the function: self.high_uncertainty_sample(name)
        if sample_procedure == 'weighted_sample':
            sampled_occluded_items = [self.high_uncertainty_sample(object_name) \
                                     for object_name in prob_occluded_items]
        elif sample_procedure == 'mc_sample':
            sampled_occluded_items = [self.monte_carlo_sample(object_name) \
                                     for object_name in prob_occluded_items]
        elif sample_procedure == 'divergent_set':
            sampled_occluded_items = self.divergent_set_sample(possibly_occluded_items)

        num_heavy = 0; num_light = 0;
        for it in sampled_occluded_items:
            if self.items[it].mass == 'heavy':
                num_heavy +=1
            else:
                num_light += 1
        #finding percentage of heavy and light
        # print("NUM HEAVY: "+str(num_heavy)+" over "+str(N_o_s))
        oh = (float(num_heavy)/float(N_o_s))

        suh = 0
        print('surface items:')
        print(surface_items)
        for it in surface_items:
            if self.items[it].mass == 'heavy':
                suh +=1

        print("sNUM HEAVY: "+str(suh)+" over "+str(len(surface_items)))
        sh = float(suh)/float(len(surface_items))
        return oh, sh


    def declutter_surface_items(self, itemslist):
        self.should_declutter = True
        for name in itemslist:
            self.pick_up(name)
            self.drop_in_clutter(name)
        self.should_declutter = False


    def perform_dynamic_grocery_packing(self,sample_procedure):
        st = time.time()
        # inboxlist, topfreelist, mediumlist, heavylist = \
        #             self.select_perceived_objects_and_classify_weights()
        # problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
        #                                     mediumlist, heavylist)
        # self.plan_and_run_belief_space_planning(self.domain_path, 
                                                        # problem_path, alias)

        empty_clutter = self.update_items_left()

        while not empty_clutter:
            inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
            problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
            
            unoccluded_items = topfreelist
            oh, sh = self.estimate_clutter_content(unoccluded_items,inboxlist,sample_procedure)
            print("probs are "+str(oh)+" "+str(sh))

            if sh > oh:
                print('\nPERFORMING OPT\n')
                
                f = Fast_Downward()
                start = time.time()
                plan = f.plan(self.domain_path, problem_path)
                self.planning_time += time.time() - start
                self.execute_plan(plan, alias)
            else:
                print(('\nPERFORMING DECLUTTER\n'))
                N_d, surface_items = self.get_num_declutter_actions()
                self.declutter_surface_items(surface_items)

            
            empty_clutter = self.update_items_left()
        end = time.time()
        total = end-st
        print('PLANNING TIME FOR DYNAMIC: '+str(self.planning_time))
        print('EXECUTION TIME FOR DYNAMIC: '+str(total-self.planning_time))


    def perform_sbp_grocery_packing(self):
        st = time.time()

        empty_clutter = self.update_items_left()

        while not empty_clutter:
            inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
            problem_path, alias = self.create_sbp_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
            self.run_sbp(self.domain_path, problem_path, alias)

            empty_clutter = self.update_items_left()
        end = time.time()
        total = end-st
        print('PLANNING TIME FOR SBP: '+str(self.planning_time))
        print('EXECUTION TIME FOR SBP: '+str(total-self.planning_time))


    def run_sbp(self, domain_path, problem_path, alias):
        f = Fast_Downward()
        start = time.time()
        plan = f.plan(domain_path, problem_path)
        if len(plan) == 0 or plan == None:
            print('NO PLAN FOUND')
            return

        self.planning_time += time.time() - start
        for action in plan:
            print(action)
            result = self.execute_sbp_action(action, alias)
            if not result:
                self.current_action = "Action: REPLANNING..."  
                print('REPLANNING')
                inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
                new_problem_path, nalias = self.create_sbp_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
                self.run_sbp(self.domain_path, new_problem_path, nalias)
                break
        return


    def execute_sbp_action(self,action, alias):
        self.current_action = "Action: "+str(action)
        success = True
        if action[0] == 'pick-from-clutter':
            success = self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'pick-from-box':
            success = self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'pick-from':
            success = self.pick_up(alias[action[1]])
            self.box.remove_item(self.items[alias[action[1]]])

        elif action[0] == 'put-in-box':
            x,y = self.box.add_item(self.items[alias[action[1]]])
            success = self.put_in_box(alias[action[1]],x,y)

        elif action[0] == 'put-in-clutter':
            success = self.drop_in_clutter(alias[action[1]])

        elif action[0] == 'put-on':
            success = self.put_on(alias[action[1]], alias[action[2]])

        return success


    def perform_bag_sort_grocery_packing(self):
        st = time.time()
        items_in_order = self.get_objects_in_order()
        heavy=[]
        light = []
        self.should_declutter = True
        box = Box(5)
        box.cascade = True 
        for item in items_in_order:
            if self.items[item].mass == "heavy":
                heavy.append(item)
                self.put_to_the_left(item)
            else:
                self.put_to_the_right(item)
                light.append(item)

        self.should_declutter = False
        for item in heavy:
            x,y = box.add_item(self.items[item])
            self.pick_up(item)
            self.put_in_box(item,x,y)

        for item in light:
            x,y = box.add_item(self.items[item])
            self.pick_up(item)
            self.put_in_box(item,x,y)
        duration = time.time() - st
        print("PLANNING TIME FOR BAGSORT: 0")
        print("EXECUTION TIME FOR BAGSORT: "+str(duration))


    def perform_pick_n_roll(self):
        st = time.time()
        items_in_order = self.get_objects_in_order()
        light = []
        box = Box(5)
        box.cascade = True 

        for item in items_in_order:
            if self.items[item].mass == 'heavy':
                x,y = box.add_item(self.items[item])
                self.pick_up(item)
                self.put_in_box(item,x,y)

            else:
                self.should_declutter = True
                light.append(item)
                self.pick_up(item)
                self.put_to_the_right(item)
                self.should_declutter = False

        for item in light:
            x,y = box.add_item(self.items[item])
            self.pick_up(item)
            self.put_in_box(item,x,y)

        duration = time.time() - st
        print("PLANNING TIME FOR PICKNROLL: 0")
        print("EXECUTION TIME FOR PICKNROLL: "+str(duration))


    def perform_conveyor_belt_pack(self):
        items_in_order = self.get_objects_in_order()
        start = time.time()
        for item in items_in_order:
            inboxlist, topfreelist, mediumlist, heavylist = \
                    self.select_perceived_objects_and_classify_weights()
            for t in topfreelist:
                if t != item:
                    try:
                        mediumlist.remove(t)
                    except:
                        pass
            for t in topfreelist:
                if t != item:
                    try:
                        heavylist.remove(t)
                    except:
                        pass
            topfreelist = [item]

            problem_path, alias = self.create_pddl_problem(inboxlist, topfreelist,
                                                mediumlist, heavylist)
            self.plan_and_run_belief_space_planning(self.domain_path, 
                                                        problem_path, alias)
        end = time.time()
        total = end-start
        print('PLANNING TIME FOR CONVEYORBELT: '+str(self.planning_time))
        print('EXECUTION TIME FOR CONVEYORBELT: '+str(total - self.planning_time))


    def run_strategy(self, strategy):
        if strategy == 'conveyor-belt':
            self.perform_conveyor_belt_pack()
        elif strategy == 'pick-n-roll':
            self.perform_pick_n_roll()
        elif strategy == 'bag-sort':
            self.perform_bag_sort_grocery_packing()
        elif strategy == 'sbp':
            self.perform_sbp_grocery_packing()
        elif strategy == 'optimistic':
            self.perform_optimistic()
        elif strategy == 'declutter':
            self.perform_declutter_belief_grocery_packing()
        elif strategy == 'mc-dynamic':
            self.perform_dynamic_grocery_packing('mc_sample')
        elif strategy == 'weighted-dynamic':
            self.perform_dynamic_grocery_packing('weighted_sample')
        elif strategy == 'divergent-dynamic':
            self.perform_dynamic_grocery_packing('divergent_set')













































if __name__ == '__main__':
    # g = environment(uncertain="low", 
    #                     declutter="optimistic", 
    #                     order=0)
    # g.populate_belief_space()
    # for key in g.belief_space:
    #     print(key+" "+g.belief_space[key].name)
    args = sys.argv
    if len(args) != 3:
        print("Arguments should be strategy and difficulty")
    else:        
        strategy = args[1]
        difficulty = args[2]
        g = environment(strategy=strategy, 
                        order=int(difficulty))
        g.run_strategy(strategy)
        # g.test_box()
        # g.perform_bag_sort_grocery_packing()
        # g.perform_pick_n_roll()
        # g.perform_conveyor_belt_pack()
        # g.perform_declutter_belief_grocery_packing()
        # g.perform_optimistic()
        # g.perform_dynamic_grocery_packing()
        # g.perform_sbp_grocery_packing()
        # time.sleep(2)
        # g.run()
        # self, inbox, topfree, mediumlist, heavylist
        # print(g.create_pddl_problem(['pepsi','coke'], ['lipton','bleach','nutella'],
        #                          ['pepsi','nutella','bleach','lipton','coke'],[]))
        # g.run_simulation(g.domain_path, g.problem_path)
        # g.clutter_optimistic_planning()
        # g.declutter_before_clutter_planning()
        # while True:
        #     g.redrawGameWindow()









