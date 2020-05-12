import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame
import time
import math
from fd import Fast_Downward

pygame.init()
pygame.display.set_caption("Grocery Packing")


class Grocery_item:
    def __init__(self, x, y, image_path,image_width,image_height):
        self.x = x
        self.y = y
        self.width = image_width
        self.image_height = image_height
        self.body = pygame.image.load(image_path)

    def move_to(self, x, y):
        self.x = x
        self.y = y



class environment:
    def __init__(self, bool_certain):
        self.table = Grocery_item(150,300,'assets/table.jpg',419,144)
        self.pepsi = Grocery_item(10, 400,'assets/pepsi.jpg',26,49)
        self.nutella = Grocery_item(15, 400,'assets/nutella.jpg',26,37)
        self.coke = Grocery_item(20, 400, 'assets/coke.jpg',28,52)
        self.lipton = Grocery_item(25, 400, 'assets/lipton.jpg',58,28)
        self.bleach = Grocery_item(13, 400, 'assets/bleach.jpg',26,64)
        self.gripper = Grocery_item(330, 0,'assets/gripper.png',75,75)

        self.certainty = bool_certain
        self.items = {"pepsi":self.pepsi, "nutella":self.nutella,
                      "coke": self.coke, "lipton": self.lipton,
                      "bleach": self.bleach, "table":self.table}
        self.clock = pygame.time.Clock()
        self.win = pygame.display.set_mode((700,480))
        self.rate = 80


    def sample_object(self,object_name):
        if self.certainty:
            return self.items[object_name]

        objects = [self.pepsi, self.nutella,self.coke,self.lipton,
                    self.bleach]
        item_probabilities = 
            { "pepsi":[0.4, 0.1,0.3,0.1,0.1],
              "nutella":[0.2,0.4,0.2,0.1,0.1],
              "coke":[0.3,0.1,0.4,0.1,0.1],
              "lipton":[0.1,0.1,0.1,0.6,0.1],
              "bleach":[0.1,0.1,0.1,0.1,0.6]

            }

        choice = np.random.choice(objects, size=1, 
                p=item_probabilities[object_name])

        return choice


    def redrawGameWindow(self):
        self.win.fill((255,255,255))
        self.win.blit(self.table.body,(self.table.x, self.table.y))
        self.win.blit(self.pepsi.body,(self.pepsi.x, self.pepsi.y))
        self.win.blit(self.nutella.body,(self.nutella.x, self.nutella.y))
        self.win.blit(self.coke.body,(self.coke.x, self.coke.y))
        self.win.blit(self.lipton.body,(self.lipton.x, self.lipton.y))
        self.win.blit(self.bleach.body,(self.bleach.x, self.bleach.y))
        self.win.blit(self.gripper.body,(self.gripper.x, self.gripper.y))
        
        pygame.display.update()

    def execute_action(self, action):
        if action[0] == 'pick-up':
            self.pick_up(self.action[1])
        elif action[0] == 'put-on-table':
            self.put_on_table(action[1])
        elif action[0] == 'put-on':
            self.put_on(action[1], action[2])
        elif action[0] == 'put-left':
            self.put_left(action[1],action[2])
        elif action[0] == 'put-right':
            self.put_right(action[1], action[2])


    def inspect_scene(self, progress):
        result = True
        for action in progress:
            if action[0] == 'put-on-table':
                result = result and self.check_on_table(action[1])
            elif action[0] == 'put-on':
                result = result and self.check_on(action[1],action[2])
            elif action[0] == 'put-left':
                result = result and self.check_left(action[1],action[2])
            elif action[0] == 'put-right':
                result = result and self.check_right(action[1],action[2])
        return result


    def check_on_table(self, item_name):
        item = self.items[item_name]
        if item.y == 260:
            return True
        else:
            return False


    def check_left(self, left_item_name, right_item_name):
        left_item = self.items[left_item_name]
        right_item = self.items[right_item_name]

        if (right_item.x - left_item.x) == 50:
            return True
        else:
            return False


    def check_right(self, left_item_name, right_item_name):
        left_item = self.items[left_item_name]
        right_item = self.items[right_item_name]

        if (left_item.x - right_item.x) == 50:
            return True
        else:
            return False


    def check_on(self, top_item_name, bot_item_name):
        top_item = self.items[top_item_name]
        bot_item = self.items[bot_item_name]

        if (bot_item.y - top_item.y) == 64:
            return True
        else:
            return False


    def run_simulation(self):
        action_progress=[]
        f = Fast_Downward()
        plan = f.plan('/home/developer/uncertainty/pddl/dom.pddl', '/home/developer/uncertainty/pddl/prob.pddl')
        if plan is None:
            print('No valid plan found')
        else:
            raw_input('Plan computed. Execute plan?')
            for action in plan:
                self.redrawGameWindow()               
                print('Performing action: '+str(action))
                self.execute_action(action)
                action_progress.append(action)
                if not self.inspect_scene(action_progress):
                    #replan
                time.sleep(1)
        pygame.quit()


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
        self.pick_motion(self.sample_object(item))


    def put_on(self, topitem, bottomitem):
        top = self.sample_object(topitem)
        bot = self.sample_object(bottomitem)
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

        while math.fabs(top.y+64 - bot.y) > 0:
            if (top.y+64 - bot.y) > 0:
                top.y-=1
                self.gripper.y -=1

            elif (top.y+64 - bot.y) < 0:
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


    def put_on_table(self, topitem):
        top = self.sample_object(topitem)
        bot = self.items['table']
        orig_x = self.gripper.x
        orig_y = self.gripper.y
        while math.fabs(top.x - 330) > 0:
            if (top.x - 330) > 0:
                top.x-=1
                self.gripper.x -=1

            elif (top.x - 330) < 0:
                top.x += 1
                self.gripper.x +=1
            
            self.redrawGameWindow()
            self.clock.tick(self.rate)

        while math.fabs(top.y - 260) > 0:
            if (top.y - 260) > 0:
                top.y-=1
                self.gripper.y -=1

            elif (top.y - 260) < 0:
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
        focus = self.sample_object(focusitem)
        static = self.sample_object(staticitem)
        orig_x = self.gripper.x
        orig_y = self.gripper.y

        while math.fabs(focus.x - (static.x-50)) > 0:
            if (focus.x - (static.x-50)) > 0:
                focus.x -= 1
                self.gripper.x -=1

            elif (focus.x - (static.x-50)) < 0:
                focus.x += 1
                self.gripper.x +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        while math.fabs(focus.y - static.y) > 0:
            if (focus.y - static.y) > 0:
                focus.y -= 1
                self.gripper.y -=1

            elif (focus.y - static.y) < 0:
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
        focus = self.sample_object(focusitem)
        static = self.sample_object(staticitem)
        orig_x = self.gripper.x
        orig_y = self.gripper.y

        while math.fabs(focus.x - (static.x+50)) > 0:
            if (focus.x - (static.x+50)) > 0:
                focus.x -= 1
                self.gripper.x -=1

            elif (focus.x - (static.x+50)) < 0:
                focus.x += 1
                self.gripper.x +=1 

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        while math.fabs(focus.y - static.y) > 0:
            if (focus.y - static.y) > 0:
                focus.y -= 1
                self.gripper.y -=1

            elif (focus.y - static.y) < 0:
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














if __name__ == '__main__':
    g = environment(bool_certain=False)
    g.run_simulation()


































# while run:
#     clock.tick(27)

#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             run = False

#     keys = pygame.key.get_pressed()
    
#     if keys[pygame.K_LEFT]:
#         x -= vel

#     if keys[pygame.K_RIGHT]:
#         x += vel

#     if keys[pygame.K_UP]:
#         y -= vel

#     if keys[pygame.K_DOWN]:
#         y += vel
    
#     redrawGameWindow()
