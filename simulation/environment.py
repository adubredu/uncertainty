import pygame
import time
import math

pygame.init()
pygame.display.set_caption("Grocery Packing")


class Grocery_item:
    def __init__(self, x, y, image_path):
        self.x = x
        self.y = y
        self.body = pygame.image.load(image_path)

    def move_to(self, x, y):
        self.x = x
        self.y = y



class environment:
    def __init__(self):
        self.table = Grocery_item(150,300,'assets/table.jpg')
        self.pepsi = Grocery_item(230, 260,'assets/pepsi.jpg')
        self.nutella = Grocery_item(500, 260,'assets/nutella.jpg')
        self.coke = Grocery_item(180, 260, 'assets/coke.jpg')
        self.lipton = Grocery_item(400, 280, 'assets/lipton.jpg')
        self.bleach = Grocery_item(280, 250, 'assets/bleach.jpg')
        self.gripper = Grocery_item(330, 0,'assets/gripper.png')

        self.items = {"pepsi":self.pepsi, "nutella":self.nutella,
                      "coke": self.coke, "lipton": self.lipton,
                      "bleach": self.bleach}

        self.clock = pygame.time.Clock()
        self.win = pygame.display.set_mode((700,480))
        self.rate = 80

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


    def run_simulation(self):
        for i in range(1):
            self.redrawGameWindow()
            raw_input('Continue?')
            self.pick_up('coke')
            self.put_on('coke','bleach')
            self.pick_up('nutella')
            self.put_on('nutella','coke')
            self.pick_up('pepsi')
            self.put_left('pepsi','bleach')
            self.pick_up('lipton')
            self.put_right('lipton','bleach')
            time.sleep(2)
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
        print('done moving left')

        while math.fabs(item.y - (self.gripper.y+90)) > 0:
            if item.y - (self.gripper.y+90) > 0:
                self.gripper.y += 1
            elif item.y - (self.gripper.y+90) < 0:
                self.gripper.y -= 1

            self.redrawGameWindow()
            self.clock.tick(self.rate)

        print('done moving right')
        # time.sleep(2)
        print('moving back')
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
        self.pick_motion(self.items[item])


    def put_on(self, topitem, bottomitem):
        top = self.items[topitem]
        bot = self.items[bottomitem]
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

        print('moving back')
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
        focus = self.items[focusitem]
        static = self.items[staticitem]
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

        print('moving back')
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
        focus = self.items[focusitem]
        static = self.items[staticitem]
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

        print('moving back')
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
    g = environment()
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
