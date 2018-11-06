#!/usr/bin/env python
import pygame
from pygame.locals import *
import threading
from threading import Lock
import sys
import math
import os
import glob

mutex = Lock()

Color_line = (0, 0, 0)
Color_line_middle = (180, 180, 180)
Color_line_path = (240, 0, 0)
Color_Red = (255, 0, 0)
Color_BLUE = (0, 0, 255)
Color_GREEN = (0, 255, 0)
Color_PURPLE = (255, 0, 255)
Color_CYAN = (0, 255, 255)


# change from https://stackoverflow.com/questions/40399049/parallel-display-updates-for-animations-in-pygame for explosion
class Explosion(pygame.sprite.Sprite):
    def __init__(self, pos, exp_img1):
        pygame.sprite.Sprite.__init__(self)
        self.state = 0
        self.exp_img = exp_img1
        self.image = self.exp_img[0]
        self.rect = self.image.get_rect(center=pos)

    def update(self):
        self.state += 1
        self.image = self.exp_img[self.state]
        self.rect = self.image.get_rect(center=self.rect.center)
        if self.state > 5:
            self.kill()


sprites = pygame.sprite.Group()


class PLOT:
    def __init__(self, static_obs, xlim, ylim):
        self._running = True
        self.display = None
        self._image_surf = None
        self.screenH = 800
        self.screenW = 600
        self.maxX = xlim
        self.maxY = ylim
        self.xlim = xlim
        self.ylim = ylim
        self.lines = 9
        self.originX = 0
        self.originY = 0
        self.moveX = 0
        self.moveY = 0
        self.cost = 0
        self.future_x = []
        self.future_y = []
        self.goal_location = []
        self.F_goal_location = []
        self.future_heading = []
        self.estimateStart = (0,0,0)
        self.triangleX = (0, -5, 5)
        self.triangleY = (-10, 10, 10)
        self.static_obs = static_obs
        pygame.font.init()
        try:
            self.myfont = pygame.font.Font("r.ttf", 15)
        except:
            self.myfont = pygame.font.SysFont(None, 20)
        
            

    def on_init(self):
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.screenH, self.screenW), HWSURFACE | RESIZABLE)
        self.w, self.h = pygame.display.get_surface().get_size()
        self.scalew = self.w*0.8
        self.scaleh = self.h*0.8
        self.startw = self.w * 0.1
        self.starth = self.h * 0.1
        self._running = True
        self.background = pygame.Surface(self.display.get_size())
        self.background.fill((240, 240, 240))
        self.background = self.background.convert()
        # self._image_surf = pygame.transform.scale(pygame.image.load(
        #     "boat.png"), (int(self.scalew/20), int(self.scaleh/20))).convert_alpha()
        # self.image_obstacle = pygame.transform.scale(pygame.image.load(
        #     "obstacle.png"), (int(self.scalew/20), int(self.scaleh/20))).convert_alpha()
        # self.boatw = int(self.scalew/20)/2.0
        # self.boath = int(self.scaleh/20)
        self.curr_x = 0
        self.curr_y = 0
        self.start_heading = 0
        self.nobs = 0
        self.xobs = []
        self.yobs = []
        self.hobs = []
        self.exp_img = []
        for i in range(1, 8):
            self.exp_img.append(pygame.image.load(
                'explosion/{}.png'.format(i)))
        # self.exp_img = [pygame.image.load(os.path.join('explosion', img)).convert_alpha()
        #           for img in os.listdir('explosion')]

    def on_event(self, event):
        if event.type == QUIT:
            sys.exit(1)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                self.originX -= self.scalew/(self.lines+1)
                self.moveX += self.maxX / (self.lines+1)
            elif event.key == pygame.K_RIGHT:
                self.originX += self.scalew/(self.lines+1)
                self.moveX -= self.maxX / (self.lines+1)
            elif event.key == pygame.K_DOWN:
                self.originY += self.scaleh/(self.lines+1)
                self.moveY += self.maxY / (self.lines+1)
            elif event.key == pygame.K_UP:
                self.originY -= self.scaleh/(self.lines+1)
                self.moveY -= self.maxY / (self.lines+1)
            elif event.key == pygame.K_MINUS:
                d = 1 if self.maxX < 100 and self.maxY < 100 else 10
                p = d*10
                x, y = self.scale_item(self.curr_x, self.curr_y)
                x = int(round(((x - self.startw)/self.scalew)*10, 5))
                y = int(round(((1 - (y - self.starth)/self.scaleh) * 10), 5))
                self.maxX += p
                self.maxY += p
                self.originY = self.scaleh * \
                    ((int(self.curr_y) / d) * d / float(self.maxY)) - \
                    y*self.scaleh/(self.lines+1)
                self.originX = -self.scalew * \
                    ((int(self.curr_x) / d) * d / float(self.maxX)) + \
                    x*self.scalew/(self.lines+1)
                cy = (int(self.curr_y) / d) * d
                cx = (int(self.curr_x) / d) * d
                self.moveY = cy - y * self.maxY / (self.lines+1)
                self.moveX = cx - x * self.maxX / (self.lines+1)
            elif event.key == pygame.K_EQUALS:
                if self.maxX >= 20 and self.maxY >= 20:
                    d = 10 if self.maxX >= 200 and self.maxY >= 200 else 1
                    p = d*10
                    x, y = self.scale_item(self.curr_x, self.curr_y)
                    x = int(round(((x - self.startw)/self.scalew)*10, 5))
                    y = int(round(((1 - (y - self.starth)/self.scaleh) * 10), 5))
                    self.maxX -= p
                    self.maxY -= p
                    self.originY = self.scaleh * \
                        ((int(self.curr_y) / d) * d / float(self.maxY)) - \
                        y*self.scaleh/(self.lines+1)
                    self.originX = -self.scalew * \
                        ((int(self.curr_x) / d) * d / float(self.maxX)) + \
                        x*self.scalew/(self.lines+1)
                    cy = (int(self.curr_y) / d) * d
                    cx = (int(self.curr_x) / d) * d
                    self.moveY = cy - y * self.maxY / (self.lines+1)
                    self.moveX = cx - x * self.maxX / (self.lines+1)
            elif event.key == pygame.K_SPACE:
                d = 10 if self.maxX >= 100 and self.maxY >= 100 else 1
                self.maxY = self.ylim
                self.maxX = self.xlim
                self.originY = self.scaleh * \
                    ((int(self.curr_y) / d) * d / float(self.maxY))
                self.originX = -self.scalew * \
                    ((int(self.curr_x) / d) * d / float(self.maxX))
                self.moveY = (int(self.curr_y) / d) * d
                self.moveX = (int(self.curr_x) / d) * d

    def on_loop(self):
        pass

    def on_render(self):
        self.display.blit(self.background, (0, 0))
        self.draw_line()
        self.draw_path()
        self.draw_obs()
        self.draw_text()
        self.draw_future()
        self.draw_current()
        # screen.fill((0, 0, 0))
        self.checkCollision()
        sprites.draw(self.display)
        sprites.update()
        pygame.display.flip()

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs):
        if self.on_init() == False:
            self._running = False
        # while self._running:
        #     self.update()
        # self.stop()
        self.pathList = [(curr_x, curr_y)]
        self.updateInformation(
            curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs, [], [], [],self.estimateStart,[])
        self.update()

    def checkCollision(self):
        for i in self.static_obs:
            x, y = i
            if x <= self.curr_x < x + 1 and y <= self.curr_y < y + 1:
                sprites.add(Explosion(self.scale_item(
                    self.curr_x, self.curr_y), self.exp_img))
                return
        for i in range(0, self.nobs):
            if 2.25 > self.dist(self.xobs[i], self.curr_x, self.yobs[i], self.curr_y):
                sprites.add(Explosion(self.scale_item(
                    self.curr_x, self.curr_y), self.exp_img))
                return

    def dist(self, x, x1, y, y1):
        return (x - x1)**2 + (y - y1)**2

    def draw_line(self):
        pygame.draw.line(self.display, Color_line, (self.startw,
                                                    self.starth), (self.startw+self.scalew, self.starth))
        pygame.draw.line(self.display, Color_line, (self.startw,
                                                    self.starth), (self.startw, self.starth+self.scaleh))
        pygame.draw.line(self.display, Color_line, (self.startw, self.starth +
                                                    self.scaleh), (self.startw+self.scalew, self.starth+self.scaleh))
        pygame.draw.line(self.display, Color_line, (self.startw+self.scalew,
                                                    self.starth), (self.startw+self.scalew, self.starth+self.scaleh))
        scaleh = self.scaleh/(self.lines+1)
        scalew = self.scalew/(self.lines+1)
        for i in range(1, self.lines+1):
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                0, i * scaleh), self.scale_xy(self.scalew, i * scaleh))
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                i * scalew, 0), self.scale_xy(i*scalew, self.scaleh))
            # self.draw_text(self.scalew,i * scaleh, i * self.maxX/(self.lines + 1) )

    def updateInformation(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs, future_x, future_y, future_heading,estimateStart,goal_location):
            if self.pathList[-1] != (curr_x, curr_y):
                self.pathList.append((curr_x, curr_y))
            self.curr_x = curr_x
            self.curr_y = curr_y
            self.start_heading = start_heading
            self.nobs = nobs
            self.xobs = xobs
            self.yobs = yobs
            self.hobs = hobs
            self.future_heading = future_heading
            self.future_x = future_x
            self.future_y = future_y
            self.estimateStart = estimateStart
            self.goal_location = goal_location

    def draw_future(self):
        size = len(self.future_heading)
        if size > 0:
            if size > 1:
                decay = (255 - 20) / (size - 1)
                for i in range(1, size):
                    c = (255 - decay * i, 0, 255 - decay * i)
                    self.draw_vehicle(
                        self.future_heading[i], c, *self.scale_item(self.future_x[i], self.future_y[i]))
            self.draw_vehicle(
                self.future_heading[0], Color_CYAN, *self.scale_item(self.future_x[0], self.future_y[0]))
        x,y,heading = self.estimateStart
        self.draw_vehicle(
                        heading, Color_GREEN, *self.scale_item(x, y))

    def draw_current(self):
        self.draw_vehicle(self.start_heading, Color_BLUE, *
                          self.scale_item(self.curr_x, self.curr_y))
        # self.display.blit(pygame.transform.rotate(self._image_surf, -1*math.degrees(self.start_heading)),
        #                   (self.scale_item(self.curr_x, self.curr_y)))

    def draw_path(self):
        for index in range(1, len(self.pathList)):
            pygame.draw.line(self.display, Color_line_path, self.scale_path(
                *self.pathList[index-1]), self.scale_path(*self.pathList[index]))

    def draw_obs(self):
        for index in range(self.nobs):
            self.draw_vehicle(
                self.hobs[index], Color_Red, *self.scale_item(self.xobs[index], self.yobs[index]))

        for obs in self.static_obs:
            self.draw_static_obs((0,0,0),*obs)
        
        for i in range(len(self.goal_location)):
            if(self.distSquare(self.curr_x,self.curr_y,*self.goal_location[i]) <= 10):
                self.F_goal_location.append(self.goal_location.pop(i))
                break;
            

        for goal in self.goal_location:
            self.draw_static_obs((0,255,0),*goal)

        for goal in self.F_goal_location:
            self.draw_static_obs((0,255,255),*goal)

    def distSquare(self,x1,y1,x2,y2):
        return ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 )

    def draw_static_obs(self, color,x, y):
        pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
            x+1, y), self.scale_item(x+1, y+1), self.scale_item(x, y+1)))

    def draw_vehicle(self, angle, color, x, y):
        tX = []
        tY = []
        c = math.cos(angle)
        s = math.sin(angle)
        for i in range(3):
            tX.append(self.triangleX[i]*c - self.triangleY[i]*s + x)
            tY.append(self.triangleX[i]*s + self.triangleY[i]*c + y)
        pygame.draw.polygon(self.display, color, ((
            tX[0], tY[0]), (tX[1], tY[1]), (tX[2], tY[2])))

    def draw_text(self):
        scaleh = self.scaleh/(self.lines+1)
        scalew = self.scalew/(self.lines+1)
        for i in range(0, self.lines+2):
            text = str(int(self.moveY+(self.lines - i + 1)
                           * self.maxY/(self.lines + 1)))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy(0, (i) * scaleh)
            x -= textsurface.get_width() + 10
            y -= textsurface.get_height()/2
            self.display.blit(textsurface, (x, y))
            text = str(int(self.moveX+(self.lines - i + 1)
                           * self.maxX/(self.lines + 1)))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy((self.lines - i + 1)*scalew, self.scaleh)
            x -= textsurface.get_width() / 2
            y += 10
            self.display.blit(textsurface, (x, y))

        text = "Current Location:(  {:.2f} , {:.2f}  )".format(
            self.curr_x, self.curr_y)
        textsurface = self.myfont.render(text, True, (0, 0, 0))
        self.display.blit(textsurface, (20, 10))
        text = "Cost:(  {:.2f}  )".format(self.cost)
        self.cost += 0.05
        textsurface1 = self.myfont.render(text, True, (0, 0, 0))
        self.display.blit(textsurface1, (20, 10 + textsurface.get_height()))

    def update(self):
        for event in pygame.event.get():
            self.on_event(event)
        self.on_loop()
        self.on_render()

    def stop(self):
        self.on_cleanup()

    def scale_xy(self, x, y):
        return self.startw + x, self.starth + y

    def scale_item(self, x, y):
        return self.originX + self.startw + self.scalew*(x/float(self.maxX)), self.originY + self.starth + self.scaleh - self.scaleh*(y/float(self.maxY))

    def scale_path(self, x, y):
        return self.originX + self.startw + self.scalew*(x/float(self.maxX)), self.originY + self.starth + self.scaleh - self.scaleh*(y/float(self.maxY))


# if __name__ == "__main__" :
#     theApp = App()
#     theApp.on_execute()
