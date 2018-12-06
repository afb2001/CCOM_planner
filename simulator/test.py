#!/usr/bin/env python
import pygame
from pygame.locals import *
import threading
from threading import Lock
import sys
import math
import os
import glob


obs_radius = 1.5
Color_line = (0, 0, 0)
Color_line_middle = (180, 180, 180)
Color_line_path = (240, 0, 0)
Color_Red = (255, 0, 0)
Color_Red_light = (255, 100, 100)
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
    def __init__(self, static_obs,static_draw, xlim, ylim, factor):
        self.mutex = Lock()
        self._running = True
        self.display = None
        self._image_surf = None
        self.screenH = 600
        self.screenW = 800
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
        self.collision = 0
        self.future_x = []
        self.future_y = []
        self.goal_location = []
        self.F_goal_location = []
        self.future_heading = []
        self.estimateStart = (0, 0, 0)
        self.triangleX = (0, -5, 5)
        self.triangleY = (-10, 10, 10)
        self.static_obs = static_obs
        self.factor = factor
        self.static_draw = static_draw
        self.dynamicOBS =[]
        self.select = None
        self.pDynamicHit = None
        self.pStaticHit = None
        self.dynamicMap = {}
        self.dynamicAction = {}
        self.pause = False
        pygame.font.init()
        try:
            self.myfont = pygame.font.Font("r.ttf", 15)
        except:
            self.myfont = pygame.font.SysFont(None, 20)

    def on_init(self):
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.screenW, self.screenH), HWSURFACE | RESIZABLE)
        self.w, self.h = pygame.display.get_surface().get_size()
        self.minwh = min(self.w,self.h)
        self.scalew = self.minwh*0.8
        self.scaleh = self.minwh*0.8
        self.startw = (self.w - self.scalew)/2
        self.starth = (self.h - self.scaleh)/2
        self._running = True
        self.background = pygame.Surface(self.display.get_size())
        self.background.fill((240, 240, 240))
        self.background = self.background.convert()
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
    def distance(self,p0, p1):
        return (p0[0] - p1[0])**2 + (p0[1] - p1[1])**2

    def angle(self,p0, p1):
        return -math.atan2(p0[0]-p1[0],p0[1]-p1[1])

    def selects(self):
        self.mutex.acquire()
        try:
            for key, value in self.dynamicAction.iteritems():
                index = value[2]
                l = self.dynamicMap[key]
                if index == -1 or len(l) == 1:
                    continue
                if(self.distance(self.dynamicOBS[key],l[index]) < 10):
                    if len(l) > index + 1:
                        index += 1
                    else:
                        index = 0
                    self.dynamicAction[key] = (self.angle(self.dynamicOBS[key],l[index]),3,index)
            s = self.dynamicAction
        finally:
            self.mutex.release()
        return s
    def getPause(self):
        self.mutex.acquire()
        try:
            if self.pause:
                s = "pause"
            else:
                s = "start"
        finally:
            self.mutex.release()
        return s

    def on_event(self, event):
        if event.type == QUIT:
            sys.exit(1)
        elif event.type == pygame.VIDEORESIZE:
            self.originX /= self.scalew/(self.lines+1)
            self.originY /= self.scaleh/(self.lines+1)
            self.w,self.h =  event.size
            self.minwh = min(self.w,self.h)
            self.scalew = self.minwh*0.8
            self.scaleh = self.minwh*0.8
            self.startw = (self.w - self.scalew)/2
            self.starth = (self.h - self.scaleh)/2
            self.display = pygame.display.set_mode(
            (self.w, self.h), HWSURFACE | RESIZABLE)
            self.background = pygame.Surface(self.display.get_size())
            self.background.fill((240, 240, 240))
            self.background = self.background.convert()
            self.originX *= self.scalew/(self.lines+1)
            self.originY *= self.scaleh/(self.lines+1)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            p = pygame.mouse.get_pos()
            if self.select == None:
                
                for i in range(self.nobs):
                    if (self.distance(p,self.dynamicOBS[i]) < 500 ):
                        print i
                        self.mutex.acquire()
                        try:
                            self.dynamicMap[i] = []
                            self.dynamicAction[i] = (self.hobs[i],0,-1)
                            self.select = (i,self.hobs[i],0)
                        finally:
                            self.mutex.release()
                        break;
            else:
                i = self.select[0]
                if self.distance(p,self.dynamicOBS[i]) < 500:
                    self.dynamicMap[i].append(self.dynamicOBS[i])
                    l = self.dynamicMap[i]
                    if len(l) > 1:
                        self.dynamicAction[i] = (self.angle(self.dynamicOBS[i],self.dynamicMap[i][0]),3,0)
                    self.select = None
                else:
                    self.dynamicMap[i].append(p)
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
            elif event.key == pygame.K_RSHIFT or event.key == pygame.K_LSHIFT:
                self.pause = not self.pause
                

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

    def on_execute(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs,vobs):
        if self.on_init() == False:
            self._running = False
        # while self._running:
        #     self.update()
        # self.stop()
        self.pathList = [(curr_x, curr_y)]
     

        self.dynamicOBS.extend([(0,0) for i in range(nobs)])
        self.updateInformation(
            curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs,vobs, [], [], [], self.estimateStart, [])
        self.update()

    def checkCollision(self):
        x,y = self.scale_item(self.curr_x, self.curr_y)
        if  self.startw <= x < self.startw + self.scalew and self.starth <= x < self.starth + self.scaleh:
            x1 = int(self.curr_x)
            y1 = int(self.curr_y)
            try:
                collision = self.static_obs[x1][y1]
            except:
                collision = False
            if collision:
                sprites.add(Explosion((x,y), self.exp_img))
                self.collision += 0 if self.pStaticHit == (x1,y1) else 1
                self.pStaticHit = (x1,y1)
                return
            for i in range(0, self.nobs):
                if 2.25 > self.dist(self.xobs[i], self.curr_x, self.yobs[i], self.curr_y):
                    sprites.add(Explosion((x,y), self.exp_img))
                    self.collision += 0 if self.pDynamicHit == i else 1
                    self.pDynamicHit = i
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

    def updateInformation(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs,vobs, future_x, future_y, future_heading, estimateStart, goal_location):
        if self.pathList[-1] != (curr_x, curr_y):
            self.pathList.append((curr_x, curr_y))
        self.curr_x = curr_x
        self.curr_y = curr_y
        self.start_heading = start_heading
        self.nobs = nobs
        self.xobs = xobs
        self.yobs = yobs
        self.hobs = hobs
        self.vobs = vobs
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
        x, y, heading = self.estimateStart
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
            self.dynamicOBS[index] = self.scale_item(self.xobs[index], self.yobs[index])
            draw_estimate = self.draw_vehicle(
                self.hobs[index], Color_Red, *self.dynamicOBS[index])
            start_radius = obs_radius
            sx= self.xobs[index]
            sy = self.yobs[index]
            c_color = 200
            c_bright = 0
            if draw_estimate:
                for i in range(5):
                    if i != 0:
                        sx += self.vobs[index]*math.sin(self.hobs[index])*3
                        sy += self.vobs[index]*math.cos(self.hobs[index])*3
                        c_color += 10
                        c_bright += 40
                    self.drawCircle((c_color,c_bright,c_bright),start_radius,*self.scale_item(sx,sy))
                    start_radius += obs_radius
            
            

        for obs in self.static_draw:
            self.draw_static_obs1((0, 0, 0), *obs)
      
        for i in range(len(self.goal_location)):
            if(self.distSquare(self.curr_x, self.curr_y, *self.goal_location[i]) <= 10):
                self.F_goal_location.append(self.goal_location.pop(i))
                break

        for goal in self.goal_location:
            self.draw_static_obs((0, 255, 0), *goal)

        for goal in self.F_goal_location:
            self.draw_static_obs((0, 255, 255), *goal)

    def distSquare(self, x1, y1, x2, y2):
        return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

    def drawCircle(self,color,radius, x, y):
        pygame.draw.circle(self.display, color, (int(x),int(y)),int(self.scalew*(radius/float(self.maxX))) ,1)

    def draw_static_obs(self, color, x, y):
        x1,y1 = self.scale_item(x, y)
        x2,y2 = self.scale_item(x+1, y+1)
        x1 = max(self.startw,x1)
        y1 = max(self.starth,y1)
        x2 = min(self.startw + self.scalew,x2)
        y2 = min(self.starth + self.scaleh,y2)
        if not(x1 > self.startw + self.scalew or y1 > self.starth + self.scaleh or x2 < self.startw or y2 < self.starth):
            pygame.draw.rect(self.display, color, (x1, y1, x2-x1, y2-y1))
        # pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
        #     x+1, y), self.scale_item(x+1, y+1), self.scale_item(x, y+1)))

    def draw_static_obs1(self, color, o1, o2):
        x1,y1 = self.scale_item(*o1)
        x2,y2 = self.scale_item(*o2)   
        x1 = max(self.startw,x1)
        y1 = max(self.starth,y1)
        x2 = min(self.startw + self.scalew,x2)
        y2 = min(self.starth + self.scaleh,y2)
        if not(x1 > self.startw + self.scalew or y1 > self.starth + self.scaleh or x2 < self.startw or y2 < self.starth):
            pygame.draw.rect(self.display, color, (x1, y1, x2-x1, y2-y1))

    def draw_vehicle(self, angle, color, x, y):
        if  self.startw <= x <= self.startw + self.scalew and self.starth <= y <= self.starth + self.scaleh:
            tX = []
            tY = []
            c = math.cos(angle)
            s = math.sin(angle)
            for i in range(3):
                tX.append(self.triangleX[i]*c - self.triangleY[i]*s + x)
                tY.append(self.triangleX[i]*s + self.triangleY[i]*c + y)
            pygame.draw.polygon(self.display, color, ((
                tX[0], tY[0]), (tX[1], tY[1]), (tX[2], tY[2])))
            return True
        return False

    def draw_text(self):
        scaleh = self.scaleh/(self.lines+1)
        scalew = self.scalew/(self.lines+1)
        for i in range(0, self.lines+2):
            text = str(int(self.factor*(self.moveY+(self.lines - i + 1)
                                        * self.maxY/(self.lines + 1))))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy(0, (i) * scaleh)
            x -= textsurface.get_width() + 10
            y -= textsurface.get_height()/2
            self.display.blit(textsurface, (x, y))
            text = str(int(self.factor*(self.moveX+(self.lines - i + 1)
                           * self.maxX/(self.lines + 1))))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy((self.lines - i + 1)*scalew, self.scaleh)
            x -= textsurface.get_width() / 2
            y += 10
            self.display.blit(textsurface, (x, y))

        text = "Current Location:(  {:.2f} , {:.2f}  )".format(
            self.curr_x, self.curr_y)
        textsurface = self.myfont.render(text, True, (0, 0, 0))
        self.display.blit(textsurface, (20, 10))

        text = "Collisions:(  {:.2f}  )".format(self.collision)
        textsurface1 = self.myfont.render(text, True, (0, 0, 0))
        self.display.blit(textsurface1, (20, 10 + textsurface.get_height()))

        text = "Cost:(  {:.2f}  )".format(self.cost)
        self.cost += 0.05
        textsurface2 = self.myfont.render(text, True, (0, 0, 0))
        self.display.blit(textsurface2, (20, 10 + textsurface1.get_height() + textsurface.get_height()))

        

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
