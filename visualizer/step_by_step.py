#!/usr/bin/env python
import pygame
from pygame.locals import *
# import threading
# from threading import Lock
import sys
import math
# import os
# import glob
import numpy as np

# import time

# mutex = Lock()

Color_line = (0, 0, 0)
Color_line_middle = (180, 180, 180)
Color_line_path = (240, 0, 0)

Color_Red = (255, 0, 0)
Color_BLUE = (0, 0, 255)
Color_GREEN = (0, 255, 0)
Color_PURPLE = (255, 0, 255)
Color_CYAN = (0, 255, 255)

Color_Red_dark = (155, 0, 0)
Color_BLUE_dark = (0, 0, 155)
Color_GREEN_dark = (0, 155, 0)
Color_PURPLE_dark = (155, 0, 155)
Color_CYAN_dark = (0, 155, 155)

color_base = 10
color_range = 245

colors = [Color_Red, Color_BLUE, Color_GREEN, Color_PURPLE, Color_CYAN, Color_Red_dark,
          Color_BLUE_dark, Color_GREEN_dark, Color_PURPLE_dark, Color_CYAN_dark]
theApp = 0

sprites = pygame.sprite.Group()


def dist_square(x1, y1, x2, y2):
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)


class PLOT:
    def __init__(self, blocked, xlim, ylim, goal, in_file_name):
        self._running = True
        self.display = None
        self._image_surf = None
        self.screenH = 800
        self.screenW = 600
        self.maxX = xlim
        self.maxY = ylim
        self.xLim = xlim
        self.yLim = ylim
        self.lines = 9
        self.originX = 0
        self.originY = 0
        self.moveX = 0
        self.moveY = 0
        self.cost = 0
        self.future_x = []
        self.future_y = []
        self.goals = goal
        self.covered_goals = []
        self.future_heading = []
        self.estimateStart = (0, 0, 0)
        self.triangleX = (0, -5, 5)
        self.triangleY = (-10, 10, 10)
        self.static_obs = blocked
        self.maxColor = -10000000
        self.minColor = 100000000
        self.displayNumber = 0
        self.input_file_name = in_file_name
        self.input_lines = []
        self.input_index = 0
        # declare stuff defined in on_init
        self.w, self.h, self.scaleW, self.scaleH, self.startW, self.startH = 0, 0, 0, 0, 0, 0
        self.curr_x, self.curr_y, self.start_heading, self.nobs = 0, 0, 0, 0
        self.xobs, self.yobs, self.hobs, self.costobs, self.heauristicobs, self.shapeobs = [], [], [], [], [], []
        self.background = None
        pygame.font.init()
        try:
            self.font = pygame.font.Font("r.ttf", 15)
            self.font1 = pygame.font.Font("r.ttf", 8)
        except:
            self.font = pygame.font.SysFont(None, 20)
            self.font1 = pygame.font.SysFont(None, 10)

    def on_init(self):
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.screenH, self.screenW), HWSURFACE | RESIZABLE)
        self.w, self.h = pygame.display.get_surface().get_size()
        self.scaleW = self.w * 0.8
        self.scaleH = self.h * 0.8
        self.startW = self.w * 0.1
        self.startH = self.h * 0.1
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
        self.costobs = []
        self.heauristicobs = []
        self.shapeobs = []
        self.reset()

    def on_event(self, event):
        if event.type == QUIT:
            pygame.quit()
            exit(0)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                self.originX -= self.scaleW / (self.lines + 1)
                self.moveX += self.maxX / (self.lines + 1)
            elif event.key == pygame.K_RIGHT:
                self.originX += self.scaleW / (self.lines + 1)
                self.moveX -= self.maxX / (self.lines + 1)
            elif event.key == pygame.K_DOWN:
                self.originY += self.scaleH / (self.lines + 1)
                self.moveY += self.maxY / (self.lines + 1)
            elif event.key == pygame.K_UP:
                self.originY -= self.scaleH / (self.lines + 1)
                self.moveY -= self.maxY / (self.lines + 1)
            elif event.key == pygame.K_MINUS:
                d = 1 if self.maxX < 100 and self.maxY < 100 else 10
                p = d * 10
                x, y = self.scale_item(self.curr_x, self.curr_y)
                x = int(round(((x - self.startW) / self.scaleW) * 10, 5))
                y = int(round(((1 - (y - self.startH) / self.scaleH) * 10), 5))
                self.maxX += p
                self.maxY += p
                self.originY = self.scaleH * \
                               ((int(self.curr_y) / d) * d / float(self.maxY)) - \
                               y * self.scaleH / (self.lines + 1)
                self.originX = -self.scaleW * \
                               ((int(self.curr_x) / d) * d / float(self.maxX)) + \
                               x * self.scaleW / (self.lines + 1)
                cy = (int(self.curr_y) / d) * d
                cx = (int(self.curr_x) / d) * d
                self.moveY = cy - y * self.maxY / (self.lines + 1)
                self.moveX = cx - x * self.maxX / (self.lines + 1)
            elif event.key == pygame.K_EQUALS:
                if self.maxX >= 20 and self.maxY >= 20:
                    d = 10 if self.maxX >= 200 and self.maxY >= 200 else 1
                    p = d * 10
                    x, y = self.scale_item(self.curr_x, self.curr_y)
                    x = int(round(((x - self.startW) / self.scaleW) * 10, 5))
                    y = int(round(((1 - (y - self.startH) / self.scaleH) * 10), 5))
                    self.maxX -= p
                    self.maxY -= p
                    self.originY = self.scaleH * \
                                   ((int(self.curr_y) / d) * d / float(self.maxY)) - \
                                   y * self.scaleH / (self.lines + 1)
                    self.originX = -self.scaleW * \
                                   ((int(self.curr_x) / d) * d / float(self.maxX)) + \
                                   x * self.scaleW / (self.lines + 1)
                    cy = (int(self.curr_y) / d) * d
                    cx = (int(self.curr_x) / d) * d
                    self.moveY = cy - y * self.maxY / (self.lines + 1)
                    self.moveX = cx - x * self.maxX / (self.lines + 1)
            elif event.key == pygame.K_SPACE:
                d = 10 if self.maxX >= 100 and self.maxY >= 100 else 1
                self.maxY = self.yLim
                self.maxX = self.xLim
                self.originY = self.scaleH * \
                               ((int(self.curr_y) / d) * d / float(self.maxY))
                self.originX = -self.scaleW * \
                               ((int(self.curr_x) / d) * d / float(self.maxX))
                self.moveY = (int(self.curr_y) / d) * d
                self.moveX = (int(self.curr_x) / d) * d
            elif event.key == pygame.K_r:
                self.reset()
            elif event.key == pygame.K_n:
                self.nobs += 1
            elif event.key == pygame.K_BACKSPACE:
                self.nobs -= 1
                if self.nobs < 0:
                    self.nobs = 0
            elif event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit(0)

    def on_render(self):
        self.display.blit(self.background, (0, 0))
        self.draw_line()
        self.draw_text()
        self.draw_obs()
        sprites.draw(self.display)
        sprites.update()
        pygame.display.flip()

    def on_cleanup(self):
        pygame.quit()

    def draw_line(self):
        pygame.draw.line(self.display, Color_line, (self.startW,
                                                    self.startH), (self.startW + self.scaleW, self.startH))
        pygame.draw.line(self.display, Color_line, (self.startW,
                                                    self.startH), (self.startW, self.startH + self.scaleH))
        pygame.draw.line(self.display, Color_line, (self.startW, self.startH +
                                                    self.scaleH),
                         (self.startW + self.scaleW, self.startH + self.scaleH))
        pygame.draw.line(self.display, Color_line, (self.startW + self.scaleW,
                                                    self.startH),
                         (self.startW + self.scaleW, self.startH + self.scaleH))
        scaleh = self.scaleH / (self.lines + 1)
        scalew = self.scaleW / (self.lines + 1)
        for i in range(1, self.lines + 1):
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                0, i * scaleh), self.scale_xy(self.scaleW, i * scaleh))
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                i * scalew, 0), self.scale_xy(i * scalew, self.scaleH))
            # self.draw_text(self.scalew,i * scaleh, i * self.maxX/(self.lines + 1) )

    # TODO! -- why were there two "cost" parameters? (costobs, cost)
    def update_information(self, x, y, heading, h, shape, cost):
        # self.nobs += 1
        self.xobs.append(x)
        self.yobs.append(y)
        self.hobs.append(heading)
        self.costobs.append(cost)
        self.heauristicobs.append(h)
        self.shapeobs.append(shape)
        self.maxColor = max(self.maxColor, cost)
        self.minColor = min(self.minColor, cost)
        self.displayNumber = self.maxColor - self.minColor
        if self.displayNumber == 0:
            self.displayNumber = 1

    def get_color(self, cost):
        return 0, 0, 255 - ((cost - self.minColor) / self.displayNumber) * color_range

    def draw_obs(self):
        for obs in self.static_obs:
            self.draw_static_obs(Color_Red, *obs)
        for index in range(self.nobs):
            if self.shapeobs[index] == 1:
                yy = self.draw_vehicle(
                    self.hobs[index], self.get_color(self.costobs[index]),
                    *self.scale_item(self.xobs[index], self.yobs[index]))
                self.draw_text_cost(self.costobs[index], self.heauristicobs[index],
                                    self.scale_item(self.xobs[index], self.yobs[index])[0], yy)
            else:
                self.draw_circle(self.get_color(self.costobs[index]),
                                 *self.scale_item(self.xobs[index], self.yobs[index]))
        for i in range(len(self.goals)):
            if dist_square(self.curr_x, self.curr_y, *self.goals[i]) <= 10:
                self.covered_goals.append(self.goals.pop(i))
                break

        for goal in self.goals:
            self.draw_static_obs((0, 255, 0), *goal)

        for goal in self.covered_goals:
            self.draw_static_obs((0, 255, 255), *goal)

    def draw_static_obs(self, color, x, y):
        pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
            x + 1, y), self.scale_item(x + 1, y + 1), self.scale_item(x, y + 1)))

    def draw_goal(self, x, y, covered=False):
        if covered:
            self.draw_static_obs((0, 255, 255), x, y)
        else:
            self.draw_static_obs((0, 255, 0), x, y)

    def draw_dot(self, color, x, y):
        pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
            x + 0.3, y), self.scale_item(x + 0.3, y + 0.3), self.scale_item(x, y + 0.3)))

    def draw_vehicle(self, angle, color, x, y):
        t_x = []
        t_y = []
        c = math.cos(angle)
        s = math.sin(angle)
        for i in range(3):
            t_x.append(self.triangleX[i] * c - self.triangleY[i] * s + x)
            t_y.append(self.triangleX[i] * s + self.triangleY[i] * c + y)
        pygame.draw.polygon(self.display, color, ((t_x[0], t_y[0]), (t_x[1], t_y[1]), (t_x[2], t_y[2])))
        return min(t_y[0], t_y[1], t_y[2])

    def draw_circle(self, color, x, y):
        pygame.draw.circle(self.display, color, (int(x), int(y)), 2)

    def draw_text_cost(self, cost, heuristic, x, y):
        text = "f " + str(int(cost + heuristic))
        text_surface = self.font.render(text, True, (0, 0, 0))
        y += text_surface.get_height()
        self.display.blit(text_surface, (x, y))

        text = "g " + str(int(cost))
        text_surface = self.font.render(text, True, (0, 0, 0))
        y += text_surface.get_height()
        self.display.blit(text_surface, (x, y))

        text = "h " + str(int(heuristic))
        text_surface = self.font.render(text, True, (0, 0, 0))
        y += text_surface.get_height()
        self.display.blit(text_surface, (x, y))

    def draw_text(self):
        scale_h = self.scaleH / (self.lines + 1)
        scale_w = self.scaleW / (self.lines + 1)
        for i in range(0, self.lines + 2):
            text = str(int(self.moveY + (self.lines - i + 1)
                           * self.maxY / (self.lines + 1)))
            text_surface = self.font.render(text, True, (0, 0, 0))
            x, y = self.scale_xy(0, (i) * scale_h)
            x -= text_surface.get_width() + 10
            y -= text_surface.get_height() / 2
            self.display.blit(text_surface, (x, y))
            text = str(int(self.moveX + (self.lines - i + 1)
                           * self.maxX / (self.lines + 1)))
            text_surface = self.font.render(text, True, (0, 0, 0))
            x, y = self.scale_xy((self.lines - i + 1) * scale_w, self.scaleH)
            x -= text_surface.get_width() / 2
            y += 10
            self.display.blit(text_surface, (x, y))

    def update(self):
        event = pygame.event.wait()
        self.on_event(event)
        self.on_render()

    def stop(self):
        self.on_cleanup()

    def scale_xy(self, x, y):
        return self.startW + x, self.startH + y

    def scale_item(self, x, y):
        return self.originX + self.startW + self.scaleW * (
                x / float(self.maxX)), self.originY + self.startH + self.scaleH - self.scaleH * (
                       y / float(self.maxY))

    def scale_path(self, x, y):
        return self.originX + self.startW + self.scaleW * (
                x / float(self.maxX)), self.originY + self.startH + self.scaleH - self.scaleH * (
                       y / float(self.maxY))

    def reset(self):
        with open(self.input_file_name, "r") as input_file:
            input_lines = input_file.readlines()
        self.input_index = 0
        self.maxColor = -1000000
        self.minColor = 1000000
        self.nobs = 0
        self.xobs = []
        self.yobs = []
        self.hobs = []
        self.costobs = []
        self.heauristicobs = []
        self.shapeobs = []
        for line in input_lines:
            line = line.split(' ')
            if len(line) < 3 or line[0].strip().lower() != 'planner' or line[1].strip().lower() != 'visualization:':
                continue
            if line[2].strip().lower() == 'done':
                continue  # ignore "done" for now
                # break
            if len(line) > 18 and line[18].strip().lower() != 'vis2':
                continue
            xobs = (float(line[2]))
            yobs = (float(line[3]))
            hobs = (float(line[4]))
            costobs = (float(line[9]))
            heauristicobs = (float(line[12]))
            if line[15].strip().lower() == 'dot':
                shapeobs = 0
            else:
                shapeobs = 1
            theApp.update_information(xobs, yobs, hobs, heauristicobs, shapeobs, costobs)


def dist(x, x1, y, y1):
    return (x - x1) ** 2 + (y - y1) ** 2


if __name__ == "__main__":

    map_file_name = goal_file_name = None
    if len(sys.argv) == 4 and sys.argv[1] == "-test":
        base_name = sys.argv[2]
        map_file_name = base_name + ".map"
        goal_file_name = base_name + ".goal"

    elif len(sys.argv) == 4:
        map_file_name = sys.argv[1]
        goal_file_name = sys.argv[2]

    if len(sys.argv) != 4:
        print 'Usage: "./step_by_step.py mapfile goalfile inputfile" or\n       "./step_by_step.py -test testname inputfile"\n'
        exit(0)

    input_file_name = sys.argv[3]

    with open(map_file_name, "r") as map_file:
        map_contents = map_file.readlines()
    static_obs = []
    goal_location = []
    max_x = np.abs(int(int(map_contents[1])))
    max_y = np.abs(int(int(map_contents[2])))
    for i in range(3, len(map_contents)):
        for j in range(0, len(map_contents[i])):
            if map_contents[i][j] == '#':
                static_obs.append((j, (max_y - i + 1)))

    with open(goal_file_name, "r") as goal_file:
        goal_file_contents = goal_file.readlines()
    numOfGoal = int(goal_file_contents[0])
    for i in range(1, len(goal_file_contents)):
        s = goal_file_contents[i].split(' ')
        goal_location.append((float(s[0]), float(s[1])))

    theApp = PLOT(static_obs, max_x, max_y, goal_location, input_file_name)
    theApp.on_init()

    try:
        while True:
            theApp.update()
    except Exception as e:
        print "ERROR"
        print(e)
        raise
    # finally:
    #     print "TERMINATE"
    #     theApp.stop()
    #     exit(0)
