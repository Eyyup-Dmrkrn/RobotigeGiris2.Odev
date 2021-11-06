# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 19:05:50 2021

@author: eyp_d
"""

import pygame
import time
from math import*
import random
from scipy.integrate import quad
from scipy.misc import derivative

class Environment:
    def __init__(self, dimentions):
        
        #Colors
        self.black = (0, 0 ,0)
        self.white = (255 ,255, 255)
        self.green = (0, 255, 0)
        self.blue  = (0, 0, 255)
        self.red   = (255, 0, 0)
        self.yel   = (255, 255, 0)
        
        #map dims
        self.height = dimentions[0]
        self.width  = dimentions[1]
        
        # window settings
        pygame.display.set_caption("Differential Drive Robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        
        # text variables
        self.font = pygame.font.Font("freesansbold.ttf", 25)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimentions[1]-350,
                                 dimentions[0]-160)
        
        self.font2 = pygame.font.Font("freesansbold.ttf", 35)
        self.text2 = self.font2.render("default", True, self.white, self.black)
        self.textRect2 = self.text2.get_rect()
        self.textRect2.center = (dimentions[1]-500,
                                 dimentions[0]-200)
        
        # trail coordinate list
        self.trail_set = []
        
    def write_info(self, vl, vr, theta, x, y):
        txt = f"vl: {vl:.2f} vr: {vr:.2f}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
        txt2 = f"x: {x:.1f} y: {y:.1f} theta: {degrees(theta):.1f}"
        self.text2 = self.font2.render(txt2, True, self.white, self.black)
        self.map.blit(self.text2, self.textRect2)
        
            
    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yel, (self.trail_set[i][0], self.trail_set[i][1]),
                                 (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
            
        if self.trail_set.__sizeof__()>30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)
        
    def robot_frame(self, pos, rotation):
        n = 90
        centerx, centery = pos
        x_axis = (centerx + n*cos(-rotation), centery + n*sin(-rotation))
        y_axis = (centerx + n*cos(-rotation + pi/2),
                  centery + n*sin(-rotation + pi/2))
                   
        pygame.draw.line(self.map, self.red, (centerx, centery), x_axis, 5)
        pygame.draw.line(self.map, self.green, (centerx, centery), y_axis, 5) 
        
class Robot:
    def __init__(self, startpos, robotImg, goalImg, length, x_goal, y_goal):
        
        # meters to pixels
        self.m2p = 1
            
        # robot dims
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = startpos[2]
        self.l = length 
        self.x_goal = 0
        self.y_goal = 0
        self.angle_goal = 0
        self.K_linear = 0.52
        self.K_angular = 50
        self.K_Integrate = 1
        self.distance = 0
        self.R = 0.1
        self.d_star = 100
        self.t = 0
        self.now = time.time()

        self.vl = 0
        self.vr = 0
        self.vr = self.vr*self.m2p
        self.vl = self.vl*self.m2p

        self.wr = 0
        self.wl = 0
        self.w  = 0
        self.dt = 0.0012

        # graphics
        # car image
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center = (self.x, self.y))
        
        # goal image
        self.goal_sign = pygame.image.load(goalImg)
        
    def draw (self, map):
        map.blit(self.goal_sign, (self.x_goal-20, self.y_goal-20))
        map.blit(self.rotated, self.rect)
            
    def move (self):    
        self.systemDynamics()
        time.sleep(self.dt)
    
    def systemDynamics (self): 
    
        self.integrator = quad(self.integrand, 0, self.t/100)
        
        self.v = self.K_linear*self.e + self.K_Integrate*self.integrator[0] 
        
        self.angle_goal = atan2(self.y-self.y_goal, self.x_goal-self.x)
        self.e_phi_prime = (self.angle_goal-self.theta)
        self.e_phi = atan2( sin(self.e_phi_prime), cos(self.e_phi_prime))
        self.w = self.K_angular*self.e_phi

        self.vl = (2*self.v - self.w*self.l)/(2*100000)
        self.vr = (2*self.v + self.w*self.l)/(2*100000)

        # Define wheels linear velocities
        self.wr = self.vr/self.R
        self.wl = self.vl/self.R
        
        self.differentialEqs()
   
        # reset theta
        if self.theta > 2*pi or self.theta < -2*pi:
            self.theta = 0  
                
        self.rotated = pygame.transform.rotozoom(self.img, degrees(self.theta), 2)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))   
        # self.goal_sign = self.
        
    def integrand(self, t):
        # Go to Goal Equations
        self.distance = sqrt((self.x-self.t)**2 + (self.y-cos(self.t))**2)
        self.e = self.distance - self.d_star
        return self.e   

    
    def differentialEqs (self):
     
        # Differential Equations
        self.x += (self.v*cos(self.theta))*self.dt
        self.y -= (self.v*sin(self.theta))*self.dt
        self.theta = (self.theta + self.w*self.dt)

# initialisation
pygame.init()               

#start positions
start = (30, 550, pi/2)

# dimentions
dims = (800, 1200)

# the envir
environment = Environment(dims)

# the robot
robot = Robot(start, "car.png", "cross.png", 0.00003*3775, 0, 0)


other_time = time.time() + 100
# simulation loop
while time.time() < other_time:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()       
    
    robot.t = time.time() - robot.now
    
    print(robot.t)
    # counter = int(end_time - time.time())
    pygame.display.update()
    environment.map.fill(environment.black)
    
    robot.move()
    
    environment.write_info((robot.vl)/robot.m2p, (robot.vr)/robot.m2p, robot.theta, (robot.x), (robot.y))
    robot.draw(environment.map)
    environment.robot_frame((robot.x, robot.y), robot.theta)
    environment.trail((robot.x, robot.y))  
        
# =============================================================================
#     if robot.distance < 200:   
#         robot.x_goal = (robot.t)*50
#         robot.y_goal = (cos(robot.t))*50 + 400
#         robot.x = robot.x_goal + 20
#         robot.y = robot.y_goal + 20
# =============================================================================
    robot.x_goal = (robot.t)*63 + 100
    robot.y_goal = (cos(robot.t))*63 + 400  
    
    
    if robot.distance < 0.1:
        break
    