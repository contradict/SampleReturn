#!/usr/bin/env python
import rospy
import pygame
import numpy as np

rospy.init_node("rostime_display")
pygame.init()
screen = pygame.display.set_mode((1024,768))
background = pygame.Surface(screen.get_size())
background = background.convert()

font = pygame.font.Font(None, 100)
while True:
    #for event in pygame.event.get():
    #    if event.type == QUIT:
    #        break
    background.fill((0,0,0))
    text = font.render("%f"%rospy.Time.now().to_sec(), 1, (255,255,255))
    textpos = text.get_rect()
    textpos.centerx = background.get_rect().centerx
    background.blit(text, textpos)
    screen.blit(background, (0,0))
    pygame.display.flip()

