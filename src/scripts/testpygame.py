#!/usr/bin/env python
import rospy
import os
import pygame
import time
import random
 
class pyscope :
    screen = None;
    
    def __init__(self):
        "Ininitializes a new pygame screen using the framebuffer"
        # Based on "Python GUI in Linux frame buffer"
        # http://www.karoltomala.com/blog/?p=679
        disp_no = os.getenv("DISPLAY")
        if disp_no:
            print "I'm running under X display = {0}".format(disp_no)
        
        # Check which frame buffer drivers are available
        # Start with fbcon since directfb hangs with composite output
        drivers = ['fbcon', 'directfb', 'svgalib']
        found = False
        for driver in drivers:
            # Make sure that SDL_VIDEODRIVER is set
            if not os.getenv('SDL_VIDEODRIVER'):
                os.putenv('SDL_VIDEODRIVER', driver)
            try:
                pygame.display.init()
            except pygame.error:
                print 'Driver: {0} failed.'.format(driver)
                continue
            found = True
            break
    
        if not found:
            raise Exception('No suitable video driver found!')
        
        size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
        print "Framebuffer size: %d x %d" % (size[0], size[1])
        self.screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
        # Clear the screen to start
        self.screen.fill((255, 255, 255))        
        # Initialise font support
        pygame.font.init()
        # Render the screen
        pygame.display.update()
        self.font = pygame.font.Font(None, 100)
 
    def __del__(self):
        "Destructor to make sure pygame shuts down, etc."
 
    def test(self):
        # Fill the screen with red (255, 0, 0)
        red = (255, 0, 0)
        self.screen.fill(red)
        # Update the display
        pygame.display.update()

    def rostime(self):
        background = pygame.Surface(self.screen.get_size())
        background = background.convert()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    break
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        break
            background.fill((255,255,255))
            text = self.font.render("%f"%rospy.Time.now().to_sec(), 1,
                    (0,0,0), (255,255,255))
            textpos = text.get_rect()
            textpos.centerx = background.get_rect().centerx
            textpos.centery = background.get_rect().centery
            background.blit(text, textpos)
            self.screen.blit(background, (0,0))
            pygame.display.flip()
            #r.sleep()
        pygame.quit()
 
# Create an instance of the PyScope class
rospy.init_node("rostime_display")
scope = pyscope()
scope.rostime()
