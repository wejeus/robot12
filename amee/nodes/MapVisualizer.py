#!/usr/bin/env python

import roslib; roslib.load_manifest('amee')
import rospy, math, operator, sys
from amee.msg import MapVisualization, WallVisualization
#import and init pygame
import pygame
pygame.init() 

#create the screen
window = pygame.display.set_mode((1024, 768)) 

#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

#draw it to the screen
pygame.display.flip() 

def refresh(self,msg):
	pygame.display.flip()
	for event in pygame.event.get(): 
      		if event.type == pygame.QUIT: 
          		sys.exit(0)


if __name__ == '__main__':
    # Register this node
    rospy.init_node("MapVisualizationNode")
    
    try:
        # The TOPIC we want to listen to
        rospy.Subscriber("/amee/map/visualization", MapVisualization, refresh)

        rospy.loginfo("... done! Entering spin() loop")

        rospy.spin()
       # while not rospy.is_shutdown():
       #     rospy.spin()

    except rospy.ROSInterruptException: pass
