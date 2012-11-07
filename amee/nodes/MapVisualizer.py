#!/usr/bin/env python

import roslib; roslib.load_manifest('amee')
import rospy, math, operator, sys
from amee.msg import MapVisualization, WallVisualization
#import and init pygame
import pygame
pygame.init() 

#create the screen
xMax = 800
yMax = 800
window = pygame.display.set_mode((xMax, yMax)) 
pygame.display.flip()
#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

#draw it to the screen

def refresh(msg):
  window.fill((0, 0, 0))
  print "Refresh"
  for wall in msg.walls:
    startX = wall.startX/8 * xMax
    startY = wall.startY/8 * yMax
    endX = wall.endX/8 * xMax
    endY = wall.endY/8 * yMax
    color = (255,0,0)
    if (wall.type == 1):
      color = (0,255,0)
    pygame.draw.line(window, color,(startX + xMax/2,startY + yMax/2),(endX + xMax/2, endY + yMax/2))
    #pygame.draw.line(window, (255,255,255),(0,0),(xMax, yMax))
    print startY
    print startX
    print endX
    print endY
    pygame.display.flip() 
    

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
