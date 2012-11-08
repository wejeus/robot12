#!/usr/bin/env python

import roslib; roslib.load_manifest('amee')
import rospy, math, operator, sys
from amee.msg import MapVisualization, WallVisualization
#import and init pygame
import pygame
pygame.init() 

#create the screen
xMax = 1280
yMax = 768
window = pygame.display.set_mode((xMax, yMax)) 
pygame.display.flip()
#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

#draw it to the screen

def refresh(msg):
  window.fill((0, 0, 0))
  #print "Refresh"
  for wall in msg.walls:
    color = (255,0,0)
    if (wall.type == 1):
      color = (0,255,0)
    start = transform(wall.startX,wall.startY)
    end = transform(wall.endX,wall.endY)
    pygame.draw.line(window, color,start,end)
    #pygame.draw.line(window, (255,255,255),(0,0),(xMax, yMax))
    #print startY
    #print startX
    #print endX
    #print endY
  drawAmee(msg.robotPose.x,msg.robotPose.y,msg.robotPose.theta)
  pygame.display.flip() 

def drawAmee(x,y,theta):
  pygame.draw.circle(window,(255,255,255),transform(x,y),5)  
  start = transform(x,y)
  end = (int(start[0] + 5 * math.cos(theta)),int(start[1] - 5 * math.sin(theta)))
  pygame.draw.line(window,(255,0,0),start,end)  

def transform(x,y):
  return (int(2.0/3.0 * xMax) + int(x / 6 * xMax),yMax-100 - int(y / 6 * yMax))

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
