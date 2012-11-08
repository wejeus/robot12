#!/usr/bin/env python

import roslib; roslib.load_manifest('amee')
import rospy, math, operator, sys
from amee.msg import MapVisualization, WallVisualization
#import and init pygame
import pygame
pygame.init() 

#create the screen
xMax = 800
yMax = 600
window = pygame.display.set_mode((xMax, yMax)) 
pygame.display.flip()
#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

#draw it to the screen

def refresh(msg):
  window.fill((0, 0, 0))
  #print "Refresh"
  for wall in msg.walls:
    color = (255,20,147)
    if (wall.type == 1):
      color = (255,255,0)
    start = transform(wall.startX,wall.startY)
    end = transform(wall.endX,wall.endY)
    pygame.draw.line(window, color,start,end)
    #pygame.draw.line(window, (255,255,255),(0,0),(xMax, yMax))
    #print startY
    #print startX
    #print endX
    #print endY
  for tag in msg.tags:
    drawTag(tag.x,tag.y)  
  drawAmee(msg.robotPose.x,msg.robotPose.y,msg.robotPose.theta)
  pygame.display.flip() 

def drawAmee(x,y,theta):
  pygame.draw.circle(window,(255,255,255),transform(x,y),5)  
  start = transform(x,y)
  end = (int(start[0] + 5 * math.cos(theta)),int(start[1] - 5 * math.sin(theta)))
  pygame.draw.line(window,(255,0,0),start,end)  

def drawTag(x,y):
  (px,py) = transform(x,y)
  rect = pygame.Rect(px - 5,py - 5,10,10)
  pygame.draw.rect(window,(255,0,0),rect,2)

def transform(x,y):
  return (int(2.0/3.0 * xMax) + int(x / 6 * xMax),yMax-500 - int(y / 6 * yMax))

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
