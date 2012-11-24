#!/usr/bin/env python

import roslib; roslib.load_manifest('amee')
import rospy, math, operator, sys
from amee.msg import MapVisualization, WallVisualization, NodeMsg, GraphMsg, Pose
#import and init pygame
import pygame
pygame.init() 

#create the screen
resolution = (800,600)
offset = (0,0)
scale = (1,1)
border = (30,30)
window = pygame.display.set_mode((resolution[0] + 2 * border[0], resolution[1] + 2 * border[1])) 
pygame.display.flip()
#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

#draw it to the screen
nodes = []
pose = (0,0,0)

def refresh(msg):
  window.fill((0, 0, 0))
  findScaleAndOffset(msg)
  #print "Refresh"
  for wall in msg.walls:
    color = (255,20,147)
    if (wall.type == 1):
      color = (255,20,147)
    start = transform(wall.startX,wall.startY)
    end = transform(wall.endX,wall.endY)
    #print end
    #print start
    pygame.draw.line(window, color,start,end)
    #pygame.draw.line(window, (255,255,255),(0,0),(xMax, yMax))
    #print start[0]
    #print start[1]
    #print end[0]
    #print end[1]
  for tag in msg.tags:
    drawTag(tag.x,tag.y)  
  drawAmee(pose[0],pose[1],pose[2])

  for node in nodes:
    (x, y, theta, nodeID, edges) = node
    start = transform(x,y)
    for nId in edges:
      if (nId > nodeID & nId >= 0 & nId < len(nodes)):
        endNode = nodes[nId]
        end = transform(endNode[0], endNode[1])
        pygame.draw.line(window,(100,0,0),start,end) 
    drawNode(x, y, theta)

  pygame.display.flip() 

def findScaleAndOffset(msg):
  global scale
  global offset
  minX = 0
  minY = 0
  maxY = 0
  maxX = 0
  for wall in msg.walls:
    minX = min(minX,wall.startX)
    minY = min(minY,wall.startY)
    maxX = max(maxX,wall.endX)
    maxY = max(maxY,wall.endY)
  maxX = max(maxX,pose[0])
  maxY = max(maxY,pose[1])
  minX = min(minX,pose[0])
  minY = min(minY,pose[1])
  scale = (resolution[0] / abs(maxX - minX), resolution[1] / abs(maxY - minY))
  if (scale[0] * 0.12 > 1/4 * resolution[0]): #the robot is half of the screen!
    scale = (resolution[0] / 4, resolution[1] / 4)
  offset = (abs(minX * scale[0]) + border[0], abs(minY * scale[1]) + border[1])
  #print ("Scale: " + str(scale))
  #print ("Offset: "  + str(offset))

def drawNode(x,y,theta):
  r = int(scale[0] * 0.02)
  pygame.draw.circle(window,(0,255,0),transform(x,y),r)  
  start = transform(x,y)
  end = (int(start[0] + r * math.cos(theta)),int(start[1] - r * math.sin(theta)))
  pygame.draw.line(window,(255,0,0),start,end)  

def drawAmee(x,y,theta):
  r = int(scale[0] * 0.12)
  pygame.draw.circle(window,(255,255,255),transform(x,y),r)  
  start = transform(x,y)
  end = (int(start[0] + r * math.cos(theta)),int(start[1] - r * math.sin(theta)))
  pygame.draw.line(window,(255,0,0),start,end)  

def drawTag(x,y):
  (px,py) = transform(x,y)
  rect = pygame.Rect(px - 5,py - 5,10,10)
  pygame.draw.rect(window,(255,0,0),rect,2)

def transform(x,y):
  return (int(scale[0] * x + offset[0]),resolution[1] - int(scale[1] * y + offset[1]))

def onNodeMsgUpdate(msg):
  global nodes
  nodes = []
  for nodeMsg in msg.nodes:
    edges = []
    for neighbor in nodeMsg.edges:
      edges.append(neighbor)
    node = (nodeMsg.pose.x, nodeMsg.pose.y, nodeMsg.pose.theta, nodeMsg.nodeID, edges)
    nodes.append(node)

def poseMsg(msg):
  global pose
  pose = (msg.x, msg.y, msg.theta)   


if __name__ == '__main__':
    # Register this node
    rospy.init_node("MapVisualizationNode")
    
    try:
        # The TOPIC we want to listen to
        rospy.Subscriber("/amee/map/visualization", MapVisualization, refresh)
        rospy.Subscriber("/amee/map/graph", GraphMsg, onNodeMsgUpdate)
        rospy.Subscriber("/amee/pose", Pose, poseMsg)

        rospy.loginfo("Displaying map...")

        rospy.spin()
       # while not rospy.is_shutdown():
       #     rospy.spin()

    except rospy.ROSInterruptException: pass