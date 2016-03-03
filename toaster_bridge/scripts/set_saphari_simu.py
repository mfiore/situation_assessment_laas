#!/usr/bin/env python

import sys
import rospy
from toaster_msgs.srv import *
from toaster_msgs.msg import *
from geometry_msgs.msg import *

def set_pose_client(name,type,x,y,z,roll,pitch,yaw):
  try:
    add_entity=rospy.ServiceProxy("/toaster_simu/add_entity",AddEntity)
    res=add_entity(name,name,type,"")

    set_entity_pose=rospy.ServiceProxy("/toaster_simu/set_entity_pose",SetEntityPose)
    res=set_entity_pose(name,'',type,x,y,z,roll,pitch,yaw)
  
  except rospy.ServiceException, e:
    print "Service call failed %s"%e


def create_area(id,name,center,ray):
  try:
    add_area=rospy.ServiceProxy("/area_manager/add_area",AddArea)
    my_area=toaster_msgs.msg.Area()
    my_area.id=id
    my_area.name=name
    my_area.areaType='room'
    my_area.entityType='entities'
    my_area.isCircle=True
    my_area.center=center
    my_area.ray=ray

    res=add_area(myArea=my_area)
  except rospy.ServiceException, e:
   print "Service call failed %s"%e 


   #rosservice call /pdg/manage_stream "{morseHuman: False, niutHuman: False, groupHuman: False, mocapHuman: False, 
   #adreamMocapHuman: False,
    # toasterSimuHuman: True, pr2Robot: True, spencerRobot: False, toasterSimuRobot: False,
     #toasterSimuObject: True}" 

def set_pdg():
  try:
    manage_stream=rospy.ServiceProxy("/pdg/manage_stream",AddStream)
    res=manage_stream(False,False,False,False,False,True,True,False,False,True)
  except rospy.ServiceException, e:
    print "Service call failed %s"%e

if __name__=="__main__":
  rospy.wait_for_service("/toaster_simu/set_entity_pose")
  rospy.wait_for_service("/toaster_simu/add_entity")
  set_pose_client("TABLE_4","object",-0.3,11.4,0.0,0.0,0.0,1.6)
  set_pose_client("GREY_TAPE","object",-0.8,11.7,0.8,0.0,0.0,0.0)
  set_pose_client("LOTR_TAPE","object",0.3,11.7,0.9,0,0,0)
  set_pose_client("WALLE_TAPE","object",-0.2,11.7,0.9,0,0,0)
  set_pose_client("IKEA_SHELF_LIGHT_1","object",-3.2,13.9,0,0,0,-0.5)
  set_pose_client("IKEA_SHELF_LIGHT_2","object",-0.3,15.3,0,0,0,-1.5)
  set_pose_client("IKEA_SHELF_LIGHT_3","object",2.3,13.7,0,0,0,4.1) 
  set_pose_client("HERAKLES_HUMAN1","human",-2.2,13.6,0.0,0.0,0.0,0)
#  set_pose_client("PR2_ROBOT","robot",1.3,12.9,0,0,0,2.9)
  set_pdg()
  center=Point(-2.2,13.6,0.0)
  create_area(0,"try",center,5)