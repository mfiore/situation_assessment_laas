#!/usr/bin/env python

import sys
import rospy
from toaster_msgs.srv import *

def set_pose_client(name,type,x,y,z,roll,pitch,yaw):
  try:
    add_entity=rospy.ServiceProxy("/toaster_simu/add_entity",AddEntity)
    res=add_entity(name,name,type,"")

    set_entity_pose=rospy.ServiceProxy("/toaster_simu/set_entity_pose",SetEntityPose)
    res=set_entity_pose(name,'',type,x,y,z,roll,pitch,yaw)
  
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
  set_pose_client("PR2_ROBOT","robot",1.3,12.9,0,0,0,2.9)
