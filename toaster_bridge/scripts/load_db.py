#!/usr/bin/env python

import sys
import rospy
from toaster_msgs.srv import *
from toaster_msgs.msg import *
import xml.etree.ElementTree as ET

def add_facts(facts,agent):
  try:
    add_facts=rospy.ServiceProxy("/database/add_facts_to_agent",AddFactsToAgent)
    res=add_facts(facts,agent)

  
  except rospy.ServiceException, e:
    print "Service call failed %s"%e


if __name__=="__main__":
  rospy.wait_for_service("/database/add_facts_to_agent")
  xml_path=rospy.get_param("/toaster_bridge/database_xml_path")
  robot_name=rospy.get_param("/toaster_bridge/robot_name")
  tree=ET.parse(xml_path)
  root=tree.getroot()
  facts=[]
  for child in root:
    fact=Fact()
    fact.property=child.attrib["property"]
    fact.propertyType=child.attrib["propertyType"]
    fact.subProperty=child.attrib["subProperty"]
    fact.subjectId=child.attrib["subjectId"]
    fact.targetId=child.attrib["targetId"]
    fact.subjectOwnerId=child.attrib["subjectOwnerId"]
    fact.targetOwnerId=child.attrib["targetOwnerId"]
    fact.valueType=int(child.attrib["valueType"])
    fact.factObservability=float(child.attrib["factObservability"])
    fact.doubleValue=float(child.attrib["doubleValue"])
    fact.stringValue=child.attrib["stringValue"]
    fact.confidence=float(child.attrib["confidence"])
    fact.time=int(child.attrib["time"])
    fact.timeStart=int(child.attrib["timeStart"])
    fact.timeEnd=int(child.attrib["timeEnd"])
    facts.append(fact)
  add_facts(facts,robot_name)  