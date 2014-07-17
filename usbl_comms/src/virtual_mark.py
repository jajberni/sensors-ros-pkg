#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
import math;
import random;
import numpy
from auv_msgs.msg import NavSts  
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
     
class VirtualMark:
    def __init__(self):
        
        self.stateHat = rospy.Subscriber('stateHat', NavSts, self.onStateHat);
        self.genIn = rospy.Subscriber('start_generation', Bool, self.onGenerate);
        
        self.trackPoint = rospy.Publisher('TrackPoint_Gen', NavSts)
        self.usblSim = rospy.Publisher('usbl_nav', PointStamped)
        
        self.baseState = NavSts()
        self.generate = False;self.uRand = numpy.array([0.3, 0.1, 0.2, 0.15, 0.25, 0.21])
        self.Ts = 0.1
        self.u = 0.3
        
        self.distance = 0;
        self.step = 40;
        self.stepU = 100;
        self.cnt = 0;
        self.cntU = 0;
        #self.uRand = numpy.array([0.3, 0.1, 0.2, 0.15, 0.25, 0.21])
        self.uRand = numpy.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
        
        
    def onStateHat(self,data):
        if not self.generate:
            self.baseState = data
        else:
            self.generatePoint(data)
        
    def onGenerate(self,data):
        self.generate = data.data
            
    def generatePoint(self,data):
        self.baseState.position.north += self.u*self.Ts;
        self.distance += self.u*self.Ts;
               
        if abs(self.distance) > 30:
            self.distance = 0
            self.uRand *= -1;
        
        self.cnt += 1;
        if self.cnt == self.step:
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.point.x = self.baseState.position.north - data.position.north
            point.point.y = self.baseState.position.east - data.position.east
            self.usblSim.publish(point)
            self.baseState.header.stamp = rospy.Time.now()
            self.baseState.body_velocity.x = self.u;
            self.baseState.body_velocity.y = 0;
            self.trackPoint.publish(self.baseState)
            self.cnt = 0
        
        self.cntU += 1;    
        if self.cntU == self.stepU:
            self.cntU = 0;
            #self.u = 0.1 + 0.2 * random.random()
        
        idx = abs(int(self.distance/5));
        if idx > 5: 
            idx = 5
            
        self.u = self.uRand[idx];
        rospy.loginfo("Current speed:{0} - distance:{1}".format(self.u, self.distance))
                                
if __name__ == "__main__":
    rospy.init_node("virtual_mark");
    mark = VirtualMark(); 
    
    while not rospy.is_shutdown():
        rospy.spin();        
        
        
