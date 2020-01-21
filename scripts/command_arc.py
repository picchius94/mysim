#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
from scipy.spatial.transform import Rotation as R
import time


class command_class(object):
    def __init__(self, rad, dth):
        self.rate = rospy.Rate(10.0)  
        self.lin_vel = 0.2
        self.ang_vel = 0.2
        self.dist_tol = 0.05
        self.ang_tol = 0.2*math.pi/180
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/curiosity_mars_rover/odom', Odometry, self.callback)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.linear.z = 0
        self.cmd_vel_msg.angular.x = 0
        self.cmd_vel_msg.angular.y = 0
        self.rad = rad
        self.dth = dth
        
        
        time.sleep(2)        
        self.action_command()
        
    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        r = R.from_quat([q_x,q_y,q_z,q_w])
        ypr = r.as_euler('zyx', degrees=False)
        th = ypr[0] - math.pi # yaw angle
        if th < 0:
            self.th = th + 2*math.pi
        elif th > 2*math.pi:
            self.th = th - 2*math.pi
        else:
            self.th = th
        
    def action_command(self):
        xi = self.x
        yi = self.y
        thi = self.th
        print("Initial position x: {0:.3f}m".format(xi))
        print("Initial position y: {0:.3f}m".format(yi))
        print("Initial position th: {0:.1f}°".format(thi*180/math.pi))
        
        if self.rad: # no point turn
            if self.dth: # no straight line
                print("Arc with Radius {0:.3f}m and Theta {1:.1f}°".format(self.rad, self.dth*180/math.pi))
                self.cmd_vel_msg.linear.x =  self.lin_vel
                self.cmd_vel_msg.angular.z = self.lin_vel/self.rad
                thf = thi+self.dth
                xf = xi - self.rad*math.cos(thi) + self.rad*np.cos(thf)
                yf = yi - self.rad*math.sin(thi) + self.rad*np.sin(thf)
            else: # straight lines
                print("Straight Line {0:.3f}m".format(self.rad))
                self.cmd_vel_msg.linear.x =  self.lin_vel
                self.cmd_vel_msg.angular.z = 0
                thf = thi
                xf = xi-self.rad*np.sin(thi)
                yf = yi+self.rad*np.cos(thi)   
        else:
            if self.dth: #point turn rotation
                print("Point Turn Rotation of {0:.1f}°".format(self.dth*180/math.pi))
                self.cmd_vel_msg.linear.x =  0
                if self.dth > 0 :
                    self.cmd_vel_msg.angular.z = self.ang_vel
                else:
                    self.cmd_vel_msg.angular.z = -self.ang_vel
                thf = thi+self.dth
                xf = xi
                yf = yi
            else:
                print("Don't move action selected")
                return 0
            
        if thf < 0:
            thf= thf + 2*math.pi
        elif thf > 2*math.pi:
            thf = thf - 2*math.pi
                
        print("Expected Final position x: {0:.3f}m".format(xf))
        print("Expected Final position y: {0:.3f}m".format(yf))
        print("Expected Final position th: {0:.1f}°".format(thf*180/math.pi))   
                
        # Velocity Commands
        if self.rad:
            current_dist_prev = np.linalg.norm([self.x-xf,self.y-yf])
            current_dist_new = current_dist_prev
            count = 0
            while current_dist_new > self.dist_tol and count < 10:
                print("Distance: {0:.3f}m".format(current_dist_new))
                self.pub.publish(self.cmd_vel_msg)
                current_dist_prev = current_dist_new
                current_dist_new = np.linalg.norm([self.x-xf,self.y-yf])
                if current_dist_new > current_dist_prev:
                    count += 1
                else:
                    count = 0
                self.rate.sleep()
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0
            self.pub.publish(self.cmd_vel_msg) 
            print("Distance: {0:.3f}m".format(current_dist_new))
        else:
            current_dist_prev = np.linalg.norm([self.th-thf])
            if current_dist_prev > math.pi:
                current_dist_prev = 2*math.pi - current_dist_prev
            current_dist_new = current_dist_prev
            count = 0
            while current_dist_new > self.ang_tol and count < 10:
                print("Distance: {0:.1f}°".format(current_dist_new*180/math.pi))
                self.pub.publish(self.cmd_vel_msg)
                current_dist_prev = current_dist_new
                current_dist_new = np.linalg.norm([self.th-thf])
                if current_dist_new > math.pi:
                    current_dist_new = 2*math.pi - current_dist_new
                if current_dist_new > current_dist_prev:
                    count += 1
                else:
                    count = 0
                self.rate.sleep()
            self.cmd_vel_msg.angular.z = 0
            self.pub.publish(self.cmd_vel_msg)
            print("Distance: {0:.1f}°".format(current_dist_new*180/math.pi))
        
        print("Final position x: {0:.3f}m".format(self.x))
        print("Final position y: {0:.3f}m".format(self.y))
        print("Final position th: {0:.1f}°".format(self.th*180/math.pi))
        

def main(argv):
    if len(argv) != 3:
        print("Wrong number of params")
        print("Usage: command_arc.py <arc radius [m]> <rot angle [°]")  
        return 0         
    rad = float(argv[1])
    dth = float(argv[2])
    cmd_object = command_class(rad,dth*math.pi/180)

if __name__ == "__main__":
    rospy.init_node("Path_Planner_node", log_level=rospy.INFO)
    main(sys.argv)
