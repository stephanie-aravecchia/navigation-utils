#!/usr/bin/python3
import os
os.environ["OMP_NUM_THREADS"] = "1"
import numpy as np
from nav_msgs.msg import Odometry
import rospy
import tf
from math import cos, sin, atan2

class NoisyLocalization:
    def __init__(self):

        rospy.init_node("noisy_loc")
        self.robot_frame = rospy.get_param("~robot_frame","base_link")
        self.gt_frame = rospy.get_param("~gt_frame","world")
        self.output_frame = rospy.get_param("~output_frame","noisy_loc_1")
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        rospy.sleep(5.)
        self.listener.waitForTransform(self.robot_frame, self.gt_frame, rospy.Time(), rospy.Duration(1.))
        self.sigma_position = rospy.get_param("~noise_sigma_position",0.005)
        self.sigma_orientation = rospy.get_param("~noise_sigma_orientation",0.005)
        self.debug = rospy.get_param("~debug", False)
        if self.debug:
            self.loc_publisher = rospy.Publisher("~odom", Odometry, queue_size=1)        
    
    def yaw_from_quaternion(self,q):
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def addNoiseToPosition(self, t):
        t[0] += np.random.normal(0.0,self.sigma_position)
        t[1] += np.random.normal(0.0,self.sigma_position)
        t[2] += np.random.normal(0.0,self.sigma_position)
        return t

    def addNoiseToOrientation(self, q):
        yaw = self.yaw_from_quaternion(q)
        noise = np.random.normal(0.0,self.sigma_orientation)
        Rgt = np.array(((cos(yaw),-sin(yaw),0),(sin(yaw), cos(yaw),0),(0,0,1)))       
        Rnoise = np.array(((cos(noise),-sin(noise),0),(sin(noise), cos(noise),0),(0,0,1)))       
        Rnew = np.matmul(Rnoise, Rgt)
        yaw_new = atan2(Rnew[1,0],Rnew[0,0]) 
        q = tf.transformations.quaternion_from_euler(0,0,yaw_new)
        return q
    
    def publishOdom(self, t, q):
        msg = Odometry()
        msg.header.frame_id = self.output_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = t[0] 
        msg.pose.pose.position.y = t[1] 
        msg.pose.pose.position.z = t[2] 
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.loc_publisher.publish(msg)
        return

    def run(self):
        rate = rospy.Rate(50)
        last_time = None
        while not rospy.is_shutdown():
            t, q = self.listener.lookupTransform(self.robot_frame, self.gt_frame, rospy.Time(0))
            t = self.addNoiseToPosition(t) 
            q = self.addNoiseToOrientation(q)
            #we publish the transform as child
            #In simulation, we may broadcast twice with the same time stamp, we check that before broadcasting
            current_time = rospy.Time.now()
            if not (last_time == None):
                if not (last_time == current_time): 
                    self.broadcaster.sendTransform(t,q, current_time, self.output_frame, self.robot_frame)
                    last_time = current_time 
            else:
                last_time = current_time 
                self.broadcaster.sendTransform(t,q, current_time, self.output_frame, self.robot_frame)

            if self.debug:
                self.publishOdom(t,q)
            rate.sleep()
    

if __name__ == '__main__':
    loc = NoisyLocalization()
    loc.run()
