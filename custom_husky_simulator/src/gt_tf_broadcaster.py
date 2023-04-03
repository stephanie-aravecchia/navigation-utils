#!/usr/bin/python3
import os
#we need that to avoid numpy calling openBLAS which is highly CPU extensive...
os.environ["OMP_NUM_THREADS"] = "1"
import rospy
from nav_msgs.msg import Odometry	       
import tf

broadcaster = tf.TransformBroadcaster()	
last_time = None
    
#This node takes the gt pose in simulation, and publish the corresponding transform
def gtCallback(msg):
    global broadcaster, last_time
    #we broadcast the new frame as child of base_link, so we invert it
    p = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y , msg.pose.pose.orientation.z , msg.pose.pose.orientation.w]
    transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(p), tf.transformations.quaternion_matrix(q))
    inverse = tf.transformations.inverse_matrix(transform)
    t = tf.transformations.translation_from_matrix(inverse) 
    q = tf.transformations.quaternion_from_matrix(inverse)
    current_time = rospy.Time.now()
    #If the simulated time is too slow, we may broadcast several time wieht the same timestamp, we prevent that
    if not (last_time == None):
        if not (last_time == current_time):
            broadcaster.sendTransform(t, q, current_time, "world", "base_link")
            last_time = current_time
            
    else:
        broadcaster.sendTransform(t, q, current_time, "world", "base_link")
        last_time = current_time 

if __name__ == '__main__':
    rospy.init_node('gt_tf_broadcaster')
    sub = rospy.Subscriber('gt_pose', Odometry, gtCallback, queue_size=1)
    rospy.spin()






