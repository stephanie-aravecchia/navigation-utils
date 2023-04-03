#!/usr/bin/python3
import rospy
import tf

rospy.init_node('gt_tf_broadcaster')
broadcaster = tf.TransformBroadcaster()	
listener = tf.TransformListener()	

while (not listener.canTransform("base_link", "os1", rospy.Time(0))) :
    rospy.loginfo("Waiting for the initial transform base_link os1")
    rospy.sleep(5.)
t,q = listener.lookupTransform("base_link", "os1", rospy.Time(0))

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    broadcaster.sendTransform(t, q, rospy.Time.now(), "os1", "base_link")
    rate.sleep()