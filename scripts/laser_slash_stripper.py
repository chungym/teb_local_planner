#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def init():
    rospy.init_node("scan_slash_stripper", anonymous=True)

    scan_topic_1 = "/robot_0/base_scan"
    scan_topic_2 = "/robot_1/base_scan"
    scan_topic_3 = "/robot_2/base_scan"
    scan_topic_4 = "/robot_3/base_scan"
    scan_topic_5 = "/robot_4/base_scan"
    scan_topic_6 = "/robot_5/base_scan"
    scan_topic_7 = "/robot_6/base_scan"
    scan_topic_8 = "/robot_7/base_scan"

    pub1 = rospy.Publisher(scan_topic_1+"_filtered", LaserScan, queue_size=1)
    pub2 = rospy.Publisher(scan_topic_2+"_filtered", LaserScan, queue_size=1)
    pub3 = rospy.Publisher(scan_topic_3+"_filtered", LaserScan, queue_size=1)
    pub4 = rospy.Publisher(scan_topic_4+"_filtered", LaserScan, queue_size=1)
    pub5 = rospy.Publisher(scan_topic_5+"_filtered", LaserScan, queue_size=1)
    pub6 = rospy.Publisher(scan_topic_6+"_filtered", LaserScan, queue_size=1)
    pub7 = rospy.Publisher(scan_topic_7+"_filtered", LaserScan, queue_size=1)
    pub8 = rospy.Publisher(scan_topic_8+"_filtered", LaserScan, queue_size=1)


    rospy.Subscriber(scan_topic_1, LaserScan, feedback_callback, pub1, queue_size = 2) 
    rospy.Subscriber(scan_topic_2, LaserScan, feedback_callback, pub2, queue_size = 2) 
    rospy.Subscriber(scan_topic_3, LaserScan, feedback_callback, pub3, queue_size = 2) 
    rospy.Subscriber(scan_topic_4, LaserScan, feedback_callback, pub4, queue_size = 2) 
    rospy.Subscriber(scan_topic_5, LaserScan, feedback_callback, pub5, queue_size = 2) 
    rospy.Subscriber(scan_topic_6, LaserScan, feedback_callback, pub6, queue_size = 2) 
    rospy.Subscriber(scan_topic_7, LaserScan, feedback_callback, pub7, queue_size = 2) 
    rospy.Subscriber(scan_topic_8, LaserScan, feedback_callback, pub8, queue_size = 2) 

def feedback_callback(data, pub):

    data.header.frame_id = data.header.frame_id.lstrip("/")
    pub.publish(data)


if __name__ == '__main__': 

  try:

    init()

    rospy.spin()

  except rospy.ROSInterruptException:

    pass



