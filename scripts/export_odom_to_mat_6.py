#!/usr/bin/env python3

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and exports data to a mat file.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy.io as sio
import time

def feedback_callback(data, robot_num):

  global global_data, index
  
  (roll,pitch,yaw) = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
  
  global_data = np.append(global_data, [[robot_num, data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y, yaw, data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z]], axis=0)
 
  
  
def feedback_exporter():

  rospy.init_node("export_to_mat", anonymous=True)
  
  odom_topic_1 = "/robot_0/odom"
  odom_topic_2 = "/robot_1/odom"
  odom_topic_3 = "/robot_2/odom"
  odom_topic_4 = "/robot_3/odom"
  odom_topic_5 = "/robot_4/odom"
  odom_topic_6 = "/robot_5/odom"
  
  #topic_name = "/robot/move_base/teb_feedback" # define feedback topic here!

  rospy.Subscriber(odom_topic_1, Odometry, feedback_callback, 1, queue_size = 2) 
  rospy.Subscriber(odom_topic_2, Odometry, feedback_callback, 2, queue_size = 2) 
  rospy.Subscriber(odom_topic_3, Odometry, feedback_callback, 3, queue_size = 2) 
  rospy.Subscriber(odom_topic_4, Odometry, feedback_callback, 4, queue_size = 2) 
  rospy.Subscriber(odom_topic_5, Odometry, feedback_callback, 5, queue_size = 2) 
  rospy.Subscriber(odom_topic_6, Odometry, feedback_callback, 6, queue_size = 2) 

  #rospy.loginfo("Waiting for feedback message on topic %s.", topic_name)
 
  rospy.spin()
  '''
  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    if got_data:
      rospy.loginfo("Data export completed.")
      return

    r.sleep()
  '''

if __name__ == '__main__': 

  global global_data, index

  global_data = np.zeros((0,8))
  index = 0  

  try:

    feedback_exporter()

    
    dic={'data': global_data}

    timestr = time.strftime("%Y%m%d_%H%M%S")
    filename = './' + 'teb_data_' + timestr + '.mat'

    rospy.loginfo("Saving mat-file '%s'.", filename)
    sio.savemat(filename, dic)

  except rospy.ROSInterruptException:

    pass

