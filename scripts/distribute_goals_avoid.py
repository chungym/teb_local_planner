#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def sendGoals(msg):

    pair_list = []

    for robot in msg:
        pub = rospy.Publisher(robot[0] + '/move_base_simple/goal', PoseStamped, queue_size=1)
        new_msg = PoseStamped()
        new_msg.header.frame_id = "map_common"
        new_msg.header.stamp = rospy.Time.now()
        new_msg.pose = robot[1]
        
        pair_list.append([pub,new_msg])
    
    rospy.sleep(2.0)
    
    for pair in pair_list:
        pair[0].publish(pair[1])
        rospy.sleep(0.1)
    	
    	
    #for robot in msg.robots:
    #    pub.unregister()

def talker():
    
    rospy.init_node('goal_distributor', anonymous=True)

    #sub = rospy.Subscriber('goals', RobotGoalsArray, callback)

    pose = Pose()
    pose.position.x = 23
    pose.position.y = 10.2
    pose.orientation.z = 0
    pose.orientation.w = 1

    robot1 = ["robot", pose]
    
    pose2 = Pose()
    pose2.position.x = 12.0
    pose2.position.y = 10.2
    pose2.orientation.z = 1
    pose2.orientation.w = 0
    robot2 = ["robot_b", pose2]

    msg = [robot2, robot1]

    sendGoals(msg)

if __name__ == '__main__':

    talker()

