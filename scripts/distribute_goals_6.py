#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def sendGoals(msg):

    pair_list = []

    for robot in msg:
        pub = rospy.Publisher(robot[0] + '/move_base_simple/goal', PoseStamped, queue_size=1)
        new_msg = PoseStamped()
        new_msg.header.frame_id = "map"
        new_msg.header.stamp = rospy.Time.now()
        new_msg.pose = robot[1]
        
        pair_list.append([pub,new_msg])
    
    rospy.sleep(2.0)
    
    for pair in pair_list:
        pair[0].publish(pair[1])
        rospy.sleep(0.001)
    	
    	
    #for robot in msg.robots:
    #    pub.unregister()

def talker():
    
    rospy.init_node('goal_distributor', anonymous=True)

    #sub = rospy.Subscriber('goals', RobotGoalsArray, callback)

    pose = Pose()
    pose.position.x = 5
    pose.orientation.z = 0
    pose.orientation.w = 1
    robot1 = ["robot_0", pose]
    
    pose2 = Pose()
    pose2.position.x = -5
    pose2.orientation.z = 1.0
    robot2 = ["robot_1", pose2]

    pose3 = Pose()
    pose3.position.x = 2.5
    pose3.position.y = -4.33013
    pose3.orientation.z = -0.5
    pose3.orientation.w = 0.8660254
    robot3 = ["robot_2", pose3]

    pose4 = Pose()
    pose4.position.x = -2.5
    pose4.position.y = 4.33013
    pose4.orientation.z = 0.8660254
    pose4.orientation.w = 0.5
    robot4 = ["robot_3", pose4]

    pose5 = Pose()
    pose5.position.x = -2.5
    pose5.position.y = -4.33013
    pose5.orientation.z = -0.8660254
    pose5.orientation.w = 0.5
    robot5 = ["robot_4", pose5]

    pose6 = Pose()
    pose6.position.x = 2.5
    pose6.position.y = 4.33013
    pose6.orientation.z = 0.5
    pose6.orientation.w = 0.8660254
    robot6 = ["robot_5", pose6]


    msg = [robot1, robot2, robot3, robot4, robot5, robot6]

    sendGoals(msg)

if __name__ == '__main__':

    talker()

