#!/usr/bin/env python

import rospy
import tf
from math import sin,cos
from geometry_msgs.msg import PoseStamped
pi = 3.14159

def HardCodedSquare(curr_x,curr_y):
    if curr_x == 0.75 and curr_y == 0.75:
        return (0.75,2.25,1)
    elif curr_x == 0.75 and curr_y == 2.25:
        return (2.25,2.25,1)
    elif curr_x == 2.25 and curr_y == 2.25:
        return (2.25,2.25,1) 
    else:
        return (0.75,0.75,1)

def Circular(angle,offset_x,offset_y,x,y,z):
    norm_x = x - offset_x
    norm_y = y - offset_y
    x_new = cos(angle)*norm_x - sin(angle)*norm_y + offset_x
    y_new = sin(angle)*norm_x + cos(angle)*norm_y + offset_y
    return (x_new,y_new,z)

def Forwardx(curr_x, curr_y, curr_z):
    return (curr_x+0.01, curr_y, curr_z)



if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")
    coordinate = [0,0,0]

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        # modify to time segments instead of infinitely moving
        msg.header.seq += 1

        if msg.header.seq % 2 == 0:
            coordinate = Forwardx(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            msg.pose.position.x = coordinate[0]
            msg.pose.position.y = coordinate[1]
            msg.pose.position.z = coordinate[2]
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()