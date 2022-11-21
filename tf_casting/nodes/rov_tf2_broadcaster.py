#!/usr/bin/env python
from tf2_ros import TransformBroadcaster
import rospy
from rospy import Time
import time 
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
import tf_conversions


def handle_rov_pose(msg):
    pos = msg.pose.pose.position
    orient = msg.pose.pose.orientation

    br = tf2_ros.TransformBroadcaster()
    odom = TransformStamped()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "map"
    odom.child_frame_id = 'base_link'
    odom.transform.translation.x = pos.x
    odom.transform.translation.y = pos.y
    odom.transform.translation.z = pos.z

    q = tf_conversions.transformations.quaternion_from_euler(orient.x,orient.y,orient.z)
    odom.transform.rotation.x = q[0]
    odom.transform.rotation.y = q[1]
    odom.transform.rotation.z = q[2]
    odom.transform.rotation.w = q[3]

    br.sendTransform(odom)

if __name__ == '__main__':
    rospy.init_node('tf2_rov_broadcaster')
    rospy.Subscriber('/bluerov/ground_truth/state',Odometry, handle_rov_pose)
    rospy.spin()
