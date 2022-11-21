#!/usr/bin/env python
from tf2_ros import TransformBroadcaster
import rospy
from rospy import Time
import time 
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
import tf_conversions

class RovBroadCaster:
    def __init__(self):
        rospy.init_node('rov_broadcaster')
        rospy.Subscriber('/bluerov/ground_truth/state',Odometry, self.current_pose_callback) # to get current depth

        self.broadCaster = TransformBroadcaster()
        self.odom = TransformStamped()
        self.translation_vector = (0,0,0)
        self.rotational_vector = tf_conversions.transformations.quaternion_from_euler(0,0,0)



    def current_pose_callback(self,msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = 'base_link'
        self.odom.transform.translation.x = pos.x
        self.odom.transform.translation.y = pos.y
        self.odom.transform.translation.z = pos.z

        q = tf_conversions.transformations.quaternion_from_euler(orient.x,orient.y,orient.z)
        self.odom.transform.rotation.x = q[0]
        self.odom.transform.rotation.y = q[1]
        self.odom.transform.rotation.z = q[2]
        self.odom.transform.rotation.w = q[3]



        # self.translation_vector = (pos.x,pos.y,pos.z)
        # self.rotational_vector =  tf_conversions.transformations.quaternion_from_euler(orient.x,orient.y,orient.z)


    def main(self):

        while not rospy.is_shutdown():
            #create a quaternion
            # rotation_quaternion = tf.transformations.quaternion_from_euler(0.2, 0.3, 0.1)

            #translation vector
            # translation_vector = (1.0, 2.0, 3.0)

            #time
            current_time = rospy.Time.now()
            
            # self.broadCaster.sendTransform(
            # self.translation_vector,
            # self.rotational_vector,
            # current_time,
            # "base_link",
            # "world"
            # )
            self.broadCaster.sendTransform(self.odom)
            # t.sendTransform(
            #     translation_vector, 
            #     rotation_quaternion,
            #     current_time, 
            #     "basic_link", "world") #child frame, parent frame
            # print("I am here")
        

if __name__ == '__main__':
    bc = RovBroadCaster()
    bc.main()



