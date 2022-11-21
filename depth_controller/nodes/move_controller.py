#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import time

class MoveController:
    def __init__(self,wave_length=1.5,general_depth=0.4,speed=0.4):
        rospy.init_node('move_controller')
        self.wave_length = wave_length
        self.general_depth = general_depth
        self.speed = speed
        self.doneForward = False

        self.for_back_pub = rospy.Publisher('/bluerov/thrust',Float64,queue_size=10)
        self.up_down_pub = rospy.Publisher('/bluerov/vertical_thrust',Float64,queue_size=10)
        self.left_right_pub = rospy.Publisher('/bluerov/lateral_thrust',Float64,queue_size=10)
        self.rotate_z_pub = rospy.Publisher('/bluerov/yaw',Float64,queue_size=10)

    def squareWave(self):
        # Forward >> Down >> Forward >> Up >> Forward
        dis = self.wave_length / 3
        self.moveForward()
        self.moveDown()
        self.moveForward()
        self.moveUp()
        self.moveForward()
        self.doneForward = True

    
    def moveForward(self):
        t = self.wave_length/self.speed if self.speed  else 0
        self.for_back_pub.publish(self.speed)   
        print(f"time: {t}")
        time.sleep(t)
        self.for_back_pub.publish(0)

    
    def moveBackward(self):
        t = self.wave_length/self.speed if self.speed  else 0
        self.for_back_pub.publish(self.speed * -1)   
        print(f"time: {t}")
        time.sleep(t)
        self.for_back_pub.publish(0)
    
    def moveUp(self):
        t = self.wave_length/self.speed if self.speed  else 0
        self.up_down_pub.publish(self.speed)   
        print(f"time: {t}")
        time.sleep(t)
        self.up_down_pub.publish(0)
    
    def moveDown(self):
        t = self.wave_length/self.speed if self.speed  else 0
        self.up_down_pub.publish(self.speed * -1)   
        print(f"time: {t}")
        time.sleep(t)
        self.up_down_pub.publish(0)
    
    def RotateZ(self):
        t = self.wave_length/self.speed if self.speed  else 0
        self.rotate_z_pub.publish(self.speed * -1)   
        print(f"time: {t}")
        time.sleep(t)
        self.rotate_z_pub.publish(0)


if __name__ == '__main__':
    a = MoveController()
    print("start")
    while not rospy.is_shutdown() and not a.doneForward:
        if a.for_back_pub.get_num_connections():
            time.sleep(1)
            print("Moving....")
            a.squareWave()
