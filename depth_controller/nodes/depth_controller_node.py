#!/usr/bin/env python
import rospy
from fav_msgs.msg import ThrusterSetpoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
from depth_controller.msg import PID
import time 

g = 9.81
rho = 1023 # water density >> 1023 for salt water and 997 for fresh water
p0 = 101325
class DepthController:
    """
    notes:

    >> one can get current depth using this topic directly /bluerov/ground_truth/state 
       OR, using that equation (p0-pf)/(rho*g) that depends on the the pressure

    >> one can get the target pressure using this topic /bluerov/depth_setpoint,
        OR, this new created topic /controller/custom_depth_setpoint (from web ui)

    """

    def __init__(self,max_throttling=0.3,min_throttling=-0.3,Rate=20):

        rospy.init_node('depth_controller_node', anonymous=True)
        rospy.set_param('use_sim_time', True)
        # subscribers
        # rospy.Subscriber('/bluerov/ground_truth/state',Odometry, self.current_depth_callback) # to get current depth
        rospy.Subscriber('/bluerov/pressure',FluidPressure,self.current_pressure_callback)
        rospy.Subscriber('/controller/pid',PID,self.pid_callback) # to update the KP, KI, KD dynamically from the web ui

        rospy.Subscriber('/controller/custom_depth_setpoint',Float64,self.webui_target_depth_callback) # to get target depth from web ui
        rospy.Subscriber('/bluerov/depth_setpoint',Float64,self.target_depth_callback) # to get target depth


        # publishers
        self.thrust_pub = rospy.Publisher('/bluerov/vertical_thrust',Float64,queue_size=10)
        self.current_depth_pup = rospy.Publisher('/controller/actual_depth',Float64,queue_size=10)


        # controller parameters
        self.KP = 1.2
        self.KI = .5
        self.KD = 0.3
        self.max_throttling = max_throttling
        self.min_throttling = min_throttling
        self.current_depth = 0
        self.target_depth = 0
        self.webui_target_depth = 0
        self.current_pressure = self.init_pressure_message()
        self.controller_output = 0
        self.thruster_output = self.init_thruster_message() # this to control all motors, we may need this in the future
        self.rate = rospy.Rate(Rate)
        self.current_time = rospy.get_rostime().secs
        self.dt = 0.05
        self.I = 0
        self.prev_error = 0

    
    def init_pressure_message(self):
        msg = FluidPressure()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "current pressure"
        msg.fluid_pressure = 0
        return msg
    
    def init_thruster_message(self):
        msg = ThrusterSetpoint()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "depth control"
        msg.data = [0,0,0,0,0,0,-0.5,-0.5]
        return msg
        
    def target_depth_callback(self,msg):
        self.target_depth = msg.data
    
    def current_depth_callback(self,msg):
        self.current_depth = msg.pose.pose.position.z
    
    def current_pressure_callback(self,data):
        self.current_pressure = data

    def webui_target_depth_callback(self,data):
        self.webui_target_depth = data.data

    def pid_callback(self,data):
        self.KP = data.KP
        self.KI = data.KI
        self.KD = data.KD

        print(f"KP = {self.KP},KI = {self.KI},KD = {self.KD}")


        
    def control(self):
        depth = -1 * (self.current_pressure.fluid_pressure - p0)/(rho*g) # depth has negative value, z-axis points downwards
        self.current_depth_pup.publish(depth)
        
        # error
        target_depth = abs(self.webui_target_depth)
        delta_depth =  abs(depth) - target_depth
        print(f"delta_depth: {delta_depth}")

        # time
        delta_time = rospy.get_rostime().secs - self.current_time
        self.current_time = rospy.get_rostime().secs

        # propotional term
        P = delta_depth

        # integral term
        I = self.I + delta_depth * delta_time

        # derivative term
        D = 0 
        if delta_time:
            D = (delta_depth - self.prev_error) / self.dt
            self.prev_error = delta_depth

        # controller output
        controller_output = self.saturate_controller(self.KP * P + self.KI * I - self.KD * D)

        print(f"current saturated controller: {controller_output}")
        print("#############################")


        # publishing controller output
        self.thrust_pub.publish(controller_output)            
        # self.thrust_pub.publish(0)        

    def saturate_controller(self,controller_output):
        if controller_output > self.max_throttling:
            return self.max_throttling
        elif controller_output < self.min_throttling:
            return self.min_throttling
        else:
            return controller_output


if __name__ == '__main__':
    try:
        a = DepthController()
        print("Start Moving")
        print("Test")
        while not rospy.is_shutdown():
            a.control()
            time.sleep(a.dt)
        print("Stop")
        
    except rospy.ROSInterruptException:
        print("Exception Error")
