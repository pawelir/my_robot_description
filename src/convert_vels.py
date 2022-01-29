#! /usr/bin/env python3

import time
import rospy
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

'''
This file provide conversion between cmd_vel command and specific rotation commands on wheels #
User provide cmd_vel message and then using this converter this message is translated to robot and commands are published on robot's wheels #
'''

class ConvertVelocities(object):

    def __init__(self):
        # Create publishers for each wheel (simulation) and vector (epos controller)
        self.cmd_pub_l_f = rospy.Publisher('/my_robot/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.cmd_pub_l_r = rospy.Publisher('/my_robot/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.cmd_pub_r_f = rospy.Publisher('/my_robot/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.cmd_pub_r_r = rospy.Publisher('/my_robot/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.epos_cmd_pub = rospy.Publisher('/my_robot/epos_cmd_vel', Float64MultiArray, queue_size=1)
        # Create subscriber for cmd_vel message
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.callback)
        
        self.Vl = Float64()
        self.Vr = Float64()
        self.epos_msg = Float64MultiArray()
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(10)
        
        time.sleep(1)


    def callback(self, msg):
        ### Callback function will convert the velocity commands ###
        self.cmd_vel = msg
        V = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z
        
        L = 0.4         #distance between front and rear wheel
        W = 0.41        #distance between L and R wheel
        R = 0.075       #wheel's radius

        self.Vl = (2*float(V) - float(w)*W) / (2*R)                                        
        self.Vr = (2*float(V) + float(w)*W) / (2*R)       
        self.epos_msg.data = [self.Vl/(2*pi)*60, self.Vr/(2*pi)*60]        

    def pub_velocities(self):
        rospy.loginfo('Start publishing velocity commands...')
        while not rospy.is_shutdown():
            self.cmd_pub_l_f.publish(self.Vl)
            self.cmd_pub_l_r.publish(self.Vl)
            self.cmd_pub_r_f.publish(self.Vr)
            self.cmd_pub_r_r.publish(self.Vr)
            self.epos_cmd_pub.publish(self.epos_msg)
            self.rate.sleep()       

if __name__ == '__main__':
    rospy.init_node('velocity_node')
    CV = ConvertVelocities()
    CV.pub_velocities()


