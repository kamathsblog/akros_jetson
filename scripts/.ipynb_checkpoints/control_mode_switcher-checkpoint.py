#!/usr/bin/python

"""
Class to switch between different modes. Switches between twist outputs from different publishers using the mode number published by the Joystick node.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class ModeSwitcher():
    def __init__(self):

        rospy.init_node('control_mode_switcher')
        rospy.loginfo("[MSW] Mode switcher application initialized")
        
        #--- Create the Subscriber to read selected Mode
        self.ros_sub_teleop = rospy.Subscriber("/mode", Int8, self.set_mode, queue_size=1)
        rospy.loginfo("[MSW] Teleop Twist Subscriber initialized")
        
        #--- Create the Subscriber to Teleop Twist commands
        self.ros_sub_teleop = rospy.Subscriber("/teleop/cmd_vel", Twist, self.set_teleop_twist, queue_size=1)
        rospy.loginfo("[MSW] Teleop Twist Subscriber initialized")
        
        #--- Create the Subscriber to Auto Twist commands
        self.ros_sub_auto = rospy.Subscriber("/auto/cmd_vel", Twist, self.set_auto_twist, queue_size=1)
        rospy.loginfo("[MSW] Auto Twist Subscriber initialized")
        
        # initialize twist variables for teleop/cmd_vel and auto/cmd_vel, and integer variable for mode
        self.teleop_msg = Twist()
        self.auto_msg   = Twist()
        self.mode       = 0
        self.prev_mode  = 0

        #--- Create the Publisher to send switched Twist commands
        self.ros_pub_switched = rospy.Publisher("/switch_node/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("[MSW] Switched Twist Publisher initialized")

        rospy.loginfo("[MSW] Initialization complete")
        
    def set_teleop_twist(self, msg):
        self.teleop_msg = msg
        
    def set_auto_twist(self, msg):
        self.auto_msg = msg

    def set_mode(self, msg):
        self.mode = msg.data
        if (self.mode != self.prev_mode):
            if(msg.data != 0):
                rospy.loginfo("[MSW] Mode set to: %d", msg.data)
            else:
                rospy.loginfo("[MSG] Mode set to: Joystick OFF")
        self.prev_mode = self.mode;
        
    def run(self):
        #--- Set the control rate
        rate = rospy.Rate(50)
            
        while not rospy.is_shutdown():    
            
            if self.mode == 0:
                mode0 = self.teleop_msg
                mode0.linear.x = self.teleop_msg.linear.x * 0.75
                mode0.angular.z = self.teleop_msg.angular.z* 0.75
                self.ros_pub_switched.publish(mode0)
            
            # manual control
            elif self.mode == 1:
                #rospy.loginfo("[MSW] Mode:1 Teleop")
                self.ros_pub_switched.publish(self.teleop_msg)
            
            # autonomous steering, manual throttle
            elif self.mode == 2:
                mode2 = self.auto_msg
                mode2.linear.x = self.teleop_msg.linear.x
                self.ros_pub_switched.publish(mode2)
            
            # fully autonomous
            elif self.mode == 3:
                self.ros_pub_switched.publish(self.auto_msg)
            
            elif self.mode > 3:
                rospy.logerr("[MSW] Invalid mode")
        
            rate.sleep()
            
if __name__ == "__main__":
    switchmode = ModeSwitcher()
    switchmode.run()
