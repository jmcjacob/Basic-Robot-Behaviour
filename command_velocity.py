# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class CommandVelocity():
    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("/turtlebot_1/cmd_vel", Twist)
        
    def send_velocities(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Sending commands")
            twist_msg = Twist()
            
            twist_msg.linear.x = 1.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0            
            self.pub.publish(twist_msg)
            
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 1.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0            
            self.pub.publish(twist_msg)
            
            twist_msg.linear.x = -1.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0            
            self.pub.publish(twist_msg)
            
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = -1.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0            
            self.pub.publish(twist_msg)
            
            r.sleep()
            
if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities()
    rospy.spin()
    
        