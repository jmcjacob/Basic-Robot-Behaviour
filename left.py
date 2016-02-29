import rospy
from std_msgs.msg import Float32 
from geometry_msgs.msg import Twist

wheel_l_vel = 0.0
wheel_r_vel = 1.0
wheel_radius = 1
robot_radius = 1

def callback(data):
    wheel_l_vel = data
    talker()
    
def listener():
    rospy.Subscriber("/turtlebot_1/wheel_vel_left", Float32, callback)
    rospy.spin()
    
def talker():
    pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist, queue_size=10)
    (v,a) = forward_kinematics(wheel_l_vel, wheel_r_vel)
    twist_msg = Twist()
    twist_msg.linear.x = v
    twist_msg.angular.z = a
    pub.publish(twist_msg)
    
    
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / robot_radius
    return (v, a)

rospy.init_node('node', anonymous = True)
listener()