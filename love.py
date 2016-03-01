import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

wheel_radius = 1
robot_radius = 1

class behave:

    def __init__(self):
        cv2.namedWindow("Left window", 1)
        cv2.namedWindow("Right window", 1)
        cv2.startWindowThread()
        self.threshold = 40
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/turtlebot_1/scan", LaserScan, self.lasercallback)
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw", Image, self.callback)
                                          
    def lasercallback(self, data):
        self.laser = data                                          
                                          
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width, channels = cv_image.shape
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mean = numpy.mean(hsv_img[:, :, 2])
            print mean
            
            left = cv_image[0:height,0:width/2]
            right = cv_image[0:height,width/2:width]
            height, width, channels = left.shape
        except CvBridgeError, e:
            print e
            
        l_wheel = 0
        r_wheel = 0
        hit = False
    
        if self.laser.range_max > 0.2:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                self.pub.publish(twist_msg)
                print "too close" 
                
        else:
            leftRange = numpy.zeros((height,width,1), numpy.uint8)
            rightRange = numpy.zeros((height,width,1), numpy.uint8)
            cv2.inRange(left, numpy.array([0, 100, 0]), numpy.array([0, 255, 0]), leftRange)
            cv2.inRange(right, numpy.array([0, 100, 0]), numpy.array([0, 255, 0]), rightRange)      

            if cv2.countNonZero(leftRange) > 0:
                print "left"
                if mean > self.threshold:
                    l_wheel = 1.0 # Love
                else:
                    r_wheel = 1.0 # Fear
                hit = True
            
            if cv2.countNonZero(rightRange) > 0:
                print "right"
                if mean > self.threshold:
                    r_wheel = 1.0 # Love
            else:
                l_wheel = 1.0 # Fear
            hit = True
            
        if hit:
            print "hit"
            (v, a) = forward_kinematics(l_wheel, r_wheel)
            twist_msg = Twist()
            twist_msg.linear.x = v
            twist_msg.angular.z = a
            self.pub.publish(twist_msg)
            
        else:
            print "nope"
            twist_msg = Twist()
            twist_msg.angular.z = 0.5
            self.pub.publish(twist_msg)
            
        cv2.imshow("Left window", left)
        cv2.imshow("Right window", right)

            
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / robot_radius
    return (v, a)

rospy.init_node('node', anonymous=True)
behave()
rospy.spin()
cv2.destroyAllWindows()