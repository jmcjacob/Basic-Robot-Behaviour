import rospy
import cv2
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

wheel_radius = 1
robot_radius = 1

class behave:

    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.threshold = 30
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
            
            left = cv_image[0:height,0:width/2]
            right = cv_image[0:height,width/2:width]
            height, width, channels = left.shape
            
        except CvBridgeError, e:
            print e
            
        l_wheel = 0
        r_wheel = 0
        hit = False
        
        ranges = self.laser.ranges
        middle = int(round((len(ranges)-1)/2))
        center = ranges[middle]
        leftRanges = ranges[middle-15:middle]
        rightRanges = ranges[middle:middle+15]
    
        if min(leftRanges) < 0.9 or min(rightRanges) < 0.9:
            print "Too Close"
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.2
            self.pub.publish(twist_msg)
                
        else:
            leftRange = numpy.zeros((height,width,1), numpy.uint8)
            rightRange = numpy.zeros((height,width,1), numpy.uint8)
            cv2.inRange(left, numpy.array([0, 1, 0]), numpy.array([0, 255, 0]), leftRange)
            cv2.inRange(right, numpy.array([0, 1, 0]), numpy.array([0, 255, 0]), rightRange)      

            if cv2.countNonZero(leftRange) > 0:
                print "left"
                if mean > self.threshold:
                    l_wheel = 0.5 # Love
                else:
                    r_wheel = 0.5 # Fear
                hit = True
            
            if cv2.countNonZero(rightRange) > 0:
                print "right"
                if mean > self.threshold:
                    r_wheel = 0.5 # Love
                else:
                    l_wheel = 0.5 # Fear
                hit = True
            
        if hit:
            twist_msg = Twist()
            if r_wheel == l_wheel:
                print "center"
                if math.isnan(center):
                    value = 0.5
                else:
                    value = 1.0 - (1.0/center)
                    if value <= 0:
                        value = 0
                (v, a) = forward_kinematics(value, value)
                twist_msg.linear.x = v
                twist_msg.angular.z = a
            else:
                (v, a) = forward_kinematics(l_wheel, r_wheel)
                twist_msg.linear.x = v
                twist_msg.angular.z = a
            self.pub.publish(twist_msg)
            
        else:
            twist_msg = Twist()
            
            if min(leftRanges) > 2.0 and min(rightRanges) > 2.0:
                print "explore forward"
                twist_msg.linear.x = 0.5
                twist_msg.angular.z = 0.0
            elif min(leftRanges) > 2.0 and min(rightRanges) < 2.0:
                print "explore left"
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -0.5
            elif min(leftRanges) < 2.0 and min(rightRanges) > 2.0:
                print "explore right"
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5
            else:
                print "explore recover"
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5
            self.pub.publish(twist_msg)
            
        cv2.imshow("Image window", cv_image)

            
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