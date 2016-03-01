import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

wheel_radius = 1
robot_radius = 1

class behave:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw", Image, self.callback)
        
        # Make Publisher
                                          
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width, channels = cv_image.shape
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            left = hsv_img[0:height,0:width/2]
            right = hsv_img[0:height,width/2:width]
            height, width, channels = left.shape
        except CvBridgeError, e:
            print e
        
        leftRange = numpy.zeros((height,width,3), numpy.uint8)
        rightRange = numpy.zeros((height,width,3), numpy.uint8)
        cv2.inRange(left, numpy.array((60, 200, 0)), numpy.array((170, 255, 255)), leftRange)
        cv2.inRange(left, numpy.array((60, 200, 0)), numpy.array((170, 255, 255)), rightRange)
        
        #Convert left and right Range to 8 bit        
        
        leftcircles = cv2.HoughCircles(leftRange, cv2.cv.CV_HOUGH_GRADIENT, 4, 20, param1=230, param2=230, minRadius=0, maxRadius=0)
        rightcircles = cv2.HoughCircles(rightRange, cv2.cv.CV_HOUGH_GRADIENT, 4, 20, param1=230, param2=230, minRadius=0, maxRadius=0)
        
        if leftcircles is not None:
            leftcircles = numpy.round(leftcircles[0,:]).astype("int")
            highest = 0
            for (x,y,r) in leftcircles:
                if left(x,y,3) > highest:
                    highest = left(x,y,3)
            l_wheel = 2.55 - (highest/100)
            
        if rightcircles is not None:
            rightcircles = numpy.round(rightcircles[0,:]).astype("int")
            highest = 0
            for (x,y,r) in rightcircles:
                if right(x,y,3) > highest:
                    highest = right(x,y,3)
            r_wheel = 2.55 - (highest/100)
        
        (v, a) = forward_kinematics(l_wheel, r_wheel)
        # Publish to publisher
        
        cv2.imshow("Image window", cv_image)
            
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / robot_radius
    return (v, a)

behave()
rospy.init_node('behave', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()