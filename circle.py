import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    
    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)
        #self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
        #                                  Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            img = cv2.medianBlur(img,5)
        except CvBridgeError, e:
            print e

        circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,4,20,
                            param1=230,param2=230,minRadius=0,maxRadius=0)
        if circles is not None:
            print len(circles)
            circles = numpy.uint16(numpy.around(circles))
            for i in circles[0,:]:
        # draw the outer circle
                cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
                cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
                cv2.imshow("Image window", cv_image)
                            

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()