import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy

class image_converter:
    
    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)
        self.pub = rospy.Publisher('canny_mean', String, queue_size=10)
        #self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
        #                                  Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        canny  = cv2.Canny(img, 10, 200)   
        cv2.imshow("Image window", canny)
        self.pub.publish(str(numpy.mean(canny)))
        
                                    
rospy.init_node('image_converter', anonymous=True)
image_converter()
rospy.spin()
cv2.destroyAllWindows()