import py_trees
import rospy
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

color_range = {
    "red" : ((0, 220, 0), (30, 255, 255)),
    "green": ((40, 220, 0), (80, 255, 255)),
    "blue": ((100, 220, 0), (140, 255, 255)),
}

class LookForColoredObject(py_trees.behaviour.Behaviour):

    def __init__(self, name,color,ros_duration = 3.0,visualize = True):
        super(LookForColoredObject,self).__init__(name)
        self.color = color
        self.visualize = visualize
        self.ros_timeout = rospy.Duration(ros_duration)
        self.h_min = color_range[self.color][0]
        self.h_max = color_range[self.color][1]

    def initialise(self):

        self.image = None
        self.bridge = CvBridge()
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = 100
        params.maxArea = 100000
        params.filterByArea = True
        params.filterByColor = False
        params.filterByInertia = False
        params.filterByConvexity = False
        params.thresholdStep = 50
        self.detector = cv2.SimpleBlobDetector_create(params)

    def update(self):

        start_time = rospy.Time.now()
        image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        start_time = rospy.Time.now()
        while self.image is None and (rospy.Time.now()-start_time < self.ros_timeout):
            rospy.sleep(0.5)

        image_sub = None

        if self.image is None:
            self.logger.info("ROS TIMEOUT")
            return py_trees.common.Status.FAILURE
                
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.h_min, self.h_max)
        keypoints = self.detector.detect(mask)

        if len(keypoints) == 0:
            self.logger.info("NO OBJECT DETECTED")
            return py_trees.common.Status.FAILURE
        
        for k in keypoints:
            self.logger.info(f"Detected object at [{k.pt[0]}, {k.pt[1]}]")
        return py_trees.common.Status.SUCCESS
        
    def callback(self,msg):

        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")