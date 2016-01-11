#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import scipy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/rgb/image_mono",Image)

    #cv2.namedWindow("Image window", 1)
    cv2.namedWindow("Depth window" , 1)	
    #self.detector = cv2.SimpleBlobDetector()
    self.c = 0
    self.x = 0
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
        

  def callback(self,data):
    self.c = self.c+1
    if self.c==5 :
	    try:
	      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError, e:
	      print e

	    #rospy.loginfo(str(frame[1,1,1]))
	    (rows,cols,channels) = frame.shape
	    #rospy.loginfo("size: " + str((rows,cols,channels)))
	    frame = np.array(frame, dtype=np.uint8)	
	    #blue mask
	    # Convert BGR to HSV
	    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	    # define range of blue color in HSV
	    lower_blue = np.array([150,168,20])
	    upper_blue = np.array([179,255,255])

	    # Threshold the HSV image to get only blue colors
	    mask = cv2.inRange(hsv, lower_blue, upper_blue)

	    # Bitwise-AND mask and original image
	    #res = cv2.bitwise_and(frame,frame, mask= mask)
	    res = cv2.morphologyEx(mask,cv2.MORPH_OPEN,(12,12))
	    count = np.sum(res)/255
	    self.x = self.findCOM(res)
	    if (self.x>5 and self.x<595 and count>200):
		    for i in range(self.x-5,self.x+5):
			for j in range(240-5,240+5):
				res[j][i] = 150
            rospy.loginfo("avg: "  + str(self.x) + "total: " + str(count))
    	    #scipy.ndimage.measurements.label(res,struct, res)
	    #keypoints = self.detector.detect(frame)
	    #rospy.loginfo(str(keypoints))

	    # Bitwise-AND mask and original image
	    #res = cv2.bitwise_and(frame,frame, mask= mask)
	    #res = cv2.morphologyEx(res,cv2.MORPH_CLOSE,(5,5))
	    #keypoints = self.detector.detect(frame)
	    #rospy.loginfo(str(keypoints))

	    cv2.imshow('frame',frame)
	    #cv2.imshow('mask',mask)
	    cv2.imshow('res',res)
	    #k = cv2.waitKey(5) & 0xFF

	    #cv2.imshow("Image window", frame)
	    cv2.waitKey(3)

	    try:
	      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
	    except CvBridgeError, e:
	      print e
    	    self.c=0

  def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv(ros_image, "32FC1")
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # Process the depth image
        #depth_display_image = self.process_depth_image(depth_array)
    
        # Display the result
        cv2.imshow("Depth window", depth_array)

  def findCOM(self, grid):
	COM = (0,0)
	Mx = 0
	mass = 0
	for i in range(len(grid)):
		weight = grid[:,i].sum()
		if weight>10:
			Mx += i
			mass += 1
			COM = int(Mx/mass)
	return COM

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

