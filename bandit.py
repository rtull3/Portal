#!/usr/bin/env python
import rospy
import roslib
import sys
import rospy
import cv2
import scipy
import math
import numpy as np
import time
import serial #using the pyserial module open a serial connection
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import CliffEvent


reddetect = 0
targetpos = 0
trigger=serial.Serial('/dev/ttyUSB0',115200)
headdepth = 0


class Scan_msg:

    
    def __init__(self):
	'''Initializes an object of this class.

	The constructor creates a publisher, a twist message.
	3 integer variables are created to keep track of where obstacles exist.
	3 dictionaries are to keep track of the movement and log messages.'''
	global trigger
	self.pub = rospy.Publisher('/key_cmd_vel',Twist)
	self.qpub = rospy.Publisher('/cmd_vel',Twist)
	self.msg = Twist()
	self.bumperstate = BumperEvent()
	self.cliffstate = CliffEvent()
	self.sectorangles = [0,0,0,0,0]
	self.sectorrange = [0,0,0,0,0]
	self.sectorvio = [0,0,0,0,0]
	self.omega=0.0
	self.speed=0.25
	self.ang = [-0.42,-0.21,0,0.21,0.42]
	self.aim = 0
	trigger.write("#0 P1500 U1000\r")


    def reset_sect(self):
	'''Resets the below variables before each new scan message is read'''
	for i in range(0,5):
		self.sectorangles[i] = 0
		self.sectorrange[i] = 0
		self.sectorvio[i] = 0

    def sort(self, laserscan):
	'''Goes through 'ranges' array in laserscan message and determines 
	where obstacles are located. The class variables sect_1, sect_2, 
	and sect_3 are updated as either '0' (no obstacles within 0.7 m)
	or '1' (obstacles within 0.7 m)

	Parameter laserscan is a laserscan message.'''
	entries = len(laserscan.ranges)
	for entry in range(0,entries):
	    if 0.3 < laserscan.ranges[entry] < .6:
		if (0 < entry < (entries/5)):
			self.sectorvio[0] = self.sectorvio[0]+1
		if (entries/5 < entry < 2*entries/5):
			self.sectorvio[1] = self.sectorvio[1]+1
		if (2*entries/5 < entry < 3*entries/5):
			self.sectorvio[2] = self.sectorvio[2]+1
		if (3*entries/5 < entry < 4*entries/5):
			self.sectorvio[3] = self.sectorvio[3]+1
		if (4*entries/5 < entry < 9*entries/10):
			self.sectorvio[4] = self.sectorvio[4]+1
		'''self.sectorangles[1] = 1 if (entries/5 < entry < 2*entries/5) else 0
		self.sectorangles[2] = 1 if (2*entries/5 < entry < 3*entries/5) else 0
		self.sectorangles[3] = 1 if (3*entries/5 < entry < 4*entries/5) else 0
		self.sectorangles[4] = 1 if (4*entries/5 < entry < 9*entries/10) else 0'''

	for scores in range(0,5):
		if (self.sectorvio[scores] > 5):
			self.sectorangles[scores] = 1

	rospy.loginfo("sort complete,sect_1: " + str(self.sectorangles[0]) + " sect_2: " + str(self.sectorangles[1]) + " sect_3: " + str(self.sectorangles[2]) + " sect_4: " + str(self.sectorangles[3])+ " sect_5: " + str(self.sectorangles[4]))
	
	for entry in range(0,entries):
		if (math.isnan(laserscan.ranges[entry])==0):
			if (0 <entry<entries/5): 
				self.sectorrange[0] = self.sectorrange[0]+laserscan.ranges[entry] 
				#rospy.loginfo(str(laserscan.ranges[entry]) + "entry " + str(entry))
			if (entries/5 < entry < 2*entries/5): 
				self.sectorrange[1] = self.sectorrange[1]+laserscan.ranges[entry] 
			if (2*entries/5 < entry < 3*entries/5): 	
				self.sectorrange[2] = self.sectorrange[2]+laserscan.ranges[entry]
			if (3*entries/5 < entry < 4*entries/5): 
				self.sectorrange[3] = self.sectorrange[3]+laserscan.ranges[entry] 
			if (4*entries/5 < entry < entries): 
				self.sectorrange[4] = self.sectorrange[4]+laserscan.ranges[entry]
				#rospy.loginfo(str(self.sectorrange[4]))
	rospy.loginfo("sort complete,sect_1: " + str(self.sectorrange[0]) + " sect_2: " + str(self.sectorrange[1]) + " sect_3: " + str(self.sectorrange[2]) + " sect_4: " + str(self.sectorrange[3])+ " sect_5: " + str(self.sectorrange[4]))
	
	for j in range(0,5):
		rospy.loginfo(str(self.sectorrange[j])+" "+str(j) + " Z: " +str(self.msg.angular.z))
		self.sectorrange[j] = self.sectorrange[j]/(entries/5)
		
	
    def movement(self, bump,cliff):
	'''Uses the information known about the obstacles to move robot.

	Parameters are class variables and are used to assign a value to
	variable sect and then	set the appropriate angular and linear 
	velocities, and log messages.
	These are published and the sect variables are reset.'''
	global reddetect
	global targetpos
	
	if ( cliff.state == CliffEvent.FLOOR ) :
 		statec = "on the floor"
	else:
 		statec = "on the cliff"  
	if ( cliff.sensor == CliffEvent.LEFT ) :
 		cliffs = "Left"
	elif ( cliff.sensor == CliffEvent.CENTER ) :
 		cliffs = "Centre"
	else:
 		cliffs = "Right"
     	#rospy.loginfo("%s side of robot is %s."%(cliff, state))
	
	if(statec=="on the floor"):
		if ( bump.state == BumperEvent.RELEASED ) :
			state = "released"
		else:
			state = "pressed"  
		if ( bump.bumper == BumperEvent.LEFT ) :
			bumper = "Left"
		elif ( bump.bumper == BumperEvent.CENTER ) :
			bumper = "Center"
		else:
			bumper = "Right"

		longest=3
		wall=self.sectorangles[0]+self.sectorangles[1]+self.sectorangles[2]+self.sectorangles[3]+self.sectorangles[4]

		if (state == "released"):
			if (reddetect == 1):
				error = 300-targetpos
				if abs(error)>20:
					rate = 2*error/300.0
					self.msg.angular.z = rate
					rospy.loginfo("Target acquired, moving. Rate: " + str(rate) + " Error: " + str(error) + " Range: " + str(self.sectorrange[2]))
					self.aim = 0
				else:
					rospy.loginfo("Target in sight.. Error: " + str(error) + " Range: " + str(self.sectorrange[2]))
					self.msg.angular.z = 0
					if self.sectorrange[2]>3:
						self.msg.linear.x = .25
					else:
						self.msg.linear.x = 0
					self.aim+=1
					if self.aim>50:
						rospy.loginfo("Firing!")
						self.shoot(50)
						self.aim = 0
						#rospy.sleep(1.5)

			elif (wall==0):
				for entry in range(0,5): 
					if (self.sectorangles[entry]==0 and self.sectorrange[entry]>self.sectorrange[longest]):
						longest=entry
				self.msg.angular.z= self.ang[longest]
				self.msg.linear.x= 0.25
				rospy.loginfo("Sector " + str(longest) + "chosen. Wall is " + str(wall) + " z is " +str(self.ang[longest]))
				'''elif (wall>2):	
		       		self.msg.angular.z = 5*0.21
				self.msg.linear.x = 0
				rospy.loginfo("Sectors blocked doing full rotation")'''
			else:
				right = self.sectorangles[0]+self.sectorangles[1]
				left = self.sectorangles[2]+self.sectorangles[3]+self.sectorangles[4]
				if (right>left):
					self.msg.angular.z= .42
					self.msg.linear.x= 0
					direction = "left"
				else:
					self.msg.angular.z= -.42
					self.msg.linear.x= 0
					direction = "right"
				rospy.loginfo("Going " + direction + "Wall is " + str(wall))
			
		else:
			self.backup(2,2)
			rospy.loginfo("Backing up")	
	else:
		self.msg.angular.z=0
		self.msg.linear.x=0   	
	#rospy.loginfo("Angular Z: " + str(self.msg.angular.z))
	#rospy.loginfo("Bump: " + str(bump.CENTER))

	self.pub.publish(self.msg)
	self.reset_sect()

    def backup(self, duration, weight):
	    twist = Twist()

	    # First, back up slightly from the wall
	    currentTime = rospy.get_time();
	    stopTime = rospy.get_time() + duration;
	    while (rospy.get_time() < stopTime):
		    twist.linear.x = -.5; twist.linear.y = 0; twist.linear.z = 0
		    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		    self.pub.publish(twist)
		    rospy.sleep(.5)

	    # Now, keep turning until the end of the specified duration
	    currentTime = rospy.get_time();
	    stopTime = rospy.get_time() + duration;
	    while (rospy.get_time() < stopTime):
		 str = "turning %s"%rospy.get_time()
		 #rospy.loginfo(str)
		 twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
		 twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = weight
		 self.pub.publish(twist)
		 rospy.sleep(.5)
    def shoot(self,fpos):
	global trigger
	trigger.write("#0 P2400 U500\r")
	rospy.sleep(.3)
	trigger.write("#0 P1350 U500\r")
	#self.backup(2,2)
 
    def for_callback(self,laserscan):
	'''Passes laserscan onto function sort which gives the sect 
	variables the proper values.  Then the movement function is run 
	with the class sect variables as parameters.

	Parameter laserscan is received from callback function.'''
	self.sort(laserscan)
	self.movement(self.bumperstate,self.cliffstate)

    def for_bumpcallback(self,bumpinfo):
	self.bumperstate = bumpinfo
    
    def for_cliffcallback(self,cliffinfo):	
	self.cliffstate = cliffinfo

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/rgb/image_mono",Image)

    #cv2.namedWindow("Image window", 1)
    #cv2.namedWindow("Depth window" , 1)	

    self.c = 0
    self.x = 0
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
        

  def callback(self,data):
    global reddetect
    global targetpos
    self.c = self.c+1
    if self.c==5 :
	    try:
	      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError, e:
	      print e

	    (rows,cols,channels) = frame.shape
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

	    if (self.x>5 and self.x<595 and count>900):
		    reddetect = 1
		    targetpos = self.x
		    for i in range(self.x-5,self.x+5):
			for j in range(240-5,240+5):
				res[j][i] = 150
	    else:
		    reddetect = 0
            #rospy.loginfo("avg: "  + str(self.x) + "total: " + str(count))

	    # Bitwise-AND mask and original image
	    #res = cv2.bitwise_and(frame,frame, mask= mask)
	    #res = cv2.morphologyEx(res,cv2.MORPH_CLOSE,(5,5))

	    #cv2.imshow('frame',frame)
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

	global headdepth	
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
	
        # Process the depth image
        #depth_display_image = self.process_depth_image(depth_array)
    
        # Display the result
        #cv2.imshow("Depth window", depth_array)

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
    
def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)

def bumpcall_back(bumpmsg):
    '''Passes Bumper Event message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_bumpcallback(bumpmsg)

def cliffcall_back(cliffmsg):
	
    sub_obj.for_cliffcallback(cliffmsg) 

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    #rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    bumpsub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumpcall_back)
    cliffsub = rospy.Subscriber('/mobile_base/events/cliff',CliffEvent,cliffcall_back)
    rospy.spin()

'''def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()'''

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    '''try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
      cv2.destroyAllWindows()'''
    time.sleep(5)
    sub_obj = Scan_msg()
    listener()
    #main(sys.argv)
