#!/usr/bin/env python3
#import all necessary modules
import rospy
import math
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

#class to get x, y, and z coords/positions
class xyz:
	def __init__(self, x=0, y=0, z=0):
		self.x = x
		self.y = y
		self.z = z
#initiate a Point
Point = xyz(0,0,0)

#function to get points
def get_xyz(XYZarray):
	global Point
	XYZarray = XYZarray.points
	Point = xyz(XYZarray[0].x, XYZarray[0].y, XYZarray[0].z)
	
	

if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# define a publisher to publish position
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# set the loop frequency
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		#test to see values
		print(Point.x, Point.y, Point.z)
		# Calculates the center of the ball
		B = [[Point.x**2 + Point.y**2 + Point.z**2],
		     [Point.x**2 + Point.y**2 + Point.z**2],
		     [Point.x**2 + Point.y**2 + Point.z**2],
		     [Point.x**2 + Point.y**2 + Point.z**2]]
		    
		A = [[2*Point.x, 2*Point.y, 2*Point.z, 1],
		     [2*Point.x, 2*Point.y, 2*Point.z, 1],
		     [2*Point.x, 2*Point.y, 2*Point.z, 1],
		     [2*Point.x, 2*Point.y, 2*Point.z, 1]]
		#Uses formula to calculate P
		P = np.true_divide(B, A)
		
		xc = P[0]
		yc = P[1]
		zc = P[2]
		#Uses coords to calculate the radius of the ball
		radius = math.sqrt(P[3][0] + xc**2 + yc**2 + zc**2)
		
		#set up variable to publish
		sphere_params = SphereParams()
		sphere_params.xc = xc
		sphere_params.yc = yc
		sphere_params.zc = zc
		sphere_params.radius = radius
		sphere_pub.publish(sphere_params)
		# pause until the next iteration			
		rate.sleep()
