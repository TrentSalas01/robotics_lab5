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


xyz = XYZarray()

#function to get points
def get_xyz(XYZarray):
	global xyz
	xyz = XYZarray

def sphere_params(XYZarray):
	points = XYZarray.points
	B = [[i.x**2 + i.y**2 + i.z**2] for i in points]
	A = [[2*i.x, 2*i.y, 2*i.z, 1] for i in points]
	P = np.linalg.lstsq(A, B, rcond = None)[0]
	return SphereParams(P[0], P[1], P[2], math.sqrt(P[3] + xc**2 + yc**2 + zc**2))
	
	

if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# define a publisher to publish position
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# set the loop frequency
	rate = rospy.Rate(10)
	
	x = 0
	
	
	while not rospy.is_shutdown():
		#test to see values
		
		if len(XYZarray2.points) == 0:
			continue
		
		sphere_params = sphere_params(xyz)
		sphere_params.xc = xc
		sphere_params.yc = yc
		sphere_params.zc = zc
		sphere_params.radius = radius
		sphere_pub.publish(sphere_params)
		# pause until the next iteration			
		rate.sleep()
