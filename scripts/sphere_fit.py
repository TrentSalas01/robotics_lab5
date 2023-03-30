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

def sphere_params(xyz1):
	points = xyz1.points
	B = [[i.x**2 + i.y**2 + i.z**2] for i in points if i.x < 1]
	A = [[2*i.x, 2*i.y, 2*i.z, 1] for i in points if i.x < 1]
	P = np.linalg.lstsq(A, B, rcond = None)[0]
	
	P = np.array(P)
	return SphereParams(P[0], P[1], P[2], math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2))
	
	

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
		
		if len(xyz.points) == 0:
			continue
		
		
		
		sphere_params1 = sphere_params(xyz)
		print(sphere_params1.xc, sphere_params1.yc, sphere_params1.zc, sphere_params1.radius)
		sphere_pub.publish(sphere_params1)
		# pause until the next iteration			
		rate.sleep()

