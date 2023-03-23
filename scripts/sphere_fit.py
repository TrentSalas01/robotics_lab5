#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from SphereParams.msg import xc, yc, zc, radius
from XYZarray.msg import points


if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/xyz_cropped_ball", points) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/sphere_params', Image, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()

