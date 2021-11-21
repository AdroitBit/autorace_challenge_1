#!/usr/bin/env python
from math import *
import time
import numpy as np
import cv2
import random
import os
import sys

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from autorace_challenge.srv import TurtlebotCommand,TurtlebotCommandResponse


sign=lambda x:copysign(1,x)
def get_img():
	return received_data['cam_image']
def show_img(img):
	cv2.imshow("POV",cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
def fill_cir(img,c):
	if c==None:
		return img
	return cv2.circle(
		img,tuple(map(int,c)),
		5,(255,255,255),-1
	)
def scat_list(*a):
	r=[]
	for i in a:
		try:
			try:
				i=list(i)
			except:
				i=[i.x,i.y,i.z]
			r+=i
		except:
			r.append(i)
	return r
def hypots(*a):
	a=scat_list(a)
	r=a[0]
	for i in a[1:]:
		r=hypot(r,i)
	return r
def distance(P,Q):#both point object
	P=(P.x,P.y,0.0)
	Q=(Q.x,Q.y,0.0)
	return dist(P,Q)
def sub(P,Q):
	return Point(
		P.x-Q.x,
		P.y-Q.y,
		0.0
	)
def in_2PI(rad):
	pi2=2*pi
	return ((rad%pi2)+pi2)%(pi2)
def in_PI(rad):
	return (rad+pi)%(2*pi)-pi
def diff_ang(rad0,rad1):
	return atan2(sin(rad1-rad0),cos(rad1-rad0))
def get_ang(v):
	return atan2(v.y,v.x)
	
def gray_img(img):
	return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
def edge_img(img):
	#return cv2.Canny(img,100,200)
	return cv2.Canny(img,50,250)
def lane_filter(img):
	hls=cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
	lower=np.array([0,190,0])#white range,aka left side
	upper=np.array([255,255,255])
	mask1=cv2.inRange(hls,lower,upper)
	lower=np.array([10,0,90])#yellow range,aka right side
	upper=np.array([50,255,255])
	mask2=cv2.inRange(hls,lower,upper)
	mask=cv2.bitwise_or(mask1,mask2)
	masked=cv2.bitwise_and(img,img, mask = mask)
	return masked
	
def bluesign_filter(img):
	hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	lower=np.array([80,150,50])
	upper=np.array([100,255,255])
	mask=cv2.inRange(hsv,lower,upper)
	masked = cv2.bitwise_and(img,img, mask = mask)
	return masked
def redline_filter(img):
	hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	lower=np.array([0,150,100])
	upper=np.array([20,255,255])
	mask0=cv2.inRange(hsv,lower,upper)
	lower=np.array([160,150,100])
	upper=np.array([180,255,255])
	mask1=cv2.inRange(hsv,lower,upper)
	mask=cv2.bitwise_or(mask0,mask1)
	masked=cv2.bitwise_and(img,img,mask=mask)
	return masked
def reset_odom():
	pub=rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
	rate=rospy.Rate(10)
	while True:
		p=received_data['pos']
		pub.publish(Empty())
		if p is not None and p.x==p.y==p.z==0:
			break
		rate.sleep()
def haveXYZ(p):
	for axis in ['x','y','z']:
		if axis not in p:
			return False
	return True
def centroid(img):
	M=cv2.moments(img)
	if M["m00"]==0:
		return None
	return[
		M["m10"]/M["m00"],
		M["m01"]/M["m00"]
	]
def centroidLR(l,r):
	if l==None and r==None:
		return None
	if l==None:
		return r[:]
	elif r==None:
		return l[:]
	else:
		return [(l[0]+r[0])/2,(l[1]+r[1])/2]
	
	
def track_pos(data):
	a,d=received_data,data.pose.pose
	a['pos'],a['rot']=d.position,d.orientation
bridge=CvBridge()
def track_img(data):
	try:
		img=bridge.imgmsg_to_cv2(data,"passthrough")
		img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		received_data['cam_image']=img
	except CvBridgeError as e:
		rospy.logerr(f"CvBridge Error: {e}")
		

rospy.init_node('lol_node')
sub_movement=rospy.Subscriber('/odom',Odometry,track_pos,queue_size=10)
sub_camera=rospy.Subscriber("/camera/rgb/image_raw", Image, track_img)
received_data={
	'pos':None,
	'rot':None,
	'cam_image':None
}
send_data={
	#(publisher obj,data to publish,after publish then assign this)
	'vel_rot':(
		rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1),
		Twist(),None
	)
}
def get_pos():
	return received_data['pos']
def get_rotation():#rad
	#(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	o=received_data['rot']
	o=euler_from_quaternion((o.x,o.y,o.z,o.w))
	_,_,yaw=o
	return in_2PI(yaw)

pubs={}
for k,v in send_data.items():
	pubs[k]=v[0]