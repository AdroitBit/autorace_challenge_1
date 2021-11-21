#!/usr/bin/env python
from sub_global_fn import *
def lane_img():
	img=get_img()
	img=lane_filter(img)
	img=gray_img(img)
	img=cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)[1]
	return img
def bluesign_img():
	img=get_img()
	img=bluesign_filter(img)
	img=gray_img(img)
	img=cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)[1]
	return img
def bluesign_size(img):
	p=centroid(img)
	if p==None:
		return None
	h,w=img.shape
	x0,xf=p[0],p[0]
	y0,yf=p[1],p[1]
	for y in range(h):
		for x in range(w):
			if img[y][x]:
				x0,xf=min(x,x0),max(x,xf)
				y0,yf=min(y,y0),max(y,yf)
	#return (p,(x0,y0),(xf,yf))
	return (p,xf-x0,yf-y0,(x0,y0),(xf,yf))
def redline_img():
	img=get_img()
	img=redline_filter(img)
	#img=gray_img(img)
	#img=cv2.threshold(img,0,255,cv2.THRESH_BINARY)[1]
	return img
def rotate_to(rad=None,deg=None,spd=0.5):
	if rad==None and deg==None:
		return None
	if rad==None and deg!=None:
		rad=deg/180*pi
	pub=pubs['vel_rot']
	o=Twist()
	o.angular.z=spd
	rang=2/180*pi
	rate=rospy.Rate(30)
	while True:
		halt_or_pass()
		rad0=get_rotation()
		drad=diff_ang(rad0,rad)
		if abs(drad)<rang:
			break
		pub.publish(o)
		rate.sleep()
	o.angular.z=0
	pub.publish(o)
	
def go_to(dest,fspd=0.4):
	#rotate then go to that position
	pub=pubs['vel_rot']
	o=Twist()
	rate=rospy.Rate(30)
	while True:
		halt_or_pass()
		p0=get_pos()
		p1=dest
		v=sub(p1,p0)
		ang0=get_rotation()
		ang1=get_ang(v)
		ang=diff_ang(ang0,ang1)
		if abs(ang)>10/180*pi:
			o.linear.x=0
			o.angular.z=sign(ang)*0.5
		else:
			o.linear.x=fspd
			o.angular.z=sign(ang)*0.1
		if distance(p1,p0)<=0.05:
			break
		pub.publish(o)
		rate.sleep()
	o.linear.x=o.angular.z=0
	pub.publish(o)
def go_forward_for(d,spd=0.5):
	pub=pubs['vel_rot']
	p0=get_pos()
	x0,y0,z0=p0.x,p0.y,p0.z
	ang=get_rotation()
	xf,yf,zf=(
		x0+d*cos(ang),
		y0+d*sin(ang),
		z0
	)
	o=Twist()
	o.linear.x=spd
	rate=rospy.Rate(30)
	while True:
		halt_or_pass()
		p=get_pos()
		xi,yi,zi=p.x,p.y,p.z
		if hypots(xf-xi,yf-yi,zf-zi)<0.05:
			break
		pub.publish(o)
		rate.sleep()
	o.linear.x=0
	pub.publish(o)
def rotate(rad=0,deg=0,spd=0.5):#rad,rad/s
	if rad==0 and deg==0:
		return None
	if rad==0:
		ang=deg/180*pi
	else:
		ang=rad
	pub=pubs['vel_rot']
	o=Twist()
	o.angular.z=spd
	yaw0=get_rotation()
	yawf=in_2PI(get_rotation()+ang)
	rang=2/180*pi
	rate=rospy.Rate(30)
	while True:
		halt_or_pass()
		yawi=get_rotation()
		if abs(yawf-yawi)<rang:
			break
		pub.publish(o)
		rate.sleep()
	o.angular.z=0
	pub.publish(o)
def forward_spd(v=0.5):
	o=send_data['vel_rot'][1]
	o.linear.x=v
def rotate_spd(w=0.5):
	o=send_data['vel_rot'][1]
	o.angular.z=w
def go_in_lane(dest,f_spd=0.4):#near destination then stop,spd=[0.25,0.5]
	rate=rospy.Rate(30)
	while True:
		halt_or_pass()
		oimg=img=lane_img()
		h,w=img.shape
		y=int(h/2+170)
		img=img[y-20:y+20]#crop around that y position
		h,w=img.shape
		xc,yc=w//2,h//2
		
		xl,xr=0,w-1
		nl,nr=0,0
		for x in range(xc,0,-1):
			if img[yc][x]:
				if nl==0:
					xl=x
				nl+=1
		for x in range(xc,w):
			if img[yc][x]:
				if nr==0:
					xr=x
				nr+=1
				
		oimg=fill_cir(oimg,(xl,y))
		oimg=fill_cir(oimg,(xr,y))
		oimg=fill_cir(oimg,(xc,y))
		show_img(oimg)
		
				
		r_collide=10
		if abs(xc-xl)<r_collide and nl>nr:
			go_forward_for(0.3,spd=f_spd)
			rotate(deg=-40,spd=-1.0)
		elif abs(xc-xr)<r_collide and nr>nl:
			go_forward_for(0.3,spd=f_spd)
			rotate(deg=40,spd=1.0)
		else:
			forward_spd(f_spd)
			xcl=abs(xl-xc)
			xcr=abs(xr-xc)
			if xcl!=xcr:
				if abs(xcl-xcr)>10:#not too close
					rotate_spd(sign(xcl-xcr)*0.15)
				else:
					rotate_spd(sign(nr-nl)*0.15)
			send_all()
		if dest is not None and distance(get_pos(),dest)<=0.5:
			break
		if cv2.waitKey(1)==27:
			break
		rate.sleep()
	go_to(dest=dest,fspd=f_spd)

def send_all():
	global send_data
	for k,v in send_data.items():
		pub,data,after=v
		pub.publish(data)
		if after!=None:
			data=after
def setup(show_status=True):
	if show_status:
		print("Reseting odom....")
	reset_odom()#so that (0,0) is really (0,0)
	for k in received_data:
		received_data[k]=None
	if show_status:
		print("Filling data....")
	rate=rospy.Rate(5)#Hz
	while True:
		halt_or_pass()
		proceedable=True
		for v in received_data.values():
			if v is None:
				proceedable=False
				break
		if proceedable:
			break
		rate.sleep()
points={#relative to start point
	'B':Point(-1.962,-6.175,0.0),
	'C':Point(-1.876,-7.492,0.0),
	'D':Point(-4.419,-4.574,0.0)
}


class State():
	halt=0
	working=1
	@staticmethod
	def get_name(state):
		return {
			State.halt:'pausing',
			State.working:'working'
		}[state]
state=State.halt
def halt_or_pass():
	rate=rospy.Rate(10)
	while state==State.halt:
		rate.sleep()
def handle_request(req):
	global point
	global state
	r=''
	cmd=req.command.split()
	cmd,args=cmd[0],cmd[1:]
	if cmd in ['continue','start','s']:
		state=State.working
		r=cmd+" successfully"
	elif cmd in ['halt','stop','pause']:
		state=State.halt
		r=req.command+" successfully"
	elif cmd in ['state']:
		r=State.get_name(state)
	elif cmd in ['pos']:
		r=str(get_pos())
	elif cmd in ['point']:
		r=point
	elif cmd in ['exit']:
		state=State.halt
		try:
			rospy.signal_shutdown("Shutdown by client node")
		except rospy.exceptions.ROSInterruptException as _:
			print("Shutdown by client node")
	else:
		r='Unknown command'
	return TurtlebotCommandResponse(r)
s=rospy.Service('/autorace/command',TurtlebotCommand,handle_request)

#rosservice list
#rosservice call /autorace/command "state: 0"