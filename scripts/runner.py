#!/usr/bin/env python
#roslaunch turtlebot_autorace autorace_2021.launch
from global_fn import *

print("setting up...")
setup()
print("Boom!")
print("-"*20)

point='A'
if point=='A':
	print('Going to B')
	go_in_lane(dest=points['B'])
	go_to(dest=points['B'])
	point='B'
if point=='B':
	print('Going to C')
	rotate_to(deg=0,spd=1.0)
	go_in_lane(dest=points['C'])
	point='C'
if point=='C':
	print('Going to D')
	rotate_to(deg=-90,spd=1.0)
	go_in_lane(dest=points['D'])
	point='D'
if point=='D':
	go_to(dest=points['D'])
	print('Finished')
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		halt_or_pass()
		forward_spd(0)
		rotate_spd(10.0)
		send_all()
		rate.sleep()
	

print("-"*20)
print("Node finished")