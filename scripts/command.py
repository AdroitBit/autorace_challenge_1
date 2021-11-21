#!/usr/bin/env python
import rospy
import os
from autorace_challenge_1.srv import *
rospy.init_node('autorace_command')
def client_request(cmd):
	rospy.wait_for_service('/autorace/command')
	try:
		f=rospy.ServiceProxy('/autorace/command',TurtlebotCommand)
		resp=f(cmd)
		return resp.back
	except rospy.ServiceProxy as e:
		return "Service call failed : %s"%e
def run_cmd(cmd):
	cmd,args=cmd[0],cmd[1:]
	if True:
		pass
rate=rospy.Rate(10)
while True:
	cmd=input("Command : ")
	if cmd=='exit':
		client_request('exit')
		break
	elif cmd=='clear':
		os.system('clear')
	else:
		print(" "*2,client_request(cmd))
	rate.sleep()
#client_request("stop")