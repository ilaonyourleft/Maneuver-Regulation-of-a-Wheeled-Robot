#!/usr/bin/env python

import rospy
import rosnode
import numpy as np
import matplotlib.pyplot as plt
from ros_project.msg import *
from std_msgs.msg import *
from constants import *

x_values = []
y_values = []
theta_values = []
iterations = []
w_values = []
time_values = []

def distance(x1, y1, x2, y2):
	return (np.sqrt(np.power(x2 - x1, 2) + np.power(y2 - y1, 2)))
	
def get_theta(x1, y1, x2, y2):
	return (np.arctan2((y2 - y1), (x2 - x1)))
	
def control_law(theta1, theta2):
	w = K * (theta2 - theta1)
	return w
	
def try_again():
	global new_xdes
	global new_ydes
	
	print

	line = raw_input("what do you want to start? ( s = simulator, e = experiment )  or q if you want to quit -> ")
    			
	if line in ['s', 'S']:
		pub = rospy.Publisher('/input_topic', Input, queue_size = 1)
    		frequency = rospy.Rate(rate)
		new_xdes, new_ydes = compute_initial_try_again()
		sub = rospy.Subscriber('/position_topic', Position, callback)
	elif line in ['e', 'E']:
		print "experiment starting ..."
		rospy.signal_shutdown("closing controller")
	elif line in ['q', 'Q']:
		print "closing program ..."
		rospy.signal_shutdown("closing controller")
	else:
		print "wrong character"
		try_again()
	
def show_plot(x, y, theta, w, t):
	print
    		
    	plt.subplot(2, 1, 1)
    	plt.plot(x, y, color='blue')
	plt.axis([-10, 10, -10, 10])
	plt.xlabel("x")
	plt.ylabel("y")
	plt.title("Motion of the robot")
	plt.grid(True)
	plt.quiver(x, y, 1, 1, color='red', angles=theta, pivot='tip', width=0.01)
	
	plt.subplot(2, 1, 2)
	plt.plot(t, w, color='blue')
	plt.axis([-0.5, 10, -3, 3])
	plt.xlabel("t")
	plt.ylabel("w")
	plt.title("w(t)")
	plt.grid(True)

	plt.show()

	try_again()
	
	#rospy.signal_shutdown("closing controller")

def compute_initial_try_again():
	curr_x = x_values[-1]
	curr_y = y_values[-1]
	curr_theta = np.deg2rad(theta_values[-1])
	#curr_theta = init_theta
	
	x = input("set destination x: ")
	y = input("set destination y: ")
	
	dis = distance(curr_x, curr_y, x, y)
	theta = get_theta(curr_x, curr_y, x, y)
	
	theta_test = np.rad2deg(theta)
	if theta_test < -180 or theta_test > 180:
		s = int(theta_test / 180)
		th = theta_test - (s * 180)
		new_th = np.deg2rad(th)
	else:
		new_th = np.deg2rad(theta_test)
		
	#w = control_law(curr_theta, theta)
	w = control_law(curr_theta, new_th)
	
	#commenta da qui
	if w < -1:
		w = -1
		print "w = " + str(w)
	elif w > 1:
		w = 1
		print "w = " + str(w)
	else:
		print "w = " + str(w)
	#fine commento
	
	time = time_values[-1]
	time_values.append(time)
	
	msg = Input()
	msg.w = w
	msg.v = v
	
	while pub.get_num_connections() != 1:
    		frequency.sleep()
    	
	pub.publish(msg)
	
	x_values.append(curr_x)
	y_values.append(curr_y)
	
	curr_theta_deg = np.rad2deg(curr_theta)
	theta_values.append(curr_theta_deg)
	
	w_values.append(w)
	
	print "initial time: " + str(time)
	
	with open("./src/project/scripts/log.txt", "w") as f:
		f.write(str(curr_x) + " " + str(curr_y) + " " + str(curr_theta) + "\n")
	f.close()
	
	return x, y
	print
			
def compute_initial():
	global trying
	trying = 0
	
	x = input("set destination x: ")
	y = input("set destination y: ")
	
	dis = distance(init_x, init_y, x, y)
	theta = get_theta(init_x, init_y, x, y)
	
	theta_test = np.rad2deg(theta)
	if theta_test < -180 or theta_test > 180:
		s = int(theta_test / 180)
		th = theta_test - (s * 180)
		new_th = np.deg2rad(th)
	else:
		new_th = np.deg2rad(theta_test)
	
	#w = control_law(init_theta, theta)
	w = control_law(init_theta, new_th)
	
	'''
	if w < -1:
		w = -1
		print "w = " + str(w)
	elif w > 1:
		w = 1
		print "w = " + str(w)
	else:
		print "w = " + str(w)
	'''
	
	print "w = " + str(w)
	
	time = start
	#time_values.append(time)
	
	msg = Input()
	msg.w = w
	msg.v = v
	
	while pub.get_num_connections() != 1:
    		frequency.sleep()
    	
	pub.publish(msg)
	
	#x_values.append(init_x)
	#y_values.append(init_y)
	
	init_theta_deg = np.rad2deg(init_theta)
	#theta_values.append(init_theta_deg)
	
	#w_values.append(w)
	
	print "initial time: " + str(time)
	
	with open("./src/project/scripts/log.txt", "w") as f:
		f.write(str(init_x) + " " + str(init_y) + " " + str(init_theta) + "\n")
	f.close()
	
	return x, y
	print
			
def callback_vicon(data):
	print
	
	frequency.sleep()
	
	if not hasattr(callback_vicon, "msg"):
		callback_vicon.msg = ViconMsg()
		
	callback_vicon.msg = data
	
	x = callback_vicon.msg.p1
	y = callback_vicon.msg.p2
	
	r11 = callback_vicon.msg.r11
	r12 = callback_vicon.msg.r12
	r13 = callback_vicon.msg.r13
	r21 = callback_vicon.msg.r21
	r22 = callback_vicon.msg.r22
	r23 = callback_vicon.msg.r23
	r31 = callback_vicon.msg.r31
	r32 = callback_vicon.msg.r32
	r33 = callback_vicon.msg.r33
	
	print "x = " + str(x) + ", y = " + str(y)
	print "angles:\nr11 = " + str(r11) + ", r12 = " + str(r12) + ", r13 = " + str(r13)
	print "r21 = " + str(r21) + ", r22 = " + str(r22) + ", r23 = " + str(r23)
	print "r31 = " + str(r31) + ", r32 = " + str(r32) + ", r33 = " + str(r33)
	
def callback(data):
	print
	
	frequency.sleep()
	
	if not hasattr(callback, "curr_msg"):
		callback.curr_msg = Position()
		
	callback.curr_msg = data
	
	curr_x = callback.curr_msg.x
	curr_y = callback.curr_msg.y
	curr_theta = callback.curr_msg.theta
	
	if trying == 0:
		compute(curr_x, curr_y, curr_theta)
	else:
		compute_try_again(curr_x, curr_y, curr_theta)
	
def compute_try_again(x, y, theta):
	i = next(numIter)
	print "iteration: " + str(i)
	iterations.append(i)
	
	curr_time = i * delta
	print "current time: " + str(curr_time)
	time_values.append(curr_time)
	
	theta_deg = np.rad2deg(theta)
	print "current theta (deg) = " + str(theta_deg)
		
	x_values.append(x)
	y_values.append(y)
	theta_values.append(theta_deg)
	
	new_dis = distance(x, y, new_xdes, new_ydes)
	
	print "distance from the goal position: " + str(new_dis)

	theta_des = get_theta(x, y, new_xdes, new_ydes)
	
	theta_test = np.rad2deg(theta_des)
	if theta_test < -180 or theta_test > 180:
		s = int(theta_test / 180)
		th = theta_test - (s * 180)
		new_th = np.deg2rad(th)
	else:
		new_th = np.deg2rad(theta_test)
	
	#new_w = control_law(theta, theta_des)
	new_w = control_law(theta, new_th)
	
	'''
	if new_w < -1:
		new_w = -1
		print "w = " + str(new_w)
	elif new_w > 1:
		new_w = 1
		print "w = " + str(new_w)
	else:
		print "w = " + str(new_w)
	'''
	
	print "w = " + str(new_w)
		
	w_values.append(new_w)
			
	print "the robot is in x = " + str(x) + ", y = " + str(y)

	if new_dis < dist_des:
	
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v_stop
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		print "target reached"
	
		show_plot(x_values, y_values, theta_values, w_values, time_values)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	elif new_dis < dist_near:
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v_near
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	else:
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	print	
	
def compute(x, y, theta):
	i = next(numIter)
	print "iteration: " + str(i)
	iterations.append(i)
	
	curr_time = i * delta
	print "current time: " + str(curr_time)
	time_values.append(curr_time)
	
	theta_deg = np.rad2deg(theta)
	print "current theta (deg) = " + str(theta_deg)
		
	x_values.append(x)
	y_values.append(y)
	theta_values.append(theta_deg)
	
	new_dis = distance(x, y, xdes, ydes)
	
	print "distance from the goal position: " + str(new_dis)

	theta_des = get_theta(x, y, xdes, ydes)
	
	theta_test = np.rad2deg(theta_des)
	if theta_test < -180 or theta_test > 180:
		s = int(theta_test / 180)
		th = theta_test - (s * 180)
		new_th = np.deg2rad(th)
	else:
		new_th = np.deg2rad(theta_test)
	
	#new_w = control_law(theta, theta_des)
	new_w = control_law(theta, new_th)
	
	'''
	if new_w < -1:
		new_w = -1
		print "w = " + str(new_w)
	elif new_w > 1:
		new_w = 1
		print "w = " + str(new_w)
	else:
		print "w = " + str(new_w)
	'''
	
	print "w = " + str(new_w)
		
	w_values.append(new_w)
			
	print "the robot is in x = " + str(x) + ", y = " + str(y)

	if new_dis < dist_des:
	
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v_stop
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		print "target reached"
	
		show_plot(x_values, y_values, theta_values, w_values, time_values)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	elif new_dis < dist_near:
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v_near
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	else:
		new_msg = Input()
		new_msg.w = new_w
		new_msg.v = v
		
		while pub.get_num_connections() != 1:
    			frequency.sleep()
	
		pub.publish(new_msg)
		
		with open("./src/project/scripts/log.txt", "a") as f:
			f.write(str(x) + " " + str(y) + " " + str(theta_deg) + "\n")
		f.close()
	trying += 1
	
	print
	
if __name__ == '__main__':
        try:
		global xdes
		global ydes
        	print "controller starting ..."
        	print

		numIter = iter(range(1, num_max))
		iterations.append(0)
    		rospy.init_node('con', anonymous = True)
		
    		line = raw_input("what do you want to start? ( s = simulator, e = experiment )  or q if you want to quit -> ")
    		
		if line in ['s', 'S']:
			pub = rospy.Publisher('/input_topic', Input, queue_size = 1)
    			frequency = rospy.Rate(rate)
			xdes, ydes = compute_initial()
			sub = rospy.Subscriber('/position_topic', Position, callback)
		elif line in ['e', 'E']:
			print "experiment starting ..."
			frequency = rospy.Rate(rate)
			sub_vicon = rospy.Subscriber('ViconTopic', ViconMsg, callback_vicon)
			rospy.signal_shutdown("closing controller")
		elif line in ['q', 'Q']:
			print "closing program ..."
			rospy.signal_shutdown("closing controller")
		else:
			print "wrong character"
			try_again()
			
		rospy.spin()
        except rospy.ROSInterruptException:
		print "error"
