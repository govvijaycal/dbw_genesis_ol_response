#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import math
import argparse

# Open loop tests to evaluate low-level control tracking of acceleration input.

def pub_acc(mode='step'):
	enable_pub = rospy.Publisher("control/enable_accel", UInt8, queue_size =10, latch=True)
	pub = rospy.Publisher("control/accel", Float32, queue_size =10)
	rospy.init_node('pub_accel_ol', anonymous=True)
	r = rospy.Rate(10.0)

	while(enable_pub.get_num_connections == 0):
		print 'Waiting to enable acceleration!'
		r.sleep()

	enable_pub.publish(2); # enable acceleration control.

	t_start = rospy.Time.now()
	print('OL Acc Test: %s, Started at %f' % (mode, t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs
		
		if mode == 'step':
			a_des = step_response(dt_secs)
		elif mode == 'ramp':
			a_des = ramp_response(dt_secs)
		elif mode == 'sine':
			a_des = sine_response(dt_secs)
		else:
			a_des = 0.0

		print('\tM: %s, T: %f, A: %f' % (mode, dt_secs, a_des))
		pub.publish(a_des)
		r.sleep()
		
	enable_pub.publish(0); # disable acceleration control.

def step_response(dt_secs):
	if dt_secs < 5.0:		# [0-5 s]
		a_des = 0.0
	elif dt_secs < 15.0:	# [5-15 s]
		a_des = 1.0
	elif dt_secs < 20.0:	# [15-20 s]
		a_des = 0.0
	elif dt_secs < 30.0:	# [20-30 s]
		a_des = -1.0
	else:					# [30+ s]
		a_des = 0.0				
	return a_des

def ramp_response(dt_secs):
	if dt_secs < 5.0:		# [0-5 s]
		a_des = 0.0
	elif dt_secs < 15.0:	# [5-15 s]
		a_des = 1.0 * (dt_secs - 5.0)/10.0
	elif dt_secs < 20.0:	# [15-20 s]
		a_des = 1.0 - (dt_secs - 15.0)/5.0
	elif dt_secs < 30.0:	# [20-30 s]
		a_des = -1.0 * (dt_secs - 20.0)/10.0
	else:					# [30+ s]
		a_des = 0.0				
	return a_des

def sine_response(dt_secs):
	# amplitude 1.0, period = 25 s starting at t = 5.0
	if dt_secs < 5.0:		# [0-5 s]
		a_des = 0.0
	elif dt_secs < 30.0:	# [5-30 s]
		tm_offset = dt_secs - 5.0
		a_des = 1.0 * math.sin(2.0 * math.pi/25.0 * tm_offset)
	else:					# [30+ s]
		a_des = 0.0			
	return a_des

if __name__=="__main__":
	try:
<<<<<<< HEAD
		pub_acc(mode='step')
=======
		parser = argparse.ArgumentParser('Run an open-loop steering response test on the Hyundai Genesis.')
		parser.add_argument('-m', '--mode', type=str, choices=['step', 'ramp', 'sine'], default='step', help='Test Mode')
		args = parser.parse_args()
		pub_acc(mode=args.mode)
>>>>>>> e6bc6606f4e921a3603fc432d5e07fbc62da1182
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly

