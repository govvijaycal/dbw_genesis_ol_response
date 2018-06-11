#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import math
import argparse

# Open loop tests to evaluate low-level control tracking of tire angle (d_f) input.
# NOTE: Assumes d_f in radians.  This may change with ECU reprogramming!

def pub_steer(mode='step'):
	enable_pub = rospy.Publisher("control/enable_spas", UInt8, queue_size = 1, latch=True)
	pub = rospy.Publisher("control/steer_angle", Float32, queue_size = 1) 	
	rospy.init_node('pub_steer_ol', anonymous=True)
	r = rospy.Rate(10.0)

	while(enable_pub.get_num_connections == 0):
		print 'Waiting to enable steering!'
		r.sleep()

	enable_pub.publish(1); # enable steering control.

	t_start = rospy.Time.now()
	print('Steer Test: %s, Started at %f' % (mode, t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs

		if mode == 'step':
			st_des = step_response(dt_secs)
		elif mode == 'ramp':
			st_des = ramp_response(dt_secs)
		elif mode == 'sine':
			st_des = sine_response(dt_secs)
		else:
			pdb.set_trace()
			st_des = 0.0

		print('\tM: %s, T: %f, DF: %f' % (mode, dt_secs, st_des))

		pub.publish(st_des)
		r.sleep()

	enable_pub.publish(0); # disable steering control.

def step_response(dt_secs):
	if dt_secs < 2.0:	# [0-2 s]
		st_des = 0.0
	elif dt_secs < 4.0:	# [2-4 s]
		st_des = 0.01
	elif dt_secs < 6.0:	# [4-6 s]
		st_des = 0.0
	elif dt_secs < 8.0:	# [6-8 s]
		st_des = -0.01
	else:				# [8+ s]
		st_des = 0.0			

	return st_des

def ramp_response(dt_secs):
	if dt_secs < 2.0:	# [0-2 s]
		st_des = 0.0
	elif dt_secs < 4.0:	# [2-4 s]
		st_des = 0.01 * (dt_secs - 2.0)/2.0
	elif dt_secs < 6.0:	# [4-6 s]
		st_des = 0.01 - 0.01 * (dt_secs - 4.0)/2.0
	elif dt_secs < 8.0:	# [6-8 s]
		st_des = -0.01 * (dt_secs - 6.0)/2.0
	else:				# [8+ s]
		st_des = 0.0

	return st_des

def sine_response(dt_secs):
	# amplitude 0.01, period = 6 s starting at t = 2.0
	if dt_secs < 2.0:
		st_des = 0.0
	elif dt_secs < 8.0:
		tm_offset = dt_secs - 2.0
		st_des = 0.01 * math.sin(math.pi/3.0 * tm_offset)
	else:
		st_des = 0.0
	return st_des

if __name__=="__main__":
	try:
		parser = argparse.ArgumentParser('Run an open-loop steering response test on the Hyundai Genesis.')
		parser.add_argument('-m', '--mode', type=str, choices=['step', 'ramp', 'sine'], default='step', help='Test Mode')
		args = parser.parse_args()
		pub_steer(mode=args.mode)
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
