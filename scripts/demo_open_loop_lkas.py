#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

# Open loop tests to evaluate low-level control tracking of tire angle (d_f) input.
# NOTE: Assumes d_f in radians.  This may change with ECU reprogramming!

def pub_steer(mode='step'):
	enable_pub = rospy.Publisher("control/enable_lkas", UInt8, queue_size = 1, latch=True)
	pub = rospy.Publisher("control/steer_torque", Float32, queue_size = 1) 	
	rospy.init_node('pub_lkas_ol', anonymous=True)
	r = rospy.Rate(10.0)

	while(enable_pub.get_num_connections == 0):
		print 'Waiting to enable steering!'
		r.sleep()

	enable_pub.publish(1); # enable steering control.

	t_start = rospy.Time.now()
	rospy.loginfo('OL Steer Test Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		# 14.5 steering ratio so 0.01 df -> 8.3 deg swa
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs

		if mode is 'step':
			st_des = step_response(dt_secs)
		elif mode is 'ramp':
			st_des = ramp_response(dt_secs)
		else:
			st_des = 0.0

		print('M: %s, T: %f, DF: %f' % (mode, dt_secs, st_des))

		pub.publish(st_des)
		r.sleep()

	enable_pub.publish(0); # disable steering control.

def step_response(dt_secs):
	if dt_secs < 2.0:		# [0-5 s]
		st_des = 0.0
	elif dt_secs < 4.0:	# [5-10 s]
		st_des = 2.0
	elif dt_secs < 6.0:	# [10-15 s]
		st_des = -2.0
	elif dt_secs < 8.0:	# [15-20 s]
		st_des = 0.0
	else:					# [20+ s]
		st_des = 0.0			

	return st_des

def ramp_response(dt_secs):
	if dt_secs < 5.0:		# [0-5 s]
		st_des = 0.0
	elif dt_secs < 10.0:	# [5-10 s]
		st_des = 0.0
	elif dt_secs < 15.0:	# [10-15 s]
		st_des = 0.0
	elif dt_secs < 20.0:	# [15-20 s]
		st_des = 0.0
	else:					# [20+ s]
		st_des = 0.0

	return st_des

if __name__=="__main__":
	try:
		pub_steer(mode='step')
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
