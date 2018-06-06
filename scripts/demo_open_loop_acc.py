#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

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
	rospy.loginfo('OL Acc Test Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs
		
		if mode is 'step':
			a_des = step_response(dt_secs)
		elif mode is 'ramp':
			a_des = ramp_response(dt_secs)
		else:
			a_des = 0.0

		print('M: %s, T: %f, A: %f' % (mode, dt_secs, a_des))
		pub.publish(a_des)
		r.sleep()
		
	enable_pub.publish(0); # disable acceleration control.

def step_response(dt_secs):
	if dt_secs < 5.0:		# [0-5 s]
		a_des = 0.0
	elif dt_secs < 15.0:	# [5-20 s]
		a_des = 1.0
	elif dt_secs < 20.0:	# [20-25 s]
		a_des = 0.0
	elif dt_secs < 30.0:	# [25-35 s]
		a_des = -1.0
	else:					# [35+ s]
		a_des = 0.0				
	return a_des


def ramp_response(dt_secs):
	if dt_secs < 5.0:		# [0-5 s]
		a_des = 0.0
	elif dt_secs < 20.0:	# [5-20 s]
		a_des = 1.0 * (dt_secs - 5.0)/15.0
	elif dt_secs < 25.0:	# [20-25 s]
		a_des = 1.0 - (dt_secs - 20.0)/5.0
	elif dt_secs < 35.0:	# [25-35 s]
		a_des = -1.0 * (dt_secs - 25.0)/10.0
	else:					# [35+ s]
		a_des = 0.0				
	return a_des

if __name__=="__main__":
	try:
		pub_acc(mode='step')
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly

