#!/usr/bin/env python
import rospy
import rosbag
import pdb
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import numpy as np
import math

KPH_TO_MPS = 0.277778

def process_steering_msg(msg):
	tm = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	if spas:
		data = math.radians(msg.steering_wheel_angle)/15.87 # d_f, with steering ratio 15.87
	else:
		data = math.radians(msg.steering_wheel_angle)/15.87 #TODO: figure out what to use for torque/LKAS
	return tm, data

def process_speeds_msg(msg):
	# just use two rear wheels to get speed for now
	tm = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	avg_speed = 0.5 * (msg.wheel_speed_rl + msg.wheel_speed_rr) * KPH_TO_MPS
	return tm, avg_speed

class ROSTimeSeries():
	def __init__(self, dim):
		self.ts = []
		self.dim = dim
		self.ys = np.empty((0,dim))


	def append(self, t, y):
		if isinstance(y, float):
			assert(self.dim == 1)
			self.ys = np.append(self.ys, np.array([[y]]), axis=0)
		else:
			assert(len(y) == self.dim)
			self.ys = np.append(self.ys, [y], axis=0)
		
		self.ts.append(t)

	def zero_start_time(self):
		self.ts = [x - self.ts[0] for x in self.ts]

def lpf(signal):
	filt_signal = []
	filt_signal.append(signal[0])

	for i in range(1, len(signal)):
		filt_signal.append( 0.01* signal[i] + 0.99 * filt_signal[-1])

	return filt_signal

def main():
	bag = rosbag.Bag('/home/govvijay/Desktop/GENESIS/Open Loop Tests 6-6/ada11_lkas.bag')
	spas = False

	if spas:
		topic_list = ['/control/accel', '/control/steer_angle', '/vehicle/wheel_speeds', \
	              '/vehicle/steering']
	else:
		topic_list = ['/control/accel', '/control/steer_torque', '/vehicle/wheel_speeds', \
	              '/vehicle/steering']

	time_series = [ROSTimeSeries(1) for x in topic_list]

	for topic, msg, t in bag.read_messages(topics=topic_list):
		if '/vehicle/steering' in topic:
			tm, data = process_steering_msg(msg)
		elif '/vehicle/wheel_speeds' in topic :
			tm, data = process_speeds_msg(msg)
		else:
			tm = t.secs + 1e-9 * t.nsecs
			data = msg.data

		topic_ind = topic_list.index(topic)
		time_series[topic_ind].append(tm, data)

	plt.figure()
	plt.ioff()
	# Plot specific to steering test.
	if len(time_series[0].ts) == 0:

		plt.plot(time_series[1].ts, time_series[1].ys, 'k', label='CMD')
		plt.plot(time_series[3].ts, time_series[3].ys, 'r', label='ACT')
		plt.legend()
		plt.title('Steering Response')
		plt.xlabel('t (s)')
		plt.ylabel('Tire Angle (rad)')
	# Plot specific to acceleration test.
	else:
		dv = np.ravel( np.diff(time_series[2].ys, axis=0) )
		dt = np.diff(time_series[2].ts, axis=0)

		acc_measured = np.divide(dv, dt)
		acc_measured = lpf(acc_measured)

		plt.plot(time_series[0].ts, time_series[0].ys, 'k', label='CMD')
		#plt.plot(time_series[2].ts, time_series[2].ys, 'b', label='VEL')
		plt.plot(time_series[2].ts[:-1], acc_measured, 'r', label='ACT')
		plt.legend()
		plt.title('Acceleration Response')
		plt.xlabel('t (s)')
		plt.ylabel('Acc (m/s^2)')
	plt.show()

if __name__ == '__main__':
	main()

	

	



