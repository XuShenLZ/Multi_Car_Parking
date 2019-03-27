#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np

from parking.msg import car_state, car_input

# Parameters
ego  = [ 3.7, 1, 1, 1]
L  = 2.7
xp = [0, 0, np.pi/4, 0]
up = [-np.pi/4, 0]

# Calculation
W_ev = ego[1]+ego[3]
L_ev = ego[0]+ego[2]

w = W_ev/2;
offset = np.array([L_ev/2 - ego[2], 0])

# =============== MAP Plotting ===============
def plot_map():
	# The length of the map
	l_map = 24

	# Lane width
	w_lane = 3

	# Spot width
	w_spot = 3
	l_spot = 5

	# Lower Boundary
	plt.plot([-l_map/2, l_map/2], [-w_lane-l_spot, -w_lane-l_spot], 'k', linewidth=2.0)
	plt.hold(True)
	# Upper Boundary
	plt.plot([-l_map/2, l_map/2], [ w_lane+l_spot,  w_lane+l_spot], 'k', linewidth=2.0)

	# Center line
	plt.plot([-l_map/2, l_map/2], [0, 0], '--k', linewidth=2.0)

	# Slot boundaries
	for i in range(0, l_map/w_spot + 1):
		plt.plot([-l_map/2 + i*w_spot, -l_map/2 + i*w_spot], [-w_lane-l_spot, -w_lane], 'k', linewidth=2.0)
		plt.plot([-l_map/2 + i*w_spot, -l_map/2 + i*w_spot], [ w_lane,  w_lane+l_spot], 'k', linewidth=2.0)

	# Plot properties
	plt.axis('equal')

	plt.title("Multi-car Parking")

	# plt.draw()

# =============== End MAP Plotting ===============

# =============== Plot Car Box ==============
def car_box(x0,phi,w,l):
	car1 = x0 + np.array([np.cos(phi)*l, np.sin(phi)*l]) + np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car2 = x0 + np.array([np.cos(phi)*l, np.sin(phi)*l]) - np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car3 = x0 - np.array([np.cos(phi)*l, np.sin(phi)*l]) + np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car4 = x0 - np.array([np.cos(phi)*l, np.sin(phi)*l]) - np.array([np.sin(phi)*w, -np.cos(phi)*w])
	plt.plot([car1[0],car2[0],car4[0],car3[0],car1[0]],[car1[1],car2[1],car4[1],car3[1],car1[1]],'k')
	# plt.draw()

# =============== End Car Box ==============

class CarSubscriber(object):
	"""docstring for CarSubscriber"""
	# car_num is the integer value of car number
	def __init__(self, car_num, x0, y0):
		super(CarSubscriber, self).__init__()
		self.x     = x0
		self.y     = y0
		self.psi   = 0.0
		self.v     = 0.0
		self.delta = 0.0
		self.acc   = 0.0
		# Auto-create topic name for different cars
		self.state_topicName = "state_%d" % car_num
		self.input_topicName = "input_%d" % car_num

		# Init subscriber
		rospy.Subscriber(self.state_topicName, car_state, self.state_cb)
		rospy.Subscriber(self.input_topicName, car_input, self.input_cb)
		print('Subscriber Built')

	# Call back functions
	def state_cb(self, data):
		self.x   = data.x[0]
		self.y   = data.y[0]
		self.psi = data.psi[0]
		self.v   = data.v[0]
		print("State Call Back")
		print(self.x)

	def input_cb(self, data):
		self.delta = data.delta[0]
		self.acc   = data.acc[0]

		print("Input Call Back")

	# Get data from the object
	def read_info(self):
		return self
		

def plot_car(info):
	# Plot map
	plt.ion()

	# Load Data
	x     = info.x
	y     = info.y
	psi   = info.psi
	delta = info.delta

	# Rotation matrix
	Rot = np.array([[np.cos(psi), -np.sin(psi)], \
					[np.sin(psi),  np.cos(psi)]])

	# Current position
	x_cur = np.array([x, y])

	# Compute Center of the Car
	centerCar = x_cur + np.dot(Rot, offset)

	# Plot
	# Car body
	car_box(centerCar, psi, W_ev/2, L_ev/2)
	# Front wheels - steering included
	car_box(x_cur + np.dot(Rot, np.array([L, w-0.15])) , psi + delta, 0.15, 0.3)
	car_box(x_cur + np.dot(Rot, np.array([L, -w+0.15])), psi + delta, 0.15, 0.3)
	# Rear wheels - no steering
	car_box(x_cur + np.dot(Rot, np.array([0,  w-0.15])), psi, 0.15, 0.3)
	car_box(x_cur + np.dot(Rot, np.array([0, -w+0.15])), psi, 0.15, 0.3)
	
	plt.hold(False)
	# plt.pause(0.001)

def main():

	# Subscriber Initialization
	rospy.init_node('plotCar', anonymous=True)

	plotter = CarSubscriber(1, -12, 1.5)
	plotter2 = CarSubscriber(2, -12, -1.5)

	loop_rate = 100
	rate = rospy.Rate(loop_rate)

	while not rospy.is_shutdown():
		plot_map()
		
		info = plotter.read_info()
		plot_car(info)

		plt.hold(True)
		info2 = plotter2.read_info()
		plot_car(info2)

		plt.pause(0.001)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	