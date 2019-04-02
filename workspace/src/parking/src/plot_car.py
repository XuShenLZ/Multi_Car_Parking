#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np

from parking.msg import car_state, car_input, cost_map

# Parameters
ego = rospy.get_param('ego')
L   = rospy.get_param('L')

# The length of the map
l_map  = rospy.get_param('l_map')

# Lane width
w_lane = rospy.get_param('w_lane')

# Spot width
w_spot = rospy.get_param('w_spot')
l_spot = rospy.get_param('l_spot')

# Calculation
W_ev = ego[1]+ego[3]
L_ev = ego[0]+ego[2]

w = W_ev/2;
offset = np.array([L_ev/2 - ego[2], 0])

# =============== MAP Plotting ===============
def plot_map():
	# Interactive mode
	plt.ion()

	# Lower Boundary
	plt.plot([-l_map/2, l_map/2], [-w_lane-l_spot, -w_lane-l_spot], 'k', linewidth=3.0)
	plt.hold(True)
	# Upper Boundary
	plt.plot([-l_map/2, l_map/2], [ w_lane+l_spot,  w_lane+l_spot], 'k', linewidth=3.0)

	# Center line
	plt.plot([-l_map/2, l_map/2], [0, 0], '--k', linewidth=2.0)

	# Slot boundaries
	for i in range(0, int(l_map/w_spot) + 1):
		plt.plot([-l_map/2 + i*w_spot, -l_map/2 + i*w_spot], [-w_lane-l_spot, -w_lane], 'k', linewidth=3.0)
		plt.plot([-l_map/2 + i*w_spot, -l_map/2 + i*w_spot], [ w_lane,  w_lane+l_spot], 'k', linewidth=3.0)

	# Plot properties
	plt.axis('equal')
	# plt.xlim(-20, 20)
	# plt.ylim(-10, 10)

	plt.title("Multi-car Parking")

	# plt.draw()

# =============== End MAP Plotting ===============

# =============== Plot Car Box ==============
def car_box(x0,phi,w,l):
	car1 = x0 + np.array([np.cos(phi)*l, np.sin(phi)*l]) + np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car2 = x0 + np.array([np.cos(phi)*l, np.sin(phi)*l]) - np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car3 = x0 - np.array([np.cos(phi)*l, np.sin(phi)*l]) + np.array([np.sin(phi)*w, -np.cos(phi)*w])
	car4 = x0 - np.array([np.cos(phi)*l, np.sin(phi)*l]) - np.array([np.sin(phi)*w, -np.cos(phi)*w])
	plt.plot([car1[0],car2[0],car4[0],car3[0],car1[0]],[car1[1],car2[1],car4[1],car3[1],car1[1]],'b', linewidth=2.0)
	plt.xlim(-15, 15)
	# plt.draw()

# =============== End Car Box ==============

class CarSubscriber(object):
	"""docstring for CarSubscriber"""
	# car_num is the integer value of car number
	# lane = "U": Upper lane, start from [-l_map/2,  w_lane/2]
	# lane = "L": Lower lane, start from [-l_map/2, -w_lane/2]
	def __init__(self, car_num, lane):
		super(CarSubscriber, self).__init__()
		self.lane = lane
		self.x     = -l_map/2
		if self.lane == "U":
			self.y =  w_lane/2
		elif self.lane == "L":
			self.y = -w_lane/2
		else:
			self.y = 0.0
			print("The lane is not correctly specified")

		self.psi   = 0.0
		self.v     = 0.0
		self.delta = 0.0
		self.acc   = 0.0

		self.rest_park = car_state()
		self.rest_park.x = []
		self.rest_park.y = []

		# Auto-create topic name for different cars
		self.state_topicName = "state_%d" % car_num
		self.input_topicName = "input_%d" % car_num
		self.park_topicName  = "park_%d" % car_num

		# Init subscriber
		rospy.Subscriber(self.state_topicName, car_state, self.state_cb)
		rospy.Subscriber(self.input_topicName, car_input, self.input_cb)
		rospy.Subscriber(self.park_topicName,  car_state, self.park_cb)
		print('Plot Subscriber for Car#%d Built' % car_num)

	# Call back functions
	def state_cb(self, data):
		self.x   = data.x[0]
		self.y   = data.y[0]
		self.psi = data.psi[0]
		self.v   = data.v[0]
		# print("State Call Back")

	def input_cb(self, data):
		self.delta = data.delta[0]
		self.acc   = data.acc[0]

		# print("Input Call Back")

	# Receive the remaining parking maneuver trajectory
	def park_cb(self, data):
		self.rest_park.x = data.x
		self.rest_park.y = data.y

	# Plot the parking maneuver trajectory
	def plot_park(self):
		plt.plot(self.rest_park.x, self.rest_park.y, linewidth=1)

	# Get data from the object
	def read_info(self):
		return self
	
	# Plot Car
	def plot_car(self):

		# Load Data
		x     = self.x
		y     = self.y
		psi   = self.psi
		delta = self.delta

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
		
		# plt.hold(False)
		# plt.pause(0.001)

class CostMapSubscriber(object):
	"""docstring for CostMapSubscriber"""
	def __init__(self):
		super(CostMapSubscriber, self).__init__()
		gridsize = rospy.get_param('gridsize')
		cmap_len = rospy.get_param('cmap_len')
		cmap_wid = rospy.get_param('cmap_wid')
		self.xgrid    = range(-cmap_len/2, cmap_len/2+1, gridsize)
		self.ygrid    = range(-cmap_wid/2, cmap_wid/2+1, gridsize)

		self.length   = len(self.xgrid)
		self.width    = len(self.ygrid)
		self.time     = 0

		self.X, self.Y = np.meshgrid(self.xgrid, self.ygrid)
		self.cost     = np.zeros((self.width, self.length))

		rospy.Subscriber('cost_map', cost_map, self.costmap_cb)

	def costmap_cb(self, data):
		self.length = data.length
		self.width  = data.width
		self.time   = data.time

		# self.cost   = np.zeros((self.width, self.length))

		# The index to extract value from cost_map.data
		k = 0
		# j-th column: x
		for j in range(0, self.length):
			# i-th row: y
			for i in range(0, self.width):
				self.cost[i][j] = data.data[k]
				k += 1

		# print("Cost Map Updated")

	def plot_costmap(self):
		plt.pcolormesh(self.X, self.Y, self.cost, alpha=0.5)

def main():

	# Subscriber Initialization
	rospy.init_node('plotCar', anonymous=True)

	plotter_list = init_cars()

	costmap = CostMapSubscriber()

	loop_rate = rospy.get_param('plot_rate')
	rate = rospy.Rate(loop_rate)

	while not rospy.is_shutdown():
		plot_map()

		costmap.plot_costmap()
		
		for plotter in plotter_list:
			plt.hold(True)
			# Plot the parking maneuver
			plotter.plot_park()
			# Plot the car body
			plotter.plot_car()
			plt.hold(False)

		plt.pause(0.0001)

		rate.sleep()

def init_cars():

	plotter_list = []

	plotter = CarSubscriber(1, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(2, "L")
	plotter_list.append(plotter)

	plotter = CarSubscriber(3, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(4, "L")
	plotter_list.append(plotter)

	plotter = CarSubscriber(5, "L")
	plotter_list.append(plotter)

	plotter = CarSubscriber(6, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(7, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(8, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(9, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(10, "L")
	plotter_list.append(plotter)

	plotter = CarSubscriber(11, "U")
	plotter_list.append(plotter)

	plotter = CarSubscriber(12, "L")
	plotter_list.append(plotter)

	return plotter_list

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	