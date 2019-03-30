#!/usr/bin/env python

import rospy
from parking.msg import car_state, car_input, cost_map
from parking.srv import maneuver
import math
import time
import numpy as np

# parameters
# The length of the map
l_map = 24

# Lane width
w_lane = 3.0

class Vehicle(object):
	"""docstring for Vehicle"""
	# car_num is the integer value of car number
	# lane = "U": Upper lane, start from [-l_map/2,  w_lane/2]
	# lane = "L": Lower lane, start from [-l_map/2, -w_lane/2]
	# t0: The time the car starts to move
	def __init__(self, car_num, lane, t0, goal):
		super(Vehicle, self).__init__()
		# Lane info
		self.lane = lane

		# Current time
		self.t  = 0
		# Departure time
		self.t0 = t0

		# The goal stopping position on straight line
		self.goal = goal
		
		# Discrete Time Step
		self.dt = 0.1

		# Index for reading parking maneuver
		self.pIdx = 0

		# State of the car
		self.x     = -l_map/2
		if self.lane == "U":
			self.y =  w_lane/2
		elif self.lane == "L":
			self.y = -w_lane/2
		else:
			self.y = 0
			print("The lane is not correctly specified")

		self.psi   = 0
		self.v     = 0.0

		# Input of the car
		self.delta = 0
		self.acc   = 0

		# If it arrived at the maneuver starting point
		self.parking = False

		# If the whole parking is finished
		self.terminated = False

		# Auto-create topic name for different cars
		self.state_topicName = "state_%d" % car_num
		self.input_topicName = "input_%d" % car_num

		# State publisher
		self.state_pub = rospy.Publisher(self.state_topicName, car_state, queue_size = 10)

	def publish_state(self):
		pub_data = car_state()
		pub_data.x   = [self.x]
		pub_data.y   = [self.y]
		pub_data.psi = [self.psi]
		pub_data.v   = [self.v]
		self.state_pub.publish(pub_data)

	def get_state(self):
		return self

	def start_parking(self):
		return self.parking

	def drive_straight(self):
		# If the car has now reached the departure time
		if self.t < self.t0:
			self.t += 1
			return

		# After the car reached the departure time

		# goal is the x value of parking starting point
		if self.goal - self.x >= 0.05:
			# If far away, keep going
			self.v = 4.0
			self.parking = False
		else:
			# If arrived, stop
			self.v = 0.0
			self.parking = True
		# State evolve
		self.x = self.x + self.dt * self.v

		self.publish_state()

	def drive_parking(self):
		# Calculate the offset for current starting position
		offset = self.goal - self.maneuver_data.path.x[0]

		i = self.pIdx
		self.x     = self.maneuver_data.path.x[i] + offset
		self.y     = self.maneuver_data.path.y[i]
		self.psi   = self.maneuver_data.path.psi[i]
		self.v     = self.maneuver_data.path.v[i]
		self.delta = self.maneuver_data.input.delta[i]
		self.acc   = self.maneuver_data.input.acc[i]

		self.publish_state()

		self.pIdx += 1

		if self.pIdx == len(self.maneuver_data.input.acc):
			# The path following is finished
			self.terminated = True
		# time.sleep(0.1)

	def get_restpark(self):
		# Return the rest of parking maneuver
		offset = self.goal - self.maneuver_data.path.x[0]
		end_idx = len(self.maneuver_data.path.x)

		rest_park = car_state()
		rest_park.x = np.array(self.maneuver_data.path.x[self.pIdx:end_idx]) + offset
		rest_park.y = np.array(self.maneuver_data.path.y[self.pIdx:end_idx])
		return rest_park

	def get_maneuver(self, end_spot, end_pose):
		# end_spot: "U"(Upper) or "L"(Lower)
		# end_pose: "F"(Front) or "R"(Reverse)
		rospy.wait_for_service('park_maneuver')
		try:
			maneuver_client = rospy.ServiceProxy('park_maneuver', maneuver)
			self.maneuver_data = maneuver_client(self.lane, end_spot, end_pose)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def is_terminated(self):
		return self.terminated
		

class CostMap(object):
	"""docstring for CostMap"""
	def __init__(self, length, width, gridsize):
		super(CostMap, self).__init__()
		# The dimension of the costmap
		self.gridsize = gridsize
		self.xgrid    = range(-length/2, length/2+1, self.gridsize)
		self.ygrid    = range(-width/2, width/2+1, self.gridsize)
		self.length   = len(self.xgrid)
		self.width    = len(self.ygrid)
		self.time     = 0
		
		# The costmap dictionary
		self.cost = {}

		for i in self.xgrid:
			for j in self.ygrid:
				self.cost[(i, j)] = 0

		# Cost Map publisher
		self.map_pub = rospy.Publisher('cost_map', cost_map, queue_size = 10)

	def pub_costmap(self):
		pub_data = cost_map()
		pub_data.length = self.length
		pub_data.width  = self.width
		pub_data.time   = self.time
		pub_data.data   = []
		for i in self.xgrid:
			for j in self.ygrid:
				pub_data.data.append(self.cost[(i, j)])
		self.map_pub.publish(pub_data)


	def reset_map(self):
		for i in self.xgrid:
			for j in self.ygrid:
				self.cost[(i, j)] = 0

	def write_cost(self, x, y):
		x_floor = math.floor(x-1)
		y_floor = math.floor(y-1)
		x_ceil  = math.ceil(x+1)
		y_ceil  = math.ceil(y+1)
		for i in range(int(x_floor), int(x_ceil)):
			for j in range(int(y_floor), int(y_ceil)):
				if self.cost.has_key((i,j)):
					self.cost[(i, j)] = 1

	def get_cost(self, x, y):
		x_floor = math.floor(x)
		y_floor = math.floor(y)
		if self.cost.has_key((int(x_floor), int(y_floor))):
			return self.cost[(int(x_floor), int(y_floor))]
		else:
			print("Out of costmap range")
			return 0

# Detect the collision of the path ahead
def not_collide(car, costmap):
	state = car.get_state()
	for l in range(0,7):
		for j in range(-1,2):
			check_x = state.x + l*math.cos(state.psi) + j*math.sin(state.psi)
			check_y = state.y + l*math.sin(state.psi) - j*math.cos(state.psi)
			if costmap.get_cost(check_x, check_y) != 0:
				return False

	return True
	

def write_cost_maneuver(car, costmap):
	rest_maneuver = car.get_restpark()
	length = len(rest_maneuver.x)

	for i in range(0,length):
		costmap.write_cost(rest_maneuver.x[i], rest_maneuver.y[i])

def main():

	# Init ROS Node
	rospy.init_node("carNode", anonymous=True)

	# The iteration time
	time = 0

	costmap = CostMap(24, 16, 1)

	car_list = []

	t0   = 0
	goal = -4.5
	car = Vehicle(1, "U", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)


	t0   = 40
	goal = -1.5
	car = Vehicle(2, "L", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)


	t0   = 40
	goal = -1.5
	car = Vehicle(3, "U", t0, goal)
	car.get_maneuver("U", "R")

	car_list.append(car)

	t0   = 0
	goal = -7.5
	car = Vehicle(4, "L", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)

	t0   = 20
	goal = 1.5
	car = Vehicle(5, "L", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)

	loop_rate = 10
	rate = rospy.Rate(loop_rate)
	while not rospy.is_shutdown():

		costmap.reset_map()

		for car in car_list:
			# If the car is still running
			if not car.is_terminated():
				# If the car has not arrived at the starting point
				if not car.start_parking():
					if not_collide(car, costmap):
						car.drive_straight()
				else:
					if not_collide(car, costmap):
						write_cost_maneuver(car, costmap)
						car.drive_parking()

				state = car.get_state()
				costmap.write_cost(state.x, state.y)

		costmap.pub_costmap()
		# Time increase
		time += 1
		rate.sleep()
		# raw_input()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		