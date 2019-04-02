#!/usr/bin/env python

import rospy
from parking.msg import car_state, car_input, cost_map
from parking.srv import maneuver
import math
import time
import numpy as np

# parameters
# The length of the map
l_map  = rospy.get_param('l_map')

# Lane width
w_lane = rospy.get_param('w_lane')

class Vehicle(object):
	"""docstring for Vehicle"""
	# car_num is the integer value of car number
	# lane = "U": Upper lane, start from [-l_map/2,  w_lane/2]
	# lane = "L": Lower lane, start from [-l_map/2, -w_lane/2]
	# t0: The time the car starts to move
	def __init__(self, car_num, lane, t0, goal):
		super(Vehicle, self).__init__()
		# Car Number
		self.car_num = car_num

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
		self.park_topicName  = "park_%d" % car_num

		# State publisher
		self.state_pub = rospy.Publisher(self.state_topicName, car_state, queue_size = 10)

		# Parking maneuver publisher
		self.maneuver_pub = rospy.Publisher(self.park_topicName, car_state, queue_size = 10)

	def publish_state(self):
		pub_data = car_state()
		pub_data.x   = [self.x]
		pub_data.y   = [self.y]
		pub_data.psi = [self.psi]
		pub_data.v   = [self.v]
		self.state_pub.publish(pub_data)

	def get_state(self):
		return self

	def is_parking(self):
		return self.parking

	def add_time(self):
		self.t += 1

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

		rest_park     = car_state()
		rest_park.x   = np.array(self.maneuver_data.path.x[self.pIdx:end_idx]) + offset
		rest_park.y   = np.array(self.maneuver_data.path.y[self.pIdx:end_idx])
		rest_park.psi = np.array(self.maneuver_data.path.psi[self.pIdx:end_idx])
		self.maneuver_pub.publish(rest_park)
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

	# Detect the collision of the path ahead
	def not_collide(self, costmap):

		if not self.is_parking():
			# Only need to see front if going straight
			for l in range(2,7):
				for j in range(-1,2):
					check_x = self.x + l*math.cos(self.psi) + j*math.sin(self.psi)
					check_y = self.y + l*math.sin(self.psi) - j*math.cos(self.psi)
					cost = costmap.get_cost(check_x, check_y)
					if cost[0] != 0 and cost[1] != self.car_num:
						return False
		else:
			# Only need to check along the parking trajectory
			rest_maneuver = self.get_restpark()
			for i in range(0, len(rest_maneuver.x)):
				for l in range(-1,5):
					for j in range(-1,2):
						check_x = rest_maneuver.x[i] + l*math.cos(rest_maneuver.psi[i]) + j*math.sin(rest_maneuver.psi[i])
						check_y = rest_maneuver.y[i] + l*math.sin(rest_maneuver.psi[i]) - j*math.cos(rest_maneuver.psi[i])
						cost = costmap.get_cost(check_x, check_y)
						if cost[0] != 0 and cost[1] != self.car_num:
							return False

		return True
		

class CostMap(object):
	"""docstring for CostMap"""
	def __init__(self):
		super(CostMap, self).__init__()
		# The dimension of the costmap
		gridsize = rospy.get_param('gridsize')
		cmap_len = rospy.get_param('cmap_len')
		cmap_wid = rospy.get_param('cmap_wid')
		self.xgrid    = range(-cmap_len/2, cmap_len/2+1, gridsize)
		self.ygrid    = range(-cmap_wid/2, cmap_wid/2+1, gridsize)
		self.length   = len(self.xgrid)
		self.width    = len(self.ygrid)
		self.time     = 0
		
		# The costmap dictionary
		self.cost = {}

		for i in self.xgrid:
			for j in self.ygrid:
				# The first is cost, the second is car_num
				self.cost[(i, j)] = (0, 0)

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
				# Only publish the cost value
				pub_data.data.append(self.cost[(i, j)][0])
		self.map_pub.publish(pub_data)


	def reset_map(self):
		for i in self.xgrid:
			for j in self.ygrid:
				self.cost[(i, j)] = (0, 0)

	def write_cost(self, x, y, psi, car_num):
		for l in range(-1,5):
			for j in range(-1,2):
				write_x = math.floor(x + l*math.cos(psi) + j*math.sin(psi))
				write_y = math.floor(y + l*math.sin(psi) - j*math.cos(psi))
				if self.cost.has_key((int(write_x), int(write_y))):
					self.cost[(int(write_x), int(write_y))] = (1, car_num)
	
	def write_cost_maneuver(self, car):
		state = car.get_state()
		rest_maneuver = car.get_restpark()
		length = len(rest_maneuver.x)

		for i in range(0,length):
			self.write_cost(rest_maneuver.x[i], rest_maneuver.y[i], rest_maneuver.psi[i], state.car_num)


	def get_cost(self, x, y):
		x_floor = math.floor(x)
		y_floor = math.floor(y)
		if self.cost.has_key((int(x_floor), int(y_floor))):
			return self.cost[(int(x_floor), int(y_floor))]
		else:
			print("Detection is out of costmap range")
			return (0, 0)
	
def main():

	# Init ROS Node
	rospy.init_node("carNode", anonymous=True)

	costmap = CostMap()

	car_list = init_cars()

	loop_rate = rospy.get_param('ctrl_rate')
	rate = rospy.Rate(loop_rate)
	while not rospy.is_shutdown():

		for car in car_list:
			# Update Map
			costmap.reset_map()

			# Register the current positions of all cars
			for car0 in car_list:
				if not car0.is_terminated():
					state = car0.get_state()
					costmap.write_cost(state.x, state.y, state.psi, state.car_num)

			# Register the collision-free parking maneuver
			for car0 in car_list:
				if not car0.is_terminated():
					if car0.not_collide(costmap):
						if car0.is_parking():
							costmap.write_cost_maneuver(car0)

			costmap.pub_costmap()

			# Car control
			# If the car is still running
			if not car.is_terminated():
				# Time increase
				car.add_time()
				# If the car has not arrived at the starting point
				if not car.is_parking():
					if car.not_collide(costmap):
						car.drive_straight()
				else:
					if car.not_collide(costmap):
						car.drive_parking()


		rate.sleep()
		# raw_input()

def init_cars():

	car_list = []

	t0   = 0
	goal = -4.5
	car = Vehicle(1, "U", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)

	t0   = 20
	goal = -1.5
	car = Vehicle(2, "L", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)


	t0   = 20
	goal = -1.5
	car = Vehicle(3, "U", t0, goal)
	car.get_maneuver("U", "R")

	car_list.append(car)

	t0   = 30
	goal = -7.5
	car = Vehicle(4, "L", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)

	t0   = 40
	goal = 1.5
	car = Vehicle(5, "L", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)

	t0   = 30
	goal = 1.5
	car = Vehicle(6, "U", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)

	t0   = 40
	goal = 4.5
	car = Vehicle(7, "U", t0, goal)
	car.get_maneuver("U", "R")

	car_list.append(car)

	t0   = 50
	goal = 4.5
	car = Vehicle(8, "U", t0, goal)
	car.get_maneuver("L", "F")

	car_list.append(car)

	t0   = 60
	goal = -7.5
	car = Vehicle(9, "U", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)

	t0   = 50
	goal = -4.5
	car = Vehicle(10, "L", t0, goal)
	car.get_maneuver("L", "F")

	car_list.append(car)

	t0   = 70
	goal = -10.5
	car = Vehicle(11, "U", t0, goal)
	car.get_maneuver("U", "F")

	car_list.append(car)

	t0   = 60
	goal = -10.5
	car = Vehicle(12, "L", t0, goal)
	car.get_maneuver("L", "R")

	car_list.append(car)

	return car_list

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		