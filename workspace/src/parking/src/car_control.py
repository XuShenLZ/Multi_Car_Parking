#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16MultiArray
from parking.msg import car_state, car_input, cost_map
from parking.srv import maneuver
import math
import time
import os
import random
import numpy as np

# Data recording
import csv
import pickle

# Debugging
# import ipdb

# Spot allocation plans
import spot_allocate

# parameters
# Whether the lane and maneuver are randomly chosen
is_random = rospy.get_param('is_random')
# Whether the initial occupancy is randomly chosen
rand_occupy = rospy.get_param('rand_occupy')

# Range and number of iterations
itv_low   = rospy.get_param('itv_low')
itv_high  = rospy.get_param('itv_high')
num_rep   = rospy.get_param('num_rep')

# Assignment policy
assign_policy = rospy.get_param('assign_policy')

# Lane Opening
lane_open = rospy.get_param('lane_open')

# Data saving path, at the home path of user
time_data_path = os.environ['HOME']

# The length of the map
l_map  = rospy.get_param('l_map')
w_map  = rospy.get_param('w_map')

# Lane width
w_lane = rospy.get_param('w_lane')

# Spot width
w_spot = rospy.get_param('w_spot')
l_spot = rospy.get_param('l_spot')

maneuver_client = rospy.ServiceProxy('park_maneuver', maneuver)

# Construct a dictionary for the convenience of 
# passing parameters
Map = {}
Map['l_map']  = l_map
Map['w_map']  = w_map
Map['w_lane'] = w_lane
Map['w_spot'] = w_spot
Map['l_spot'] = l_spot

# The total amount of car
total_number = rospy.get_param('total_number')

# Parameter for exponential arrival
exp_beta = rospy.get_param('exp_beta')

# Vacant spot
# Initially, all zero
# The shape of matrix is the same as parking lot
spots_U = np.zeros((2, 22), dtype=int)
spots_L = np.zeros((2, 22), dtype=int)

spot_list = []

interval = 1

class Vehicle(object):
	"""docstring for Vehicle"""
	# car_num is the integer value of car number
	# lane = "U": Upper lane, start from [-l_map/2,  w_lane/2]
	# lane = "L": Lower lane, start from [-l_map/2, -w_lane/2]
	# t0: The time the car starts to move
	def __init__(self, car_num, lane, t0, x0, end_pose, front_num, control):
		super(Vehicle, self).__init__()
		# Car Number
		self.car_num = car_num

		# The car_num which is in front of it
		self.front_num = front_num

		# Lane info
		self.lane = lane

		# Current time
		self.t  = 0
		# Departure time
		self.t0 = t0

		# Wait time
		self.wait_t = 0
		
		# Discrete Time Step
		self.dt = 0.1

		# Index for reading parking maneuver
		self.pIdx = 0

		# State of the car
		self.x     = x0
		if self.lane == "U":
			self.y =  w_lane/2
			self.R = (l_spot + w_lane) / 2
		elif self.lane == "L":
			self.y = -w_lane/2
			self.R = (l_spot + w_lane) / 2 + w_lane
		else:
			self.y = 0
			print("The lane is not correctly specified")

		# Arc length
		self.arc = 0.5 * math.pi * self.R

		# Turning points
		self.turn = [0.0, 0.0, 0.0, 0.0]
		# The x offset for turning center
		# x offset is the offset from x=l_map/2
		# Positive: move to right; Negative: move to left
		self.turn_offset_x = 0.0
		# y offset is the offset from y=w_map/2
		# Always positive
		self.turn_offset_y = l_spot/2
		# If the car end on the upper half -- turning is needed
		self.turn[0] = l_map/2 + self.turn_offset_x
		self.turn[1] = self.turn[0] + self.arc
		self.turn[2] = self.turn[1] + 2*self.turn_offset_y
		self.turn[3] = self.turn[2] + self.arc

		if control == True:
			# The goal stopping position on straight line
			print("Allocation interval = %d" % interval)
			# self.goal, self.end_spot = spot_allocate.deepest(self, Map, spots_U, spots_L)
			# self.goal, self.end_spot = spot_allocate.same_side(self, Map, spots_U, spots_L)
			# print("Same Side Strategy is used")
			# self.goal, self.end_spot = spot_allocate.same_side_n(self, Map, spots_U, spots_L, interval)
			# print("Random Half Strategy is used")
			# self.goal, self.end_spot = spot_allocate.random_assign(self, Map, spots_U, spots_L)
			if assign_policy == 1:
				print("Paried Assign Policy is used")
				self.goal, self.end_spot = spot_allocate.deepest_n(self, Map, spots_U, spots_L, interval)
			elif assign_policy == 2:
				print("Single Assign Policy is used")
				self.goal, self.end_spot = spot_allocate.solo_n(self, Map, spots_U, spots_L, spot_list, interval)
			else:
				print("Random Assign Policy Strategy is used")
				self.goal, self.end_spot = spot_allocate.random_assign_all(self, Map, spots_U, spots_L)

			self.end_pose = end_pose
			self.get_maneuver(self.end_spot, self.end_pose)

		# The longitudinal state for circular motion
		self.s     = self.x

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

		if control == True:
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
		self.t += self.dt

	def add_wait_time(self):
		if self.t >= self.t0:
			self.wait_t += self.dt

	def get_wait_time(self):
		return self.wait_t

	def get_total_time(self):
		return self.t - self.t0

	def drive_straight(self, car_list):
		# If the car has now reached the departure time
		if self.t < self.t0:
			front_car_state = car_list[self.front_num].get_state()
			self.s = min(-l_map/2, front_car_state.s) - 5
			self.x = self.s
			return

		# goal is the s value of parking starting point
		if self.goal[2] - self.s >= 0.05:
			# If far away, keep going
			self.v = 4.0
			self.parking = False
		else:
			# If arrived, stop
			self.v = 0.0
			self.parking = True
			return
		
		# State evolve
		self.s += self.dt * self.v

		# If in the lower half
		if self.s <= self.turn[0]:
			self.x = self.s
			if self.lane == "U":
				self.y =  w_lane/2
			else:
				self.y = -w_lane/2
			self.psi   = 0

		# If along the first circle
		elif self.s > self.turn[0] and self.s <= self.turn[1]:
			angle = (self.s - self.turn[0]) / self.arc * math.pi/2
			self.x   = l_map/2 + self.turn_offset_x + self.R * math.sin(angle)
			self.y   = w_map/2 - self.turn_offset_y - self.R * math.cos(angle)
			self.psi = angle

		# If along the vertical lane
		elif self.s > self.turn[1] and self.s <= self.turn[2]:
			self.x   = l_map/2 + self.turn_offset_x + self.R
			self.y   = w_map/2 - self.turn_offset_y + self.s - self.turn[1]
			self.psi = math.pi/2

		# If along the second circle
		elif self.s > self.turn[2] and self.s <= self.turn[3]:
			angle = (self.s - self.turn[2]) / self.arc * math.pi/2
			self.x   = l_map/2 + self.R * math.cos(angle)
			self.y   = w_map/2 + self.turn_offset_y + self.R * math.sin(angle)
			self.psi = math.pi/2 + angle

		# If in the upper half
		elif self.s > self.turn[3]:
			self.x = l_map/2 + self.turn_offset_x - (self.s - self.turn[3])
			if self.lane == "U":
				self.y = w_map - w_lane/2
			else:
				self.y = w_map + w_lane/2
			self.psi   = math.pi


		self.publish_state()

	def drive_parking(self):
		# Calculate the offset for current starting position
		offset    = [0, 0, 0]
		offset[0] = self.goal[0] - self.maneuver_data.path.x[0]
		# If on the upper half
		if self.goal[1] >= w_map/2:
			offset[1] = self.goal[1] - self.maneuver_data.path.y[0]
			offset[2] = math.pi
		# If on the lower half
		else:
			offset[1] = 0
			offset[2] = 0

		i = self.pIdx
		self.x     = self.maneuver_data.path.x[i] + offset[0]
		self.y     = self.maneuver_data.path.y[i] + offset[1]
		self.psi   = self.maneuver_data.path.psi[i] + offset[2]
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
		offset    = [0, 0, 0]
		offset[0] = self.goal[0] - self.maneuver_data.path.x[0]
		# If on the upper half
		if self.goal[1] >= w_map/2:
			offset[1] = self.goal[1] - self.maneuver_data.path.y[0]
			offset[2] = math.pi
		# If on the lower half
		else:
			offset[1] = 0
			offset[2] = 0

		end_idx = len(self.maneuver_data.path.x)

		rest_park     = car_state()
		rest_park.x   = np.array(self.maneuver_data.path.x[self.pIdx:end_idx]) + offset[0]
		rest_park.y   = np.array(self.maneuver_data.path.y[self.pIdx:end_idx]) + offset[1]
		rest_park.psi = np.array(self.maneuver_data.path.psi[self.pIdx:end_idx]) + offset[2]
		self.maneuver_pub.publish(rest_park)
		return rest_park

	def get_maneuver(self, end_spot, end_pose):
		# end_spot: "U"(Upper) or "L"(Lower)
		# end_pose: "F"(Front) or "R"(Reverse)
		rospy.wait_for_service('park_maneuver')
		try:
			# maneuver_client = rospy.ServiceProxy('park_maneuver', maneuver)
			self.maneuver_data = maneuver_client(self.lane, end_spot, end_pose)

			if self.goal[1] >= w_map/2:
				self.maneuver_data.path.x = [-i for i in self.maneuver_data.path.x]
				self.maneuver_data.path.y = [-i for i in self.maneuver_data.path.y]

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def change_maneuver(self):
		if self.end_pose == "F":
			other_pose = "R"
		else:
			other_pose = "F"

		self.get_maneuver(self.end_spot, other_pose)
		self.end_pose = other_pose

	def is_terminated(self):
		return self.terminated

	# Detect the collision of the path ahead
	def not_collide(self, costmap):

		if not self.is_parking():
			# If the turning is concerned
			if self.s >= self.turn[0] - 2*w_spot and self.s < self.turn[3]:
				for l in range(3,8):
					# Extend the detection along s path
					s0 = self.s + l
					# If the detection point is before the 1st turn
					if s0 < self.turn[0]:
						check_x = s0
						check_y = self.y
					# If the detection point is during the 1st turn
					elif s0 >= self.turn[0] and s0 < self.turn[1]:
						angle = (s0 - self.turn[0]) / self.arc * 0.5 * math.pi
						check_x   = l_map/2 + self.R * math.sin(angle)
						check_y   = w_map/2 - self.turn_offset_y - self.R * math.cos(angle)
					# If the detection point is during the vertical line
					elif s0 >= self.turn[1] and s0 < self.turn[2]:
						check_x   = l_map/2 + self.turn_offset_x + self.R
						check_y   = w_map/2 - self.turn_offset_y + s0 - self.turn[1]
					# If the detection point is during the 2nd turn
					elif s0 >= self.turn[2] and s0 < self.turn[3]:
						angle = (s0 - self.turn[2]) / self.arc * 0.5 * math.pi
						check_x   = l_map/2 + self.R * math.cos(angle)
						check_y   = w_map/2 + self.turn_offset_y + self.R * math.sin(angle)
					# If the detection point is out of turning region
					else:
						check_x   = l_map/2 + self.turn_offset_x - (s0 - self.turn[3])
						check_y   = w_map/2 + self.turn_offset_y + self.R

					cost = costmap.get_cost(check_x, check_y)
					if cost[0] != 0 and cost[1] != self.car_num:
						return False
			else:
			# Only need to see front if going straight
				for l in range(3,8):
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
		self.xgrid    = range(-cmap_len/2-200,   cmap_len/2+3*int(w_spot)+1, gridsize)
		self.ygrid    = range(-cmap_wid/2, cmap_wid*3/2+1, gridsize)
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
			# print("Detection is out of costmap range")
			return (0, 0)
	
def main():

	# Init ROS Node
	rospy.init_node("carNode", anonymous=True)
	global interval

	# To iterate over all interval sizes
	for itv in range(itv_low,itv_high):
		interval = itv
		for rep in range(num_rep):
			print('Currently it is #%d iteration' % rep)

			# The total time to finish the whole task
			total_task_time = 0

			car_list = init_cars(is_random)
			# ipdb.set_trace()

			costmap = CostMap()

			dead_lock_count  = 0
			max_queue_length = 0

			loop_rate = rospy.get_param('ctrl_rate')
			rate = rospy.Rate(loop_rate)

			# ============ For Carla Replay
			# Lists of time and trajectroy
			time_list  = []
			state_list = []
			input_list = []

			while not rospy.is_shutdown():
				# Update Map
				costmap.reset_map()

				# Measure queue length
				queue_length = [0, 0]

				# Reset the state and input of all vehicles at current time step
				state_current = []
				input_current = []

				# Firstly register all vehicle positions
				# And count queue length
				for car in car_list:
					state = car.get_state()

					# Log into the state sequence
					state_current.append({'x': state.x, 'y': state.y, 'psi': state.psi, 'v': state.v})
					input_current.append({'delta': state.delta, 'acc': state.acc})

					if not car.is_terminated():
						costmap.write_cost(state.x, state.y, state.psi, state.car_num)

						# Queue length
						if state.s < -l_map/2 and state.t > state.t0:
							if state.lane == "U":
								queue_length[0] += 1
							else:
								queue_length[1] += 1

				# Write the max queue length
				max_queue_length = max(max_queue_length, max(queue_length))

				# Then register parking maneuver if collision-free
				for car in car_list:
					if not car.is_terminated():
						if car.not_collide(costmap):
							if car.is_parking():
								costmap.write_cost_maneuver(car)

				costmap.pub_costmap()

				# dead lock flag
				dead_lock = True

				# Control
				for car in car_list:
					if not car.is_terminated():
						# Time increase
						car.add_time()
						if car.not_collide(costmap):
							dead_lock = False
							if car.is_parking():
								car.drive_parking()
							else:
								car.drive_straight(car_list)
						else:
							car.add_wait_time()

				# Dead lock resolution:
				# Change a maneuver
				if dead_lock:
					print("Dead Lock")
					dead_lock_count += 1
					for car in car_list:
						if (not car.is_terminated()) and car.is_parking():
							car.change_maneuver()
							break
				else:
					# The previous dead lock is 
					# handled successfully
					dead_lock_count = 0

				# If after the dead lock resolution
				# in the next loop, there is still dead lock
				# record the data an exit
				if dead_lock_count > 1:
					print("Dead Lock is not resolved. Exiting... Please refer to the saved data file for information")
					
					# Mark something in the CSV file
					dead_lock_mark = [-1 for car in car_list]
					with open(time_data_path + "/wait_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow(dead_lock_mark)

					with open(time_data_path + "/total_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow(dead_lock_mark)

					with open(time_data_path + "/task_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow([-1])

					with open(time_data_path + "/queue_length.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow([-1])

					break

				# Log the time and trajectory
				time_list.append(total_task_time)
				state_list.append(state_current)
				input_list.append(input_current)

				# Check whether all cars are terminated
				terminated_list = [car.is_terminated() for car in car_list]
				if all(terminated_list):
					# Get the waiting time
					wait_time_list = [car.get_wait_time() for car in car_list]

					# Record data
					with open(time_data_path + "/wait_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow(wait_time_list)

					# Get the total time for each car
					total_time_list = [car.get_total_time() for car in car_list]

					# Record data
					with open(time_data_path + "/total_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow(total_time_list)

					# Record the time for the entire task
					with open(time_data_path + "/task_time.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow([total_task_time])

					with open(time_data_path + "/queue_length.csv", 'a+') as f:
						writer = csv.writer(f)
						writer.writerow([max_queue_length])

					# Record the trajectory into the pickle file
					with open(time_data_path + '/trajectory.pickle', 'w') as f:
						pickle.dump([time_list, state_list, input_list], f)

					# Leave the while loop, end the program
					break

				# Add a total time
				total_task_time += 0.1
				
				rate.sleep()

def init_cars(is_random):
	car_init_pub = rospy.Publisher('car_init', String, queue_size = 10)

	# Reset the occupancy map
	global spots_U, spots_L, spot_list
	spots_U, spots_L = init_occupancy(rand_occupy)
	spot_list = []

	# occupied = [5, 8, 9, 14, 16, 17, 19, 20, 21]
	# for x in occupied:
	# 	spots_U[0, x] = 1

	# occupied = [5, 8, 9, 11, 12, 14, 15, 17, 20, 21]
	# for x in occupied:
	# 	spots_U[1, x] = 1

	# occupied = [0, 2, 5, 6, 8, 12, 14, 15]
	# for x in occupied:
	# 	spots_L[0, x] = 1

	# occupied = [0, 1, 2, 3, 4, 6, 10, 11, 12, 13, 17, 18, 20]
	# for x in occupied:
	# 	spots_L[1, x] = 1

	car_list = []

	car_list_U = []
	car_list_L = []

	t0_U = 0
	t0_L = 0
	lane_string = ""

	if is_random:
		# If the initialization is done by random
		lane_list      = []
		t0_list        = []
		x0_list        = []
		end_pose_list  = []
		end_spot_list  = []
		goal_list      = []
		front_num_list = []

		for car_num in range(total_number):
			# Generate the starting time,
			# starting lane, and end_pose randomly
			dt, lane, end_pose = random_start()
			# Allocate the end spot for each car
			print(car_num)
			front_num = 0
			if lane == "U":
				t0_U += dt
				t0 = t0_U
				# The starting position
				if len(car_list_U) == 0:
					x0 = -l_map/2
				else:
					last_car_state = car_list_U[-1].get_state()
					# Following the last vehicle in this lane
					# But must outside the lot
					x0 = min(-l_map/2, last_car_state.x - 5)
					front_num = last_car_state.car_num
			else:
				t0_L += dt
				t0 = t0_L
				if len(car_list_L) == 0:
					x0 = -l_map/2
				else:
					last_car_state = car_list_L[-1].get_state()
					x0 = min(-l_map/2, last_car_state.x - 5)
					front_num = last_car_state.car_num

			# Initial each car object
			car = Vehicle(car_num, lane, t0, x0, end_pose, front_num, True)

			car_list.append(car)
			lane_string += lane

			if lane == "U":
				car_list_U.append(car)
			else:
				car_list_L.append(car)

			t0_list.append(t0)
			x0_list.append(x0)
			lane_list.append(lane)
			end_pose_list.append(end_pose)

			state = car.get_state()
			goal_list.append(state.goal)
			end_spot_list.append(state.end_spot)

			front_num_list.append(front_num)

		# Save the variables
		with open('init_data.pickle', 'w') as f:
			pickle.dump([t0_list, x0_list, lane_list, \
						end_pose_list, \
						end_spot_list, \
						front_num_list], f)

		print("Init Data Saved.")

		# file_name = 'random_' + str(int(time.time())) + '.pickle'
		# # Save the variables for random assign
		# with open(file_name, 'w') as f:
		# 	pickle.dump([t0_list, lane_list, \
		# 				end_pose_list, \
		# 				goal_list, end_spot_list], f)
	else:
		# If we need to recover the last settings
		with open('init_data.pickle') as f:
			t0_list, x0_list, lane_list, \
			end_pose_list, end_spot_list, \
			front_num_list = pickle.load(f)

		for car_num in range(total_number):
			# Initial each car object
			car = Vehicle(car_num, lane_list[car_num], \
				t0_list[car_num], x0_list[car_num], \
				end_pose_list[car_num], \
				front_num_list[car_num], True)

			car_list.append(car)
			lane_string += lane_list[car_num]

	car_init_pub.publish(lane_string)

	return car_list

def init_occupancy(rand_occupy):
	occup_init_pub = rospy.Publisher('occup_init', Int16MultiArray, queue_size = 10)

	spots_U = np.zeros((2, 22), dtype=int)
	spots_L = np.zeros((2, 22), dtype=int)

	# If using the fixed occupancy
	if not rand_occupy:
		loc_occupied = [5, 8, 9, 14, 16, 17, 19, 20, 21]
		for x in loc_occupied:
			spots_U[0, x] = 1

		loc_occupied = [5, 8, 9, 11, 12, 14, 15, 17, 20, 21]
		for x in loc_occupied:
			spots_U[1, x] = 1

		loc_occupied = [0, 2, 5, 6, 8, 12, 14, 15]
		for x in loc_occupied:
			spots_L[0, x] = 1

		loc_occupied = [0, 1, 2, 3, 4, 6, 10, 11, 12, 13, 17, 18, 20]
		for x in loc_occupied:
			spots_L[1, x] = 1

	# If randomly generating occupancy
	else:
		# Totally 88 spots and 4 spots at the upper right corner
		# are fixed since it is hard for maneuver after turning
		spots_U[0:2, 20:22] = 1
		num_occupied = 84 - total_number
		loc_occupied = random.sample(range(84), num_occupied)
		for x in loc_occupied:
			if x<= 19:
				spots_U[0, x] = 1
			elif 19<x and x<= 39:
				spots_U[1, x-20] = 1
			elif 39<x and x<=61:
				spots_L[0, x-40] = 1
			else:
				spots_L[1, x-62] = 1

	pub_data = Int16MultiArray()
	pub_data.data = list(spots_U.flat)
	pub_data.data += list(spots_L.flat)
	occup_init_pub.publish(pub_data)

	return spots_U, spots_L


def random_start():
	# The arrival time for car is exponentially distributed
	# And round to 0.1 
	dt = round(np.random.exponential(exp_beta), 1)
	print("Exp distr with beta = %d" % exp_beta)
	# dt = random.randint(1,10)


	# Choose the lane randomly
	if random.random() >= 0.5:
		lane = "U"
	else:
		if lane_open == 2:
			lane = "L"
		else:
			lane = "U"

	# Choose the end pose randomly
	if random.random() >= 0.5:
		end_pose = "F"
	else:
		end_pose = "R"

	return dt, lane, end_pose


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		