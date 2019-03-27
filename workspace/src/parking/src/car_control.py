#!/usr/bin/env python

import rospy
from parking.msg import car_state, car_input
from parking.srv import maneuver
import math
import time

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
	def __init__(self, car_num, lane):
		super(Vehicle, self).__init__()
		# Lane info
		self.lane = lane
		
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

		# Speed constraints
		self.v_max   = 2.0
		# Input constraints
		self.acc_max = 0.4

		# If it arrived at the maneuver starting point
		self.parking = False

		# If the whole parking is finished
		self.terminated = False

		# Auto-create topic name for different cars
		self.state_topicName = "state_%d" % car_num
		self.input_topicName = "input_%d" % car_num

		# State publisher
		self.state_pub = rospy.Publisher(self.state_topicName, car_state, queue_size = 10)

		# # Auto-create node name for different cars
		# self.node_name = "carNode_%d" % car_num

		# # Init ROS Node
		# rospy.init_node(self.node_name, anonymous=True)

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

	def drive_straight(self, goal):
		# goal is the x value of parking starting point
		if goal - self.x >= 0.05:
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

	def drive_parking(self, maneuver_data, x0):
		# x0 is the starting x for parking maneuver
		# Calculate the offset for current starting position
		offset = x0 - maneuver_data.path.x[0]

		i = self.pIdx
		self.x     = maneuver_data.path.x[i] + offset
		self.y     = maneuver_data.path.y[i]
		self.psi   = maneuver_data.path.psi[i]
		self.v     = maneuver_data.path.v[i]
		self.delta = maneuver_data.input.delta[i]
		self.acc   = maneuver_data.input.acc[i]

		self.publish_state()

		self.pIdx += 1

		if self.pIdx == len(maneuver_data.input.acc):
			# The path following is finished
			self.terminated = True
		# time.sleep(0.1)

	def get_maneuver(self, end_spot, end_pose):
		# end_spot: "U"(Upper) or "L"(Lower)
		# end_pose: "F"(Front) or "R"(Reverse)
		rospy.wait_for_service('park_maneuver')
		try:
			maneuver_client = rospy.ServiceProxy('park_maneuver', maneuver)
			maneuver_data = maneuver_client(self.lane, end_spot, end_pose)
			return maneuver_data

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def is_terminated(self):
		return self.terminated
		

class CostMap(object):
	"""docstring for CostMap"""
	def __init__(self, length, width, grid):
		super(CostMap, self).__init__()
		# The dimension of the map
		self.length = length
		self.width  = width
		self.grid   = grid
		
		# The map dictionary
		self.map = {}

		for i in range(-self.length/2, self.length/2, self.grid):
			for j in range(-self.width/2, self.width/2, self.grid):
				self.map[(i, j)] = 0

	def reset_map(self):
		for i in range(-self.length/2, self.length/2, self.grid):
			for j in range(-self.width/2, self.width/2, self.grid):
				self.map[(i, j)] = 0

	def write_cost(self, x, y):
		x_floor = math.floor(x-2)
		y_floor = math.floor(y-2)
		x_ceil  = math.floor(x+2)
		y_ceil  = math.floor(y+2)
		for i in range(int(x_floor), int(x_ceil)):
			for j in range(int(y_floor), int(y_ceil)):
				self.map[(i, j)] = 1

	def get_cost(self, x, y):
		x_floor = math.floor(x)
		y_floor = math.floor(y)
		return self.map[(int(x_floor), int(y_floor))]


def main():
	# Auto-create node name for different cars
	node_name = "carNode"

	# Init ROS Node
	rospy.init_node(node_name, anonymous=True)

	cost_map = CostMap(24, 16, 1)

	car = Vehicle(1, "U")
	maneuver_data = car.get_maneuver("U", "F")
	goal = -4.5

	car2 = Vehicle(2, "L")
	maneuver_data2 = car2.get_maneuver("L", "R")
	goal2 = -1.5
	delay = 0

	loop_rate = 10
	rate = rospy.Rate(loop_rate)
	while not rospy.is_shutdown():
		# If the car is still running
		if not car.is_terminated():
			# If the car has not arrived at the starting point
			if not car.start_parking():
				car.drive_straight(goal)
			else:
				car.drive_parking(maneuver_data, goal)

			state = car.get_state()
			cost_map.reset_map()
			cost_map.write_cost(state.x, state.y)

		delay += 1
		if delay > 40:
			if not car2.is_terminated():
				# If the car2 has not arrived at the starting point
				if not car2.start_parking():
					state2 = car2.get_state()
					if cost_map.get_cost(state2.x+5, state2.y) == 0:
						car2.drive_straight(goal2)
				else:
					car2.drive_parking(maneuver_data2, goal2)


		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		