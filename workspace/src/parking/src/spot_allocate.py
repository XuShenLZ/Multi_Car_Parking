# Script that defines different spot 
# allocation strategies
import random

# =========== Deepest Strategy =========
# Vehicles will choose the deepest vacant 
# spot, same lane first and the other lane 
# next
def deepest(car, Map, spots_U, spots_L):
	# The number of columns
	length = spots_L.shape[1]

	if car.lane == "U":
		# The car is now at the upper lane
		row_idx = 0
		other_lane = "L"
	else:
		# The car is now at the lower lane
		row_idx = 1
		other_lane = "U"

	goal = [0,0,0]

	# Check from the upper part first
	# Check from the farest end to the closest
	for col_idx in range(length):
		# Firstly, check the current lane
		if spots_U[1-row_idx, col_idx] == 0:
			goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2

			if car.lane == "U":
				goal[1] = -Map['w_lane']/2 + Map['w_map']
			else:
				goal[1] =  Map['w_lane']/2 + Map['w_map']

			# The goal position in s coordinate
			goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
			spots_U[1-row_idx, col_idx] = 1
			return goal, car.lane
		# If occupied, check the other lane
		elif spots_U[row_idx, col_idx] == 0:
			goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2 + Map['w_map']
			else:
				goal[1] =  Map['w_lane']/2 + Map['w_map']
			
			goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
			spots_U[row_idx, col_idx] = 1
			return goal, other_lane

	# Then from the lower part
	# Check from the farest end to the closest
	for col_idx in range(length-1, -1, -1):
		# Firstly, check the current lane
		if spots_L[row_idx, col_idx] == 0:
			goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
			if car.lane == "U":
				goal[1] = Map['w_lane']/2
			else:
				goal[1] = -Map['w_lane']/2
			goal[2] = goal[0]
			spots_L[row_idx, col_idx] = 1
			return goal, car.lane
		# If occupied, check the other lane
		elif spots_L[1-row_idx, col_idx] == 0:
			goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2
			else:
				goal[1] = Map['w_lane']/2
			goal[2] = goal[0]
			spots_L[1-row_idx, col_idx] = 1
			return goal, other_lane

	# If all spaces are occupied
	print("There is no free space to allocate.")
	print(spots_U)
	print(spots_L)
	goal[0] = length * 3 - Map['l_map']/2 -1.5*Map['w_spot']
	goal[2] = goal[0]
	return goal, lane

# =========== Same Side Strategy =========
# Vehicles will choose the vacant spot at the 
# same side first, from deepest to nearest.
# If there is no spot at the same side, check 
# the other side
def same_side(car, Map, spots_U, spots_L):
	# The number of columns
	length = spots_L.shape[1]

	if car.lane == "U":
		# The car is now at the upper lane
		row_idx = 0
		other_lane = "L"
	else:
		# The car is now at the lower lane
		row_idx = 1
		other_lane = "U"

	goal = [0,0,0]

	# Check from the upper part first
	# Firstly, check the current lane
	# Check from the farest end to the closest
	for col_idx in range(length):
		if spots_U[1-row_idx, col_idx] == 0:
			goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2

			if car.lane == "U":
				goal[1] = -Map['w_lane']/2 + Map['w_map']
			else:
				goal[1] =  Map['w_lane']/2 + Map['w_map']

			# The goal position in s coordinate
			goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
			spots_U[1-row_idx, col_idx] = 1
			return goal, car.lane
		else:
			continue

	# Then, check the other lane
	# Check from the farest end to the closest
	for col_idx in range(length):
		if spots_U[row_idx, col_idx] == 0:
			goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2 + Map['w_map']
			else:
				goal[1] =  Map['w_lane']/2 + Map['w_map']
			
			goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
			spots_U[row_idx, col_idx] = 1
			return goal, other_lane
		else:
			continue

	# Then from the lower part
	# Firstly, check the current lane
	# Check from the farest end to the closest
	for col_idx in range(length-1, -1, -1):
		if spots_L[row_idx, col_idx] == 0:
			goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
			if car.lane == "U":
				goal[1] = Map['w_lane']/2
			else:
				goal[1] = -Map['w_lane']/2
			goal[2] = goal[0]
			spots_L[row_idx, col_idx] = 1
			return goal, car.lane
		else:
			continue

	# Then, check the other lane
	# Check from the farest end to the closest
	for col_idx in range(length-1, -1, -1):
		if spots_L[1-row_idx, col_idx] == 0:
			goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2
			else:
				goal[1] = Map['w_lane']/2
			goal[2] = goal[0]
			spots_L[1-row_idx, col_idx] = 1
			return goal, other_lane
		else:
			continue

	# If all spaces are occupied
	print("There is no free space to allocate.")
	print(spots_U)
	print(spots_L)
	goal[0] = length * 3 - Map['l_map']/2 -1.5*Map['w_spot']
	goal[2] = goal[0]
	return goal, lane

# =========== Same Side - n Strategy =========
# Vehicles will choose the vacant spot at the 
# same side first, from deepest to nearest.
# Occupancy will be checked every n spots
# 0,n,2n,3n,4n,... and then 1,n+1,2n+1,3n+1,4n+1,...
# Finally n-1,2n-1,3n-1,4n-1,...
# If there is no spot at the same side, check 
# the other side
def same_side_n(car, Map, spots_U, spots_L, n):
	# The number of columns
	length = spots_L.shape[1]

	if car.lane == "U":
		# The car is now at the upper lane
		row_idx = 0
		other_lane = "L"
	else:
		# The car is now at the lower lane
		row_idx = 1
		other_lane = "U"

	goal = [0,0,0]

	# Check from the upper part first
	# Firstly, check the current lane
	for itv in range(n):
		for col_idx in range(itv, length, n):
			if spots_U[1-row_idx, col_idx] == 0:
				goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2

				if car.lane == "U":
					goal[1] = -Map['w_lane']/2 + Map['w_map']
				else:
					goal[1] =  Map['w_lane']/2 + Map['w_map']

				# The goal position in s coordinate
				goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
				spots_U[1-row_idx, col_idx] = 1
				return goal, car.lane
			else:
				continue

	# Then, check the other lane
	# Check from the farest end to the closest
	for col_idx in range(length):
		if spots_U[row_idx, col_idx] == 0:
			goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2 + Map['w_map']
			else:
				goal[1] =  Map['w_lane']/2 + Map['w_map']
			
			goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
			spots_U[row_idx, col_idx] = 1
			return goal, other_lane
		else:
			continue

	# From the lower part
	# Firstly, check the current lane
	for itv in range(n):
		for col_idx in range(length-1-itv, -1, -n):
			if spots_L[row_idx, col_idx] == 0:
				goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
				if car.lane == "U":
					goal[1] = Map['w_lane']/2
				else:
					goal[1] = -Map['w_lane']/2
				goal[2] = goal[0]
				spots_L[row_idx, col_idx] = 1
				return goal, car.lane
			else:
				continue

	# Finally, check the other lane
	# Check from the farest end to the closest
	for col_idx in range(length-1, -1, -1):
		if spots_L[1-row_idx, col_idx] == 0:
			goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
			if car.lane == "U":
				goal[1] = -Map['w_lane']/2
			else:
				goal[1] = Map['w_lane']/2
			goal[2] = goal[0]
			spots_L[1-row_idx, col_idx] = 1
			return goal, other_lane
		else:
			continue

	# If all spaces are occupied
	print(car.lane)
	print("There is no free space to allocate.")
	print(spots_U)
	print(spots_L)
	goal[0] = length * 3 - Map['l_map']/2 -1.5*Map['w_spot']
	goal[2] = goal[0]
	return goal, car.lane


# =========== Deepest-n Strategy =========
# Vehicles will choose the deepest vacant 
# spot, same lane first and the other lane 
# next
# With an interval size of n-1
def deepest_n(car, Map, spots_U, spots_L, n):
	# The number of columns
	length = spots_L.shape[1]

	if car.lane == "U":
		# The car is now at the upper lane
		row_idx = 0
		other_lane = "L"
	else:
		# The car is now at the lower lane
		row_idx = 1
		other_lane = "U"

	goal = [0,0,0]

	# Check from the upper part first
	# Check from the farest end to the closest
	for itv in range(n):
		for col_idx in range(itv, length, n):
			# Firstly, check the current lane
			if spots_U[1-row_idx, col_idx] == 0:
				goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2

				if car.lane == "U":
					goal[1] = -Map['w_lane']/2 + Map['w_map']
				else:
					goal[1] =  Map['w_lane']/2 + Map['w_map']

				# The goal position in s coordinate
				goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
				spots_U[1-row_idx, col_idx] = 1
				return goal, car.lane
			# If occupied, check the other lane
			elif spots_U[row_idx, col_idx] == 0:
				goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2
				if car.lane == "U":
					goal[1] = -Map['w_lane']/2 + Map['w_map']
				else:
					goal[1] =  Map['w_lane']/2 + Map['w_map']
				
				goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
				spots_U[row_idx, col_idx] = 1
				return goal, other_lane

	# Then from the lower part
	# Check from the farest end to the closest
	for itv in range(n):
		for col_idx in range(length-1-itv, -1, -n):
			# Firstly, check the current lane
			if spots_L[row_idx, col_idx] == 0:
				goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
				if car.lane == "U":
					goal[1] = Map['w_lane']/2
				else:
					goal[1] = -Map['w_lane']/2
				goal[2] = goal[0]
				spots_L[row_idx, col_idx] = 1
				return goal, car.lane
			# If occupied, check the other lane
			elif spots_L[1-row_idx, col_idx] == 0:
				goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
				if car.lane == "U":
					goal[1] = -Map['w_lane']/2
				else:
					goal[1] = Map['w_lane']/2
				goal[2] = goal[0]
				spots_L[1-row_idx, col_idx] = 1
				return goal, other_lane

	# If all spaces are occupied
	print("There is no free space to allocate.")
	print(spots_U)
	print(spots_L)
	goal[0] = length * 3 - Map['l_map']/2 -1.5*Map['w_spot']
	goal[2] = goal[0]
	return goal, lane

# =========== Random Strategy =========
# Vehicles will choose randomly
def random_assign(car, Map, spots_U, spots_L):
	# The number of columns
	[width, length] = spots_L.shape

	goal = [0,0,0]

	random_val = random.random()

	# Choose from the upper part
	free_bool = (spots_U == 0)
	free      = spots_U[free_bool]
	size_free = free.size

	# If there is free space in the upper part
	if size_free > 0:
		for bin_loc in range(1, size_free+1):
			if 1.0/size_free * bin_loc > random_val:
				break

		counter = 0
		break_flag = False
		row_idx = 0
		col_idx = 0
		for row_idx in range(width):
			for col_idx in range(length):
				if spots_U[row_idx, col_idx] == 0:
					counter += 1
				
				# Whether reached the target
				if counter == bin_loc:
					break_flag = True
					break

			if break_flag:
				break

		goal[0] = (col_idx + 2.5 )* Map['w_spot'] - Map['l_map']/2
		if car.lane == "L":
			goal[1] = Map['w_lane']/2 + Map['w_map']
		else:
			goal[1] = -Map['w_lane']/2 + Map['w_map']

		if row_idx == 1:
			output_lane = "U"
		else:
			output_lane = "L"

		goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]

		spots_U[row_idx, col_idx] = 1
		return goal, output_lane
	
	# Choose from the lower part
	free_bool = (spots_L == 0)
	free      = spots_L[free_bool]
	size_free = free.size

	if size_free > 0:
		for bin_loc in range(1, size_free+1):
			if 1.0/size_free * bin_loc > random_val:
				break

		counter = 0
		break_flag = False
		row_idx = 0
		col_idx = 0
		for row_idx in range(width):
			for col_idx in range(length):
				if spots_L[row_idx, col_idx] == 0:
					counter += 1
				
				# Whether reached the target
				if counter == bin_loc:
					break_flag = True
					break

			if break_flag:
				break

		goal[0] = col_idx * 3 - Map['l_map']/2 - 1.5*Map['w_spot']
		if car.lane == "L":
			goal[1] = -Map['w_lane']/2
		else:
			goal[1] = Map['w_lane']/2

		if row_idx == 1:
			output_lane = "L"
		else:
			output_lane = "U"

		goal[2] = goal[0]

		spots_L[row_idx, col_idx] = 1
		return goal, output_lane

	# If all spaces are occupied
	print("There is no free space to allocate.")
	print(spots_U)
	print(spots_L)
	goal[0] = length * 3 - Map['l_map']/2 -1.5*Map['w_spot']
	goal[2] = goal[0]
	return goal, lane

# =========== Solo-n Strategy =========
# Vehicles will choose the 1, n, 2n, 3n, ...
# then 2, 1+n, 1+2n,....
# With an interval size of n-1
# same lane first and the other lane next
def solo_n(car, Map, spots_U, spots_L, spot_list, n):
	# The number of columns
	length = spots_L.shape[1]

	# if no spot_list was generated
	if len(spot_list) == 0:
		print("Spot list is generating...")
		# Loop until all spaces are assigned
		while (0 in spots_U) or (0 in spots_L):
			for itv in range(n):
				# Each starting point need to go twice
				# to clear the available spots
				for repeat in range(2):
					for col_idx in range(itv, length, n):
						# Firstly, check the current lane
						if spots_U[0, col_idx] == 0:
							spot = [0, col_idx]
							spot_list.append(spot)
							spots_U[0, col_idx] = 1
						elif spots_U[1, col_idx] == 0:
							spot = [1, col_idx]
							spot_list.append(spot)
							spots_U[1, col_idx] = 1

					for col_idx in range(length-1-itv, -1, -n):
						# Firstly, check the current lane
						if spots_L[0, col_idx] == 0:
							spot = [2, col_idx]
							spot_list.append(spot)
							spots_L[0, col_idx] = 1
						# If occupied, check the other lane
						elif spots_L[1, col_idx] == 0:
							spot = [3, col_idx]
							spot_list.append(spot)
							spots_L[1, col_idx] = 1

	spot = spot_list[car.car_num]
	goal = [0, 0, 0]

	if spot[0] == 0:
		goal[0] = (spot[1] + 2.5 )* Map['w_spot'] - Map['l_map']/2

		if car.lane == "U":
			goal[1] = -Map['w_lane']/2 + Map['w_map']
		else:
			goal[1] =  Map['w_lane']/2 + Map['w_map']

		goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
		lane = "L"

	elif spot[0] == 1:
		goal[0] = (spot[1] + 2.5 )* Map['w_spot'] - Map['l_map']/2
		if car.lane == "U":
			goal[1] = -Map['w_lane']/2 + Map['w_map']
		else:
			goal[1] =  Map['w_lane']/2 + Map['w_map']

		goal[2] = Map['l_map']/2 + car.turn_offset_x - goal[0] + car.turn[3]
		lane = "U"

	elif spot[0] == 2:
		goal[0] = (spot[1] - 1.5 )* Map['w_spot'] - Map['l_map']/2
		if car.lane == "U":
			goal[1] = Map['w_lane']/2
		else:
			goal[1] = -Map['w_lane']/2

		goal[2] = goal[0]
		lane = "U"

	elif spot[0] == 3:
		goal[0] = (spot[1] - 1.5 )* Map['w_spot'] - Map['l_map']/2
		if car.lane == "U":
			goal[1] = Map['w_lane']/2
		else:
			goal[1] = -Map['w_lane']/2

		goal[2] = goal[0]
		lane = "L"

	return goal, lane