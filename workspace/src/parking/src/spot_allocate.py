# Script that defines different spot 
# allocation strategies

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
