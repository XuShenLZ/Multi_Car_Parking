import pickle
import scipy
import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

with open('trajectory.pickle', 'rb') as f:
	time, states, inputs = pickle.load(f)

class SingleVehicle(object):
	"""docstring for SingleVehicle"""
	def __init__(self, car_num, time):
		super(SingleVehicle, self).__init__()
		self.car_num = car_num
		self.key_t   = time
		self.key_x   = []
		self.key_y   = []
		self.key_psi = []
		self.key_rot = None

		self.interp_t   = []
		self.interp_x   = []
		self.interp_y   = []
		self.interp_psi = []
		self.interp_rot = None

	def transform(self, offset):
		# ========= Body frame transform =====
		for i in range(len(self.key_x)):
			x   = self.key_x[i]
			y   = self.key_y[i]
			psi = self.key_psi[i]

			Rot = np.array([[np.cos(psi), -np.sin(psi)], \
							[np.sin(psi),  np.cos(psi)]])

			# Current position
			x_cur = np.array([x, y])

			# Compute Center of the Car
			body_offset = np.array([4.7/2 - 1, 0])
			centerCar = x_cur + np.dot(Rot, body_offset)
			self.key_x[i] = centerCar[0]
			self.key_y[i] = centerCar[1]

		# ========= Curve the queue ==========
		r_lower = 6.5
		r_upper = 9.5

		arc_lower = r_lower * np.pi / 2.
		arc_upper = r_upper * np.pi / 2.


		for i in range(len(self.key_x)):
			x = self.key_x[i]
			y = self.key_y[i]
			# If the curving is needed
			if x < -33:
				s = -33. - x
				if y == -1.5:
					# The lower lane
					ratio = s / arc_lower
					if ratio < 1:
						# On the arc
						angle = ratio * np.pi/2
						self.key_x[i]   = -33 - r_lower*np.sin(angle)
						self.key_y[i]   = -8  + r_lower*np.cos(angle)
						self.key_psi[i] = angle
					else:
						# Outside of the arc
						self.key_x[i]   = -33 - r_lower
						self.key_y[i]   = -8  - (s - arc_lower)
						self.key_psi[i] = np.pi/2
				elif y == 1.5:
					# The upper lane
					ratio = s / arc_upper
					if ratio < 1:
						# On the arc
						angle = ratio * np.pi/2
						self.key_x[i]   = -33 - arc_upper*np.sin(angle)
						self.key_y[i]   = -8  + arc_upper*np.cos(angle)
						self.key_psi[i] = angle
					else:
						# Outside of the arc
						self.key_x[i]   = -33 - arc_upper
						self.key_y[i]   = -8  - (s - arc_upper)
						self.key_psi[i] = np.pi/2

		# ========== Transform into carla frame =====

		# Transform the coordinates to the new system
		new_x   = [v + offset[0] for v in self.key_y]
		new_y   = [v + offset[1] for v in self.key_x]
		new_psi = [v + offset[2] for v in self.key_psi]

		self.key_x   = new_x
		self.key_y   = new_y
		self.key_psi = new_psi

	def scipy_rot(self):
		# Creay Scipy Rotation instance
		self.key_rot = R.from_euler('z', self.key_psi, degrees=False)


	def interpolate(self, new_time):
		fx    = interpolate.interp1d(self.key_t, self.key_x)
		fy    = interpolate.interp1d(self.key_t, self.key_y)
		slerp = Slerp(self.key_t, self.key_rot)

		self.interp_t   = new_time
		self.interp_x   = fx(new_time)
		self.interp_y   = fy(new_time)
		self.interp_rot = slerp(new_time)
		self.interp_psi = self.interp_rot.as_euler('xyz', degrees=False)[:, 2]

class Fleet(object):
	"""docstring for Fleet"""
	def __init__(self, time, states, inputs):
		super(Fleet, self).__init__()
		self.vehicle_list = []

		self.time   = time
		self.states = states
		self.inputs = inputs

		self.num_car = len(states[0])

		# Assign trajectory value for all vehicles
		for id_c in range(self.num_car):
			car = SingleVehicle(id_c, self.time)
			for t in range(len(self.time)):
				car.key_x.append(self.states[t][id_c]['x'])
				car.key_y.append(self.states[t][id_c]['y'])
				car.key_psi.append(self.states[t][id_c]['psi'])

			self.vehicle_list.append(car)

	def transform_forall(self, offset):
		for car in self.vehicle_list:
			car.transform(offset)

	def interp_forall(self, new_time):
		for car in self.vehicle_list:
			car.scipy_rot()
			car.interpolate(new_time)


offset = [281.71-5.5, -239.9-(-33), -np.pi/2 - 0]

# New time interval
dt = 0.04
new_time = np.arange(time[0], time[-1], dt)

fleet = Fleet(time, states, inputs)

fleet.transform_forall(offset)

fleet.interp_forall(new_time)

plt.figure(1)
plt.plot(time, fleet.vehicle_list[1].key_x)
plt.plot(new_time, fleet.vehicle_list[1].interp_x, '--')

plt.figure(2)
plt.plot(fleet.vehicle_list[1].key_x, fleet.vehicle_list[1].key_y)
plt.axis('equal')
plt.show()