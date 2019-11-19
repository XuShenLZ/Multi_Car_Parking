import pickle
import scipy
import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

with open('trajectory.pickle', 'rb') as f:
	time, states, inputs = pickle.load(f)

class Vehicle(object):
	"""docstring for Vehicle"""
	def __init__(self, car_num, time):
		super(Vehicle, self).__init__()
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
		# Transform the coordinates to the new system
		new_x   = [ v + offset[0] for v in self.key_y]
		new_y   = [-v + offset[1] for v in self.key_x]
		new_psi = [ v + offset[2] for v in self.key_psi]

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

num_car = len(states[0])

car_list = []

offset = [281.71-5.5, -239.9-33, -np.pi/2 - 0]

# New time interval
dt = 0.04
new_time = np.arange(time[0], time[-1], dt)

for x in range(num_car):
	car = Vehicle(x, time)
	for t in range(len(time)):
		car.key_x.append(states[t][car.car_num]['x'])
		car.key_y.append(states[t][car.car_num]['y'])
		car.key_psi.append(states[t][car.car_num]['psi'])

	car.transform(offset)
	car.scipy_rot()

	car.interpolate(new_time)

	car_list.append(car)

# print('previous x', car.key_psi)
# print('interpolated x', car.interp_x)

plt.plot(time, car.key_psi)
plt.plot(new_time, car.interp_psi, '--')
plt.show()