#!bin/usr
import numpy as np
import matplotlib.pyplot as plt

class Drone:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.known_thermals = []
		self.max_move_increment = 5
	def set_pos(self, x,y):
		self.x = x
		self.y = y
	def move(self, dx, dy):
		self.x = self.x + self.max_move_increment
		self.y = self.y + self.max_move_increment
	def boundary_conditions(self, bound):
		self.x = self.x % bound
		self.y = self.y % bound
		
class Thermal:
	def __init__(self, x,y,H):
		self.x = x
		self.y = y
		self.H = H
	def set_pos(self, x,y):
		self.x = x
		self.y = y

class Simulation:
	def __init__(self, n, n_therm, n_drones):
		self.n = n
		self.n_therm = n_therm
		self.n_drones = n_drones
		self.rho = float(n_therm) / n ** 2 
		self.field = np.zeros((self.n, self.n))
		self.thermals = []
		self.drones = []
	
	def init_all(self):
		self.init_thermals(10)
		self.init_drones()

	def init_thermals(self, H):
		for x in range(len(self.field)):
			for y in range(len(self.field[x])):
				if np.random.random() < self.rho:
					self.thermals.append(Thermal(x,y,H))
					self.field[x][y] = 1
				else:
					self.field[x][y] = 0		  

	def init_drones(self):
		for d in range(self.n_drones):
			self.drones.append(Drone())
		for drone in self.drones:
			thermal = self.thermals[np.random.randint(0,len(self.thermals))]
			drone.x = thermal.x
			drone.y = thermal.y

	def plot(self, fig):
		plt.scatter(x, y, s=10, c="b", alpha=0.5)
		plt.show()

	def move_drones(self):
		for drone in self.drones:
			dx = np.random.randint(0,drone.max_move_increment + 1)
			dy = np.random.randint(0,drone.max_move_increment + 1)
			if np.random.random() < np.exp(-max(dx,dy)):
				drone.move(dx, dy) 
				drone.boundary_conditions(self.n)

	def loop(self):
		fig = plt.figure(figsize=(15, 15))
		ax = fig.gca()
		ticks = np.arange(0, self.n, 1)
		point_size = 50
		while True:
			ax.clear()
			x = []
			y = []
			for thermal in self.thermals:
				x.append(thermal.x)
				y.append(thermal.y)
			ax.scatter(x,y,s=point_size,c="r")
			x = []
			y = []
			for drone in self.drones:
				x.append(drone.x)
				y.append(drone.y)
			ax.scatter(x,y,s=point_size,c="b")
			
			self.move_drones()
			

			ax.set_xlim((0-0.5,self.n-0.5))
			ax.set_ylim((0-0.5,self.n-0.5))
			ax.set_axisbelow(True)
			ax.set_xticks(ticks, minor=False)
			ax.set_yticks(ticks, minor=False)
			ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
			plt.pause(0.01)
			plt.tight_layout()
		plt.show()
			
n = 40	
n_therm = 400
n_drones = 100
simulation = Simulation(n, n_therm, n_drones)
simulation.init_all()
simulation.loop()


