#!bin/usr
import numpy as np
import matplotlib.pyplot as plt

class Drone:
	def __init__(self,max_move):
		self.x = 0
		self.y = 0
		self.max_move = max_move
		self.in_thermal = True

	def __init__(self,x,y,max_move):
		self.x = x
		self.y = y
		self.max_move = max_move
		self.in_thermal = True

	def set_pos(self, x,y):
		self.x = x
		self.y = y

	def move(self, dx, dy, bound):
		self.x = (self.x + dx) % bound
		self.y = (self.y + dy) % bound
		self.in_thermal = False
	
	def move_to_thermal(self, dx, dy, bound, known_thermals):
		self.move(dx,dy,bound)
		for thermal in known_thermals:
			if self.x == thermal.x and self.y == thermal.x:
				self.in_thermal = True
				break
		
	def find_thermal(self, dx, dy, bound, known_thermals):
		if self.in_thermal:
			return known_thermals
		can_go = []
		is_in_rectangle = lambda self, dx, dy, bound, thermal: (((self.x + dx) % bound) >= thermal.x or ((self.x - dx) % bound) <= thermal.x) and (((self.y + dy) % bound) >= thermal.y or ((self.y - dy) % bound) <= thermal.y)	
		for thermal in known_thermals:
			if is_in_rectangle(self,dx,dy,bound,thermal):
				can_go.append(thermal)
		if len(can_go) == 0:
			return known_thermals		
		thermal = known_thermals[np.random.randint(0,len(can_go))]
		self.x = thermal.x
		self.y = thermal.y
		self.in_thermal = True
		known_thermals.append(Thermal(self.x,self.y,thermal.H))
		known_thermals.remove(thermal)
		return known_thermals

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
		self.thermals = []
		self.known_thermals = []
		self.drones = []
		self.max_move = 5

	def init_all(self):
		self.init_thermals(10)
		self.init_drones()

	def init_thermals(self, H):
		for x in range(n):
			for y in range(n):
				if np.random.random() < self.rho:
					self.thermals.append(Thermal(x,y,H))
				else:
					continue
			else:
				continue

	def init_drones(self):
		for d in range(self.n_drones):
			thermal = self.thermals[np.random.randint(0,len(self.thermals))]
			self.known_thermals.append(thermal)
			drone = Drone(thermal.x, thermal.y, self.max_move)
			self.drones.append(drone)

	def plot(self, fig):
		plt.scatter(x, y, s=10, c="b", alpha=0.5)
		plt.show()

	def move_drones_first(self, dx, dy):
		for drone in self.drones:
			if np.random.random() < np.exp(-max(dx,dy)):
				drone.move_to_thermal(dx, dy, self.n, self.known_thermals)
	
	def move_drones_second(self, dx, dy):
		for drone in self.drones:
			self.known_thermals = drone.find_thermal(dx, dy, self.n, self.known_thermals)

	def loop(self):
		fig = plt.figure(figsize=(15, 15))
		ax = fig.gca()
		ticks = np.arange(0.5, self.n + 0.5, 1)
		tick_labels = np.arange(1, self.n + 1, 1)
		point_size = 200
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
			
			dx = np.random.randint(0, self.max_move + 1)
			dy = np.random.randint(0, self.max_move + 1)
			self.move_drones_first(dx,dy)
			self.move_drones_second(dx,dy)
			
			ax.set_xlim((0-0.5,self.n-0.5))
			ax.set_ylim((0-0.5,self.n-0.5))
			ax.set_axisbelow(True)
			ax.set_xticks(ticks)
			ax.set_yticks(ticks)
			ax.set_xticklabels(tick_labels)
			ax.set_yticklabels(tick_labels)
			ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
			plt.pause(0.0001)
			plt.tight_layout()
		plt.show()
			
n = 40	
n_therm = 400
n_drones = 100
simulation = Simulation(n, n_therm, n_drones)
simulation.init_all()
simulation.loop()


