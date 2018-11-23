#!bin/usr
import numpy as np
import matplotlib.pyplot as plt

class Drone:
	def __init__(self,max_move):
		self.x = 0
		self.y = 0
		self.max_move = max_move
	
	def __init__(self,x,y,max_move):
		self.x = x
		self.y = y
		self.max_move = max_move
	
	def set_pos(self, x,y):
		self.x = x
		self.y = y

	def move(self, dx, dy, bound):
		self.x = (self.x + dx) % bound
		self.y = (self.y + dy) % bound
		self.in_thermal = False
	
class Thermal:
	def __init__(self, x,y):
		self.x = x
		self.y = y

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
		self.max_move = 2

	def init_all(self):
		self.init_thermals()
		self.init_drones()

	def init_thermals(self):
		for x in range(n):
			for y in range(n):
				if np.random.random() < self.rho:
					self.thermals.append(Thermal(x,y))
				else:
					continue
			else:
				continue

	def init_drones(self):
		for d in range(self.n_drones):
			thermal = self.thermals[np.random.randint(0,len(self.thermals))]
			drone = Drone(thermal.x, thermal.y, self.max_move)
			self.known_thermals.append(thermal)
			self.drones.append(drone)

	def move_drones(self): 
		dX = np.random.randint(low=0, high=self.max_move + 1,size=self.n_drones)
		dY = np.random.randint(low=0, high=self.max_move + 1,size=self.n_drones)
		uniform = np.random.random(size=self.n_drones)
		is_in_rectangle = lambda drone, dx, dy, bound, thermal: (((drone.x + dx) % bound) >= thermal.x or ((drone.x - dx) % bound) <= thermal.x) and (((drone.y + dy) % bound) >= thermal.y or ((drone.y - dy) % bound) <= thermal.y)	
		for d in range(self.n_drones):
			dx = dX[d]
			dy = dY[d]
			if uniform[d] < np.exp(-max(dx,dy)):		
				self.drones[d].move(dx,dy,self.n)
				found = False
				for thermal in self.thermals:
					if self.drones[d].x == thermal.x and self.drones[d].y == thermal.x:
						self.known_thermals[d].x = thermal.x
						self.known_thermals[d].y = thermal.y
						found = True
						print("NEW THERMAL=[{:d},{:d}]".format(thermal.x,thermal.y))
						break
					else:
						continue
				if found:
					return
				can_go = [thermal for thermal in self.known_thermals if is_in_rectangle(self.drones[d],dx,dy,self.n,thermal)]
				thermal = can_go[np.random.randint(0,len(can_go))]
				print("KNOWN THERMAL=[{:d},{:d}]".format(thermal.x,thermal.y))
				self.drones[d].x = thermal.x
				self.drones[d].y = thermal.y

	def loop(self):
		fig = plt.figure(figsize=(15, 15))
		ax = fig.gca()
		ticks = np.arange(0.5, self.n + 0.5, 1)
		tick_labels = np.arange(1, self.n + 1, 1)
		point_size = 400
		ax.set_xlim((0-0.5,self.n-0.5))
		ax.set_ylim((0-0.5,self.n-0.5))
		ax.set_axisbelow(True)
		ax.set_xticks(ticks)
		ax.set_yticks(ticks)
		ax.set_xticklabels(tick_labels)
		ax.set_yticklabels(tick_labels)
		ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
		plt.tight_layout()
		while True:	
			x = [thermal.x for thermal in self.thermals]
			y = [thermal.y for thermal in self.thermals]
			thermals = ax.scatter(x,y,s=point_size,color='red')
			x = [drone.x for drone in self.drones]
			y = [drone.y for drone in self.drones]
			drones = ax.scatter(x,y,s=point_size,color='black')
			self.move_drones()
			plt.pause(0.000001)
			thermals.remove()
			drones.remove()
		plt.show()
			
n = 40	
n_therm = 1000
n_drones = 200
simulation = Simulation(n, n_therm, n_drones)
simulation.init_all()
simulation.loop()


