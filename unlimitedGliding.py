#!bin/usr
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class Drone:
	def __init__(self,x,y,max_move):
		self.x = x
		self.y = y
		self.max_move = max_move

	def move(self, dx, dy, bound):
		self.x = (self.x + dx) % bound
		self.y = (self.y + dy) % bound
	
class Thermal:
	def __init__(self,x,y):
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
		self.init_thermals()
		self.init_drones()

	def init_thermals(self):
		self.thermals = [Thermal(x,y) for y in range(self.n) for x in range(self.n) if np.random.random() < self.rho]
		
	def init_drones(self):
		randint = np.random.randint(low=0,high=len(self.thermals),size=self.n_drones)
		self.drones = [Drone(self.thermals[randint[d]].x, self.thermals[randint[d]].y, self.max_move) for d in range(self.n_drones)]
		self.known_thermals = [Thermal(self.thermals[randint[d]].x, self.thermals[randint[d]].y) for d in range(self.n_drones)]
		
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
				for thermal in self.thermals:
					if self.drones[d].x == thermal.x and self.drones[d].y == thermal.y:
						self.known_thermals[d].x = thermal.x
						self.known_thermals[d].y = thermal.y
						return
					else:
						continue
				can_go = [thermal for thermal in self.known_thermals if is_in_rectangle(self.drones[d],dx,dy,self.n,thermal)]
				thermal = can_go[np.random.randint(0,len(can_go))]
				self.drones[d].x = thermal.x
				self.drones[d].y = thermal.y
				self.known_thermals[d].x = self.drones[d].x
				self.known_thermals[d].y = self.drones[d].y		

	def loop(self):
		self.fig = plt.figure(figsize=(12, 12))
		self.point_size = 20
		ax = self.fig.gca()
		ax.set_xlim((0-0.5,self.n-0.5))
		ax.set_ylim((0-0.5,self.n-0.5))
		ticks = np.arange(0.5, self.n + 0.5, 1)
		tick_labels = np.arange(1, self.n + 1, 1)
		ax.set_axisbelow(True)
		ax.set_xticks(ticks)
		ax.set_yticks(ticks)
		ax.set_xticklabels(tick_labels)
		ax.set_yticklabels(tick_labels)
		ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
		plt.tight_layout()
		self.thermals_scat, = ax.plot([],[],marker="s",markersize=self.point_size,color='red',linewidth=0)
		self.known_thermals_scat, = ax.plot([],[],marker="s",markersize=self.point_size - 5,color='blue',linewidth=0)
		self.drones_scat, = ax.plot([],[],marker="o",markersize=self.point_size - 10,color='black',linewidth=0)
		self.time_template = 'dt = %.4fs'
		self.time_text = ax.text(0, 1, '', transform=ax.transAxes)
		animation = FuncAnimation(self.fig,self.update)
		plt.show()

	def update(self,frame_number):
		start = time.time()
		self.thermals_scat.set_xdata([obj.x for obj in self.thermals])
		self.thermals_scat.set_ydata([obj.y for obj in self.thermals])
		self.known_thermals_scat.set_xdata([obj.x for obj in self.known_thermals])
		self.known_thermals_scat.set_ydata([obj.y for obj in self.known_thermals])
		self.drones_scat.set_xdata([obj.x for obj in self.drones])
		self.drones_scat.set_ydata([obj.y for obj in self.drones])
		self.move_drones()
		self.time_text.set_text(self.time_template % (time.time() - start))
			
n = 40	
n_therm = 400
n_drones = 100
simulation = Simulation(n, n_therm, n_drones)
simulation.init_all()
simulation.loop()