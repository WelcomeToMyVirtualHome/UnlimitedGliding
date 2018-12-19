#!bin/usr
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

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
		self.thermals = [[y,x] for y in range(self.n) for x in range(self.n) if np.random.random() < self.rho]
		self.thermals = np.asarray(self.thermals)
		
	def init_drones(self):
		randint = np.random.randint(low=0,high=len(self.thermals),size=self.n_drones)
		self.drones = [[self.thermals[randint[d]][0], self.thermals[randint[d]][1]] for d in range(self.n_drones)]
		self.drones = np.asarray(self.drones)
		self.known_thermals = self.drones

	def search_termal(pos,known_thermals,dy,dx):
		found = []
		for y in range(-dy,dy):
			for x in range(-dx,dx):
				pos_y = (pos[0]+dy)%self.n
				pos_x = (pos[1]+dx)%self.n
				if known_thermals[pos_y,pos_x] == (255,0,0,255):
					found.append([pos_y,pos_x])
		return found

	def move_drones(self): 
		dX = np.random.randint(low=0, high=self.max_move + 1,size=self.n_drones)
		dY = np.random.randint(low=0, high=self.max_move + 1,size=self.n_drones)
		uniform = np.random.random(size=self.n_drones)
		is_in_rectangle = lambda drone, dx, dy, bound, thermal: (((drone[1] + dx) % bound) >= thermal[1] or ((drone[1] - dx) % bound) <= thermal[1]) and (((drone[0] + dy) % bound) >= thermal[0] or ((drone[0] - dy) % bound) <= thermal[0])	
		for d,drone in enumerate(self.drones):
			if uniform[d] < np.exp(-max(dX[d],dY[d])):		
				self.drones[d][0] = (drone[0] + dY[d]) % self.n
				self.drones[d][1] = (drone[1] + dX[d]) % self.n
				for thermal in self.thermals:
					if drone[1] == thermal[1] and drone[0] == thermal[0]:
						self.known_thermals[d][1] = thermal[1]
						self.known_thermals[d][0] = thermal[0]
						break
				else:
					can_go = [thermal for thermal in self.known_thermals if is_in_rectangle(drone,dX[d],dY[d],self.n,thermal)]
					thermal = can_go[np.random.randint(0,len(can_go))]
					self.drones[d][1] = thermal[1]
					self.drones[d][0] = thermal[0]		
					self.known_thermals[d][1] = thermal[1]
					self.known_thermals[d][0] = thermal[0]
	
	def loop(self):
		fig = plt.figure(figsize=(12, 12))
		ax = fig.gca()
		plt.tight_layout()
		
		self.time_text = ax.text(0, 0, '', transform=ax.transAxes, color='w')
		self.iterate = 0
		
		self.base_img = np.zeros((self.n,self.n,4),dtype=np.uint8)
		self.base_img[:,:,3] = 255
		self.base_img[tuple(self.thermals.T)] = (255,0,0,255)
		base = plt.imshow(self.base_img,animated=True)
			
		self.im = plt.imshow(self.base_img,animated=True)

		_ = FuncAnimation(fig, self.update, interval=0, blit=True)
		plt.show()
			
	def update(self,frame_number):
		mask = np.zeros(self.base_img.shape,dtype=np.uint8)
		mask[tuple(self.drones.T)] = (0,0,255,255)
		mask = self.base_img + mask
		self.im.set_data(mask)
		self.move_drones()
		self.iterate = self.iterate + 1
		self.time_text.set_text("{:d}".format(self.iterate))
		return [self.im, self.time_text]
	
if __name__ == '__main__':
	n = 40	
	n_therm = 400
	n_drones = 100
	simulation = Simulation(n, n_therm, n_drones)
	simulation.init_all()
	simulation.loop()