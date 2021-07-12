import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class State:
	def __init__(self):
		self.px = 0.0 
		self.py = 0.0
		self.vx = 0.0 
	
	def propagate(self, u, dt):
		beta = np.arctan(2.0 * self.px)

		acc = u - 9.81 * np.sin(beta) - abs(self.vx) * self.vx * 1e-1
		acc_x = acc * np.cos(beta)

		self.px += self.vx * dt
		self.vx += acc_x * dt

		self.py = self.px**2


state = State()
linspace = np.linspace(-1.5, 1.5, 22)

fig, ax = plt.subplots()
ax.plot(linspace, linspace**2)
sc = ax.scatter(state.px, state.py, zorder = 3)
plt.grid()

def animate(i):
	state.propagate(3.0, 1e-2)
	sc.set_offsets([state.px, state.py])

anim = animation.FuncAnimation(fig, animate, frames = 100, interval = 10)
plt.show()
