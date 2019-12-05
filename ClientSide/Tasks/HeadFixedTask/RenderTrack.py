import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Wedge
import math

class RenderTrack:
    def __init__(self, radius=10, thickness=0.2):
        self.radius = radius
        print('Hello: ', self.radius)
        self.zones = []

        plt.ion()
        self.fig, self.ax = plt.subplots(1,1)

        c_outer = Circle([0,0],radius=radius*(1+thickness/2), zorder=0, alpha=0.5, fc='gray')
        c_inner = Circle([0,0],radius=radius*(1-thickness/2), zorder=0.5, fc='white')
        self.ax.add_artist(c_outer)
        self.ax.add_artist(c_inner)
        lim = radius*(1+2*thickness)
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_aspect('equal')

        self.mouse = Circle([0, radius], radius=radius*thickness/2*0.9,
                            zorder=1, facecolor='royalblue')
        self.ax.add_artist(self.mouse)
        self.fig.show()
        plt.pause(0.1)


    def add_zone(self, theta1, theta2, fillcolor='wheat', width=1.0, alpha=0.5):
        w = self.radius*0.2*width
        r = self.radius + w/2
        zone = Wedge([0,0], r, theta1, theta2, width=w, zorder=0.9, 
                      facecolor=fillcolor, alpha=alpha)
        self.ax.add_artist(zone)
        self.zones.append(zone)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.1)

    def move_mouse(self, theta):
        self.mouse.center = ( self.radius * math.sin(theta),
                            self.radius * math.cos(theta) )
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

