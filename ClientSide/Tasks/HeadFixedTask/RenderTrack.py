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

        plt.ion() # "interactive on"
        self.fig, self.ax = plt.subplots(1,1)

        # define the ring of the track
        c_outer = Circle([0,0],radius=radius*(1+thickness/2), zorder=0, alpha=0.5, fc='gray')
        c_inner = Circle([0,0],radius=radius*(1-thickness/2), zorder=0.5, fc='white')
        self.ax.add_artist(c_outer)
        self.ax.add_artist(c_inner)
        lim = radius*(1+2*thickness)
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_aspect('equal')

        # make a circle which will represent the mouse
        self.mouse = Circle([radius,0], radius=radius*thickness/2*0.9,
                            zorder=1, facecolor='royalblue')
        self.ax.add_artist(self.mouse)
        self.fig.show()

        self.ax.invert_xaxis() # make the rendered angles go clockwise...

        plt.pause(0.1)


    def add_zone(self, theta1, theta2, fillcolor='wheat', edgecolor=None, width=1.0, alpha=0.5, hatch=None):
        w = self.radius*0.2*width
        r = self.radius + w/2
        th1 = theta1 / np.pi * 180
        th2 = theta2 / np.pi * 180
        print('New zone: {} {} ({})'.format(th1, th2, fillcolor))
        fill = True if fillcolor else False
        zone = Wedge([0,0], r, th1, th2, width=w, zorder=0.9, 
                      facecolor=fillcolor, edgecolor=edgecolor,
                      fill=fill, alpha=alpha, hatch=hatch)
        self.ax.add_artist(zone)
        self.zones.append(zone)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.1)

    def move_mouse(self, theta):
        self.mouse.center = ( self.radius * math.cos(theta),
                            self.radius * math.sin(theta) )
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

