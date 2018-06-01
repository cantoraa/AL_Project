# -*- coding: utf-8 -*-
"""
Created on Mon May 28 20:51:10 2018

@author: andres
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

fig = plt.figure()
ax1 = fig.add_subplot(3,2,1)
ax2 = fig.add_subplot(3,2,2)
ax3 = fig.add_subplot(3,2,3)
ax4 = fig.add_subplot(3,2,4)
ax5 = fig.add_subplot(3,2,5)
ax6 = fig.add_subplot(3,2,6)

def animate(i):
    pullData = open('evolution_prey.txt','r').read()
    dataArray = pullData.split('\n')
    headers = dataArray[0]
    xar = []
    yar1 = []
    yar2 = []
    yar3 = []
    yar4 = []
    yar5 = []
    yar6 = []
    for line in dataArray[1:]:
        if len(line)>1:
            a,b,c,d,e,f,g,h = line.split(',')
            xar.append(int(a))
            yar1.append(float(b))
            yar2.append(int(h))
            yar3.append(float(d))
            yar4.append(float(e))
            yar5.append(float(f))
            yar6.append(float(g))
    
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    ax5.clear()
    ax6.clear()
    ax1.plot(xar,yar1)
    ax1.set_title("metabolism")
    ax2.plot(xar,yar2)
    ax2.set_title("prey_count")
    ax3.plot(xar,yar3)
    ax3.set_title("max_age")
    ax4.plot(xar,yar4)
    ax4.set_title("run_speed")
    ax5.plot(xar,yar5)
    ax5.set_title("base_speed")
    ax6.plot(xar,yar6)
    ax6.set_title("vision_rad")

ani = animation.FuncAnimation(fig, animate, interval = 200)
plt.show()