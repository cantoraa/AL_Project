# -*- coding: utf-8 -*-
"""
Created on Tue May 29 21:41:05 2018

@author: andres
"""

import numpy as np
import matplotlib.pyplot as plt
from skimage.transform import PiecewiseAffineTransform, warp
import sys
from PIL import Image
import random
from PIL import ImageChops
from PIL import ImageFilter

def transformation(params):
    c = 0        
    for val in params:
        #print(val)
        c += val    
    return c

def transition(val, current):
    #print "current: " + str(current) + "val: " + str(val)
    if val < 0:
        return 0
    elif val == 0:
        return current
    elif val > 0:
        return 1    
    
class CellularAutomata:    
    def __init__(self, n, m, r1, r2, w1, w2, e, transformation, transition):
        self.n = n
        self.m = m
        self.r1 = r1
        self.r2 = r2
        self.w1 = w1
        self.w2 = w2
        self.e = e
        self.transformation = transformation
        self.transition = transition
        self.mat = np.zeros(shape=(n,m),dtype=int)
        self.v1_dict = {}
        self.v2_dict = {}
        
    def getNeighboorhods(self, x, y):
        v1_toR = []
        v2_toR = []
        if (x,y) in self.v1_dict:
            v1_toR = self.v1_dict[(x,y)]
            v2_toR = self.v2_dict[(x,y)]
        else:
            for i in range(self.n):
                for j in range(self.m):
                    r = self.e*(min(abs(x-i),(self.n-i)+x)) + min(abs(y-j),(self.m-j)+y)
                    if r < self.r1:
                        v1_toR.append((i,j))
                    elif r >= self.r1 and r < self.r2:
                        v2_toR.append((i,j))
            self.v1_dict[(x,y)] = v1_toR
            self.v2_dict[(x,y)] = v2_toR
        return v1_toR,v2_toR
        
    def setValue(self, x, y, val):
        self.mat[x][y] = val
        
    def getValue(self, x, y):
        return self.mat[x][y]
    
    def nextState(self):
        temp = np.zeros(shape=(self.n,self.m))
        for i in range(self.n):
            for j in range(self.m):
                v1,v2 = self.getNeighboorhods(i, j)
                #print "x: " + str(i) +  "y: " + str(j)
                #print "v1: " + str(v1)
                #print "v2: " + str(v2)
                temp[i][j] = \
                self.transition( \
                    self.w1*self.transformation(self.getAllNeighboorsVals(i,j,v1)) \
                    + self.w2*self.transformation(self.getAllNeighboorsVals(i,j,v2)), \
                    self.getValue(i,j)) 
        self.mat = temp    
    
    def getNeighboor(self, x, y, delta_x, delta_y):        
        x_d = x + delta_x
        y_d = y + delta_y
        x_d = x_d % self.n if x_d >= 0 else self.n + x_d
        y_d = y_d % self.m if y_d >= 0 else self.m + y_d
        #print "x: " + str(x) + " y: " + str(y) + " x_d: " + str(x_d) + " y_d: " + str(y_d)
        return self.getValue(x_d, y_d)    
    
    def getAllNeighboorsVals(self, x, y, v):
        neigh_vals = []
        #should I take into account my value
        #neigh_vals.append( getValue( x , y ) )
        for n in v:
            #neigh_vals.append( self.getNeighboor(x, y, delta[0], delta[1]) )
            neigh_vals.append( self.getValue(n[0], n[1]) )
        return neigh_vals

def get_rgb(data, color1, color2, n , m):
    to_return = np.zeros(shape=(n,m,3))
    for i in range(n):
        for j in range(m):
            if data[i][j]:
                #print ("should be color2")
                to_return[i][j] = color2;
                #print(to_return[i][j])
            else:
                to_return[i][j] = color1;
    return to_return       
def PIL2array(img):
    return np.array(img.getdata(),
                    np.uint8).reshape(img.size[1], img.size[0])

sysargs = sys.argv[1:]
n = 50
m = 50
iterations = 10
#self, n, m, r1, r2, w1, w2, e, transformation, transition
ca = CellularAutomata(n, m, 2, 4, 1, -0.5, float(sysargs[0]), transformation, transition)    
for i in range(n):
    for j in range(m):
        ca.setValue(i,j,1 if random.random() > 0.50 else 0)
for it in range(iterations):                
    ca.nextState()    
skin = get_rgb(ca.mat, np.array([1,1,1]), np.array([0,0,0]), n, m)
skin_name = str(sysargs[7]) + "_skin.png"
plt.imsave("new_skins/"+skin_name, skin)
image = PIL2array(Image.open("/home/andres/Desktop/Andres Cantor/AL/Images/zebra.png"))
rows, cols = image.shape[0], image.shape[1]
src_cols = np.linspace(0, cols, 20)
src_rows = np.linspace(0, rows, 10)
src_rows, src_cols = np.meshgrid(src_rows, src_cols)
src = np.dstack([src_cols.flat, src_rows.flat])[0]
dst_rows = src[:, 1] - np.cos(float(sysargs[1])*np.linspace(0, float(sysargs[2])*np.pi, src.shape[0]))*float(sysargs[3])
dst_cols = src[:, 0] - np.sin(float(sysargs[4])*np.linspace(0, float(sysargs[5])*np.pi, src.shape[0]))*float(sysargs[6])
dst = np.vstack([dst_cols, dst_rows]).T
tform = PiecewiseAffineTransform()
tform.estimate(src, dst)
out = warp(image, tform)
plt.gray()
plt.imshow(image, cmap=plt.cm.gray)
plt.axis('off')
#plt.show()

fig, ax = plt.subplots()
ax.imshow(out)
trans_name = str(sysargs[7]) + "_trans.png"
plt.imsave("new_transforms/"+trans_name, out)

im = Image.open("new_transforms/"+trans_name)
im = im.filter(ImageFilter.CONTOUR)
im2 = Image.open("new_skins/"+skin_name)
im2 = im2.resize((im.size[0]/3,im.size[1]/3))
#im = ImageChops.add(im,im2)
im.paste(im2,(im.size[0]/2 - im2.size[0]/2,im.size[1]/2 - im2.size[1]/2))
final_big_name = str(sysargs[7]) + "_big.png"
im.save("new_zebra_images/"+final_big_name)
#im3 = Image.open("/home/andres/Desktop/Andres Cantor/AL/ALProject/zebra_images/zebra01_02.png")
#rezising to the size of the other zebras
#im = im.resize(im3.size)
size = 30, 30
im.thumbnail(size, Image.ANTIALIAS)
final_name = str(sysargs[7]) + ".png"
im.save("new_zebra_images/"+final_name)
