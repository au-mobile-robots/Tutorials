# -*- coding: utf-8 -*-
"""
Created on Tue Mar 05 21:45:34 2013

@author: Mads
"""



# ------------
# User Instructions
# 
# In this problem you will implement SLAM in a 2 dimensional
# world. Please define a function, slam, which takes five
# parameters as input and returns the vector mu. This vector
# should have x, y coordinates interlaced, so for example, 
# if there were 2 poses and 2 landmarks, mu would look like:
#
#  mu =  matrix([[Px0],
#                [Py0],
#                [Px1],
#                [Py1],
#                [Lx0],
#                [Ly0],
#                [Lx1],
#                [Ly1]])
#
# data - This is the data that is generated with the included
#        make_data function. You can also use test_data to
#        make sure your function gives the correct result.
#
# N -    The number of time steps.
#
# num_landmarks - The number of landmarks.
#
# motion_noise - The noise associated with motion. The update
#                strength for motion should be 1.0 / motion_noise.
#
# measurement_noise - The noise associated with measurement.
#                     The update strength for measurement should be
#                     1.0 / measurement_noise.
#
#
# Enter your code at line 509

# --------------
# Testing
#
# Uncomment the test cases at the bottom of this document.
# Your output should be identical to the given results.

from pylab import * 
import matplotlib.pyplot as plt
from math import *
import random
import numpy as np


#===============================================================
#
# SLAM in a rectolinear world (we avoid non-linearities)
#      
# 
#===============================================================



# ------------------------------------------------
# 
# this is the robot class
# 
# our robot lives in x-y space, and its motion is
# pointed in a random direction. It moves on a straight line
# until is comes close to a wall at which point it turns
# away from the wall and continues to move.
#
# For measurements, it simply senses the x- and y-distance
# to landmarks. This is different from range and bearing as 
# commonly studied in the literature, but this makes it much
# easier to implement the essentials of SLAM without
# cluttered math
#

class robot:

    # --------
    # init: 
    #   creates robot and initializes location to 0, 0
    #

    def __init__(self, world_size = 100.0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0):
        self.measurement_noise = 0.0
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.num_landmarks = 0


    def rand(self):
        return random.random() * 2.0 - 1.0

    # --------
    #
    # make random landmarks located in the world
    #

    def make_landmarks(self, num_landmarks):
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks


    # --------
    #
    # move: attempts to move robot by dx, dy. If outside world
    #       boundary, then the move does nothing and instead returns failure
    #

    def move(self, dx, dy):

        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise

        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True
    

    # --------
    #
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #

    def sense(self):
        Z = []
        for i in range(self.num_landmarks):
            dx = self.landmarks[i][0] - self.x + self.rand() * self.measurement_noise
            dy = self.landmarks[i][1] - self.y + self.rand() * self.measurement_noise    
            if self.measurement_range < 0.0 or abs(dx) + abs(dy) <= self.measurement_range:
                Z.append([i, dx, dy])
        return Z

    # --------
    #
    # print robot location
    #

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)

######################################################

# --------
# this routine makes the robot data
#

def make_data(N, num_landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):

    complete = False

    while not complete:

        data = []

        # make robot and landmarks
        r = robot(world_size, measurement_range, motion_noise, measurement_noise)
        r.make_landmarks(num_landmarks)
        seen = [False for row in range(num_landmarks)]
    
        
        # guess an initial motion
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance
    
        for k in range(N-1):
    
            # sense
            Z = r.sense()

            # check off all landmarks that were observed 
            for i in range(len(Z)):
                seen[Z[i][0]] = True
    
            # move
            while not r.move(dx, dy):
                # if we'd be leaving the robot world, pick instead a new direction
                orientation = random.random() * 2.0 * pi
                dx = cos(orientation) * distance
                dy = sin(orientation) * distance

            # memorize data
            data.append([Z, [dx, dy]])

        # we are done when all landmarks were observed; otherwise re-run
        complete = (sum(seen) == num_landmarks)

    print (' ')
    print ('Landmarks: ', r.landmarks)
    print (r)


    return data, r.landmarks
    
####################################################

# --------------------------------
#
# print the result of SLAM, the robot pose(s) and the landmarks
#

def print_result(N, num_landmarks, result):
    print()
    print ('Estimated Pose(s):')
    for i in range(N):
        print ('    ['+ ', '.join('%.3f'%x for x in result[2*i]) + ', ' \
            + ', '.join('%.3f'%x for x in result[2*i+1]) +']')
    print()
    print ('Estimated Landmarks:')
    for i in range(num_landmarks):
        print ('    ['+ ', '.join('%.3f'%x for x in result[2*(N+i)]) + ', ' \
            + ', '.join('%.3f'%x for x in result[2*(N+i)+1]) +']')
         

# --------------------------------
#
# slam - retains entire path and all landmarks
#

def slam(data, N, num_landmarks, motion_noise, measurement_noise):
    
    #The dimension of the matrix. landmarks+movements with two coordinates
    dim=2*(num_landmarks + N)
    
    #create Omega
    Omega = np.zeros((dim,dim))
    Omega[0,0]=1
    Omega[1,1]=1
    
    #create Xi matrix   
    Xi = np.zeros((50,1))
    Xi[0,0]=world_size/2
    Xi[1,0]=world_size/2
    
    for k in range(len(data)):
        #position in matrix
        n=2*k
        
        #Extract the current measurement and motion
        meas=data[k][0]
        motion=data[k][1]
        
        for i in range(len(meas)):
            
            #get the index of the landmark in the matrix
            m=2*(N+meas[i][0])
            
            #Update the information matrix
            for b in range(2):
                Omega[n+b][n+b] += 1/measurement_noise
                Omega[m+b][m+b] += 1/measurement_noise
                Omega[n+b][m+b] += -1/measurement_noise
                Omega[m+b][n+b] += -1/measurement_noise
                Xi[n+b][0] += -meas[i][1+b] /measurement_noise
                Xi[m+b][0] += meas[i][1+b] /measurement_noise                
                
        
        # Update diagonal in information matrix
        for b in range(4):
            Omega[n+b][n+b] +=1/motion_noise
            
        for b in range(2):
            Omega[n+b][n+b+2] += -1/motion_noise
            Omega[n+b+2][n+b] += -1/motion_noise
            Xi[n+b][0] += -motion[b] / motion_noise
            Xi[n+b+2][0] += motion[b] / motion_noise            
    
    mu = np.linalg.inv(Omega) @ Xi
    
    return mu # Make sure you return mu for grading!



def plot_results(N, num_landmarks, result):
    # Plot the results
    EstLandmarksX=[]
    EstLandmarksY=[]
    LandmarksX=[]
    LandmarksY=[]
    for i in range(num_landmarks):
        EstLandmarksX.append(result[2*(N+i)])
        EstLandmarksY.append(result[2*(N+i)+1])
        LandmarksX.append(Landmarks[i][0])
        LandmarksY.append(Landmarks[i][1])
    plt.plot(EstLandmarksX,EstLandmarksY, 'bx')  
    plt.plot(LandmarksX,LandmarksY, 'bo')
        
    EstPosX=[]
    EstPosY=[]
    for i in range(N):
        EstPosX.append(result[2*i])
        EstPosY.append(result[2*i+1])
    plt.plot(EstPosX, EstPosY, 'rx')
    plt.title('red x = est. position, blue x = Est. landmarks, blue o = real landmarks')
    
    
    print ('Print estimated landmarks')
    #print 'LandX ', EstLandmarksX
    #print 'LandY ', EstLandmarksY
    #print 'posX ', EstPosX
    #print 'posY ', EstPosY
    plt.show()


        
# ------------------------------------------------------------------------
# ------------------------------------------------------------------------
# ------------------------------------------------------------------------
#
# Main routines
#


num_landmarks      = 5        # number of landmarks
N                  = 20       # time steps
world_size         = 200.0    # size of world
measurement_range  = 50.0     # range at which we can sense landmarks
motion_noise       = 2.0      # noise in robot motion
measurement_noise  = 3.0      # noise in the measurements
distance           = 20.0     # distance by which robot (intends to) move each iteratation 


#Create data
data, Landmarks = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)

#Perform SLAM
result = slam(data, N, num_landmarks, motion_noise, measurement_noise)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

#Print the estimated landmarks and the pose
print_result(N, num_landmarks, result)

#Plot the estimated landmarks and the pose
plot_results(N, num_landmarks, result)

