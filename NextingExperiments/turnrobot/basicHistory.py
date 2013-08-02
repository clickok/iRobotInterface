"""
Code for reading in a data file containing sensor values and performing
analysis on the sensor values over a period of time. 

Data files formatted as:
0|1 0|1 0|1 0|1 
0|1 0|1 0|1 0|1 
...
0|1 0|1 0|1 0|1 

That is, with each line containing four values which can be zero or one.
"""

import matplotlib.pyplot as plt
import numpy as np
import os
import sys



def basicHistory(data,h,alpha):
    # Determine size of feature vector and initialize weights, traces
    shape = data.shape
    fsize  = data.shape[1]*h
    w = np.zeros((fsize,))
    e = np.zeros((fsize,))
    pred = np.zeros(len(data))
    
    alpha = alpha/h

    F = np.zeros(fsize)
    for i in range(len(data)):
        obs   = data[i]
        R     = getReward(obs)
        newF  = np.append(obs,F)[:fsize]

        pred[i] = np.dot(w,F)
        alpha = getAlpha(alpha,F,i)
        gamma = getGamma()
        delta = alpha*(R + gamma*np.dot(newF,w) - np.dot(F,w))
    

        # Update weights
        w += delta*F
        
        # Copy newF to F
        np.copyto(F,newF)
    return [w,pred]

def getReward(obs):
    return obs

def getAlpha(alpha,F,i):
    return alpha

def getLambda():
    return 0

def getGamma():
    return 0.85



################################################################################
#                                Testing
################################################################################

data = np.loadtxt("turnrobot.dat")[:,3:]
    
def main():
    if len(sys.argv) < 2:
        pass
    else:
        # Load the data from the file
        # Could change the data type for longer runs
        data = np.loadtxt(sys.argv[1]) 

if __name__ == "__main__":
    main()
