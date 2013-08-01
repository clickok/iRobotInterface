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

data = np.loadtxt("turnrobot.dat")

def idealPrediction(data):
    """Returns an array containing the ideal prediction for the given
       data (in this case, the rightmost bit). Based on Rupam's code"""
    gamma = 0.85
    
    T = len(data[:,3])
    pred = np.zeros(T)
    
    for i in range(T-2, -1, -1):
        pred[i] = data[i+1,3] + gamma*pred[i+1]
    return pred

def historyFeatures(data,k=1,stepsize=1):
    """Returns an array containing feature vectors for each timestep
       containing the current observations as well as additional 
       'history' of past observations. The parameter "k" specifies how 
       many additional observations to include, while stepsize determines
       how many 'steps' to take between the observations.

       Improvements: Could use np.copyto()
    """
    rows, cols = data.shape
    hcols = k*cols

    hist = np.zeros((rows,hcols))
    
    # Create the array of feature vectors
    # Default to the initial value of hist if there would be a range error
    for i in range(rows):
        for j in range(k):
            if (i-(j*stepsize)) >= 0:
                for c in range(cols):
                    hist[i,((j*cols)+c)] = data[i-(j*stepsize),c]
            else:
                continue
    return hist

def initHistoryFeature(obs,k):
    """Take an initial (1-D) observation and the number of histories 
       to remember and create a history feature vector."""
    tmpObs = np.array(obs)
    placeholder = np.zeros(k*(tmpObs.shape[0]-1))
    return np.concatenate(tmpObs,placeholder)

def newHistoryFeature(obs,F):
    """newHistoryFeature(obs[],F[]) --> newF[]
       Given two one dimensional arrays (an observation of current state
       and the previous feature vector), return a new feature vector of the 
       same length. Can be thought of as 'forgetting' the oldest state."""
    newF = np.concatenate(obs,F)[:len(F)]

def learn(reward,weight,F,newF):
    """learn(reward,weight[],F[],newF[]) --> updatedWeight[]
       Learns according to TD update rule"""
    # Set parameters (ultimately these should be either arguments or globals)
    alpha = 0.5
    gamma = 0.85
    weight = weight + alpha*(reward-gamma*np.dot(weight,newF)-np.dot(weight,F)
    return weight



def simpleTD0(data,alpha=0.5):
    """ Unfinished"""
    # Algorithm variables
    gamma = 0.85

    # Create data structures
    rows, cols = data.shape
    weights = np.zeros(cols,dtype=np.float)
    
    # Perform first iteration
    s        = data[0,:]
    sprime   = data[1,:]
    reward   = np.cumsum(s - gamma*sprime) # ?? Unsure
    delta    = alpha*(reward + gamma*np.dot(sprime,weights) - np.dot(s,weights))
    weights += delta*s
    
    # Iterate through data 
    for i in range(1,rows):
        s = sprime
        sprime = data[i,:]
        delta = alpha*(reward + gamma*np.dot(sprime,weights)-np.dot(s,weights))
        weights += delta*s
    return weights



    
def main():
    if len(sys.argv) < 2:
        pass
    else:
        # Load the data from the file
        # Could change the data type for longer runs
        data = np.loadtxt(sys.argv[1]) 

if __name__ == "__main__":
    main()
