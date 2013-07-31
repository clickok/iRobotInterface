"""
Functions for gathering data and processing it into numpy arrays 
"""

import numpy as np
import os
import re


def findBadTimesteps(filename,tmin=0,tmax=1):
    with open(filename,"r") as f:
        lines   = f.readlines()
        tstamps = [x.split()[1].split(".") for x in lines if x[0] != "#"]
        times   = [int(x[0])+0.000001*int(x[1]) for x in tstamps]
        deltas  = [times[i+1]-times[i] for i in range(len(times)-1)]
        for i in range(len(deltas)):
            if tmin < deltas[i] < tmax:
                continue
            else:
                print(i,deltas[i])
        
    
