"""
Functions for gathering data and processing it into numpy arrays 
"""

import numpy as np
import os
import re


def findBadTimesteps(filename,tmin=0,tmax=1):
    """Finds timesteps in a (properly formatted) log file that fell outside of a 
       specified range"""
    path = os.path.expanduser(filename) # So that "~" defines home dir
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

def findBadOrdering(filename):
    """ Finds lines where the ordering (determined by the iteration number) is 
        incorrect. These could be swapped lines or  missed iterations."""
    path = os.path.expanduser(filename)
    with open(path,"r") as f:
        lines  = f.readlines()
        numLst = [int(x.split()[0]) for x in lines if (len(x) > 0 and x[0]!="#")]

def findBadLines(filename):
    """ Finds lines not satisfying our formatting conventions for log files"""
    pass




################################################################################
#                           Data printing functions
################################################################################
        
def printLineRange(filename,start,stop):
    """ Prints all lines within specified iterations """
    path = os.path.expanduser(filename) 
    with open(path,"r") as f:
        lines     = f.readlines()
        numLst    = [int(x.split()[0]) if x[0] != "#" else "#" for x in lines]
        startLine = numLst.index(start)
        stopLine  = numLst.index(stop)
        for i in range(startLine,stopLine+1):
            print(lines[i])
        
        

def printCommentLines(filename):
    """Finds and prints lines beginning with '#', signifying non-run data"""
    path = os.path.expanduser(filename)
    with open(path,"r") as f:
        lines = f.readlines()
        for line in lines:
            if len(line) > 0 and line[0] == "#":
                print(line.strip())

def printRunCount(logDirPath,param="Alpha"):
    """ Finds how many runs of each type (for the given parameter) there are 
        in the given directory """
    filenames = getLogFileNames(logDirPath)
    metaDataCount = {}
    
    commentRegex  = re.compile(".*#")
    metadataRegex = re.compile("([\w\-]*)=(\S*)")
    for name in filenames:
        with open(name,"r") as f:
            for line in f:
                if commentRegex.match(line):
                    tmp = metadataRegex.findall(line)
                    for i in tmp:
                        if i[0] == param:
                            if i[1] in metaDataCount.keys():
                                metaDataCount[i[1]] += 1
                            else:
                                metaDataCount[i[1]] = 1
    return metaDataCount


################################################################################
#                            Generally Useful
################################################################################
def getLogFileNames(logDirPath):
    """Get the names of log files in subdirectories of the given path, return
       the paths as a list"""
    lst = []
    logRegex = re.compile("logSarsa.*\.txt")
    for (dirpath, dirnames, filenames) in os.walk(logDirPath):
        for name in filenames:
            if logRegex.match(name):
                lst.append(os.path.join(dirpath,name))
    return lst



