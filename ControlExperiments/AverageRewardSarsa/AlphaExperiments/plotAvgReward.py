import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import re

testPath = "/AlphaExperiments\ (Saskatoon)"

# Walk through subdirectories, getting all files with 
# the "logSarsa" form
def getAllLogs(parentDir):
    lst = []
    logRegex = re.compile("logSarsa.*\.txt")
    #print("Looking for log files...")
    for (dirpath, dirnames, filenames) in os.walk(parentDir):
        for name in filenames:
            if logRegex.match(name):
                #print(os.path.join(dirpath,name))
                lst.append(os.path.join(dirpath,name))
    return lst

testLst = getAllLogs(".")

# Parse a file, getting the data as well as the metadata
# Could be better done, vis-a-vis eliminating the first regex, but 
# this is reasonably clean and safe and allows us to grab the actual
# data on the same pass
def parseFile(path):
    metaData = {}
    runData = []
    metaExp = re.compile("([\w\-]*)=(\S*)")
    badSplitExp = re.compile("\.\s+")
    f = open(path,"r")
    for line in f:
        if re.match(".*#",line):
            #print(line)
            tmp = metaExp.findall(line)
            for i in tmp:
                #print(i[0],i[1])
                metaData[i[0]] = i[1]
        else:
            # This is a non-data line!
            tmp = badSplitExp.sub("",line).split()
            #print(tmp)
            runData.append([int(tmp[0]),float(tmp[1]),int(tmp[2])])
    
    return np.array(runData), metaData


# Given a bunch of log file names (stored as a list), this returns a 
# dictionary of parsed log data, indexed by the value of alpha associated
# with each run, so runs with the same alpha are grouped together (but
# the data from each run remains distinct)
def separateByAlpha(flst):
    data = {}
    alphaVals = []
    for i in flst:
        rd, md = parseFile(i)
        alpha = float(md["Alpha"])
        if alpha in alphaVals:
            data[alpha].append((rd,md))
        else:
            alphaVals.append(alpha)
            data[alpha] = [(rd,md)]
    return data


# Obtains the average of the values of a bunch of arrays, passed as
# a list "lst". Used to plot the result of multiple runs as one graph.  
# For example, if given [[1,2,2],[9,-10,4]] it would return
# --> [5,-4,3]
# It tries to ensure that if there was a bad run (perhaps
# because it was terminated early) that the run is not used to calculate
# the average values. 
def makeAvgArray(lst):
    arraySize = len(lst[0][0])
    allData = np.array(lst[0][0])
    for i in range(1,len(lst)):
        if arraySize != len(lst[i][0]):
            print("Problem with array sizes")
        else:
            allData = np.dstack((allData,lst[i][0]))
    
    avgData = allData.mean(axis=2,dtype=np.float64)
    return avgData



# Plots a series of runs, given the data for _all_ runs and an alpha
# value. (The "all runs" part is not ideal, but used here for 
# preserving metadata). Labels the graph and includes some additional info
# available from the metadata. 
def plotForAlpha(data,alpha):
    avgData = makeAvgArray(data[alpha])
    metaData = data[alpha][0][1]
    fig, ax = plt.subplots(1)
    xdata = avgData[:,0]
    ydata = np.cumsum(avgData[:,2])/avgData[:,0]

    ax.plot(xdata,ydata)
    ax.set_title("Average Reward vs. Timestep for $\\alpha = %f$"%(alpha))
    ax.set_ylabel("Average Reward")
    ax.set_xlabel("Timestep")
    
    statboxProps = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    statboxText = "\
    $\epsilon = 0.0$\n\
    $\lambda = {Lambda}$\n\
    $\\alpha_R = {Alpha-R}$\n\
    $\Delta t = {Timestep} \, (\mu s)$\n".format(**metaData)
    ax.text(0.65,0.95,statboxText,
            transform=ax.transAxes,fontsize=14,
            verticalalignment='top',
            horizontalalignment='left',
            bbox=statboxProps)

#    print("For alpha = %f, average of last 100 y-values: %f"%
#          (alpha,np.mean(ydata[-100:])))
    outputPlotName = ("logPlotAlpha-%f"%alpha).replace(".",",")
    plt.savefig(outputPlotName)

# plotEverything() uses the above functions to plot all graphs
# beginning with "logSarsa" in the directory it is called from and
# all subdirectories. 
def plotEverything():
    parentdir = "."
    # ["/dir/filename",...]
    logFileLst = getAllLogs(parentdir)
    #       = {alphaVal:[(rd1,md1),(rd2,md2),...]}
    logData = separateByAlpha(logFileLst)
    for key in logData.keys():
        plotForAlpha(logData,key)
        
    
