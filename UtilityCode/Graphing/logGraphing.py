import math
import os
import re
import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


################################################################################
#                          Graph Plotting Functions
################################################################################


    
    

def plotAvgRewardVsAlpha(plotEach=True,plotMean=False,graphName=False):
    '''
    Computes the cumulative reward for each run, calculates the average of these
    cumulative rewards for all runs with the same value of alpha, and then
    saves a graph containing (depending on the options) the above reward data
    plotted vs. the value of alpha.

    Can be modified to print the cumulative reward average for only certain
    parts of the run
    '''
    parentdir  = "."
    logFileLst = getAllLogs(parentdir)
    logData    = groupByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    stepMax = None                  # Start of timesteps to sum from
    stepMin = None                  # End of timesteps to sum to
    outputDirectory = "./Graphs/"
    outputFileName  = "graphAvgRewardVsAlpha.pdf"
    ax.set_title("Average Reward Vs. Alpha")
    ax.set_xlabel("$\\alpha$")
    ax.set_ylabel("Average Reward")

    alphaLst     = sorted(logData.keys())
    xdata        = []
    ydata        = []
    ydataErr = [] 
    for alpha in alphaLst:
        if alpha < (2**-6): # Had some runs where alpha was entered wrong
            continue
        alphaLog = logData[alpha]
        runData = alphaLog[0]
        # Create an array containing the average values
        avgArray = (np.mean(runData[stepMin:stepMin,2],axis=0,dtype=np.float64))

        if plotEach:
            tmpArray = np.empty(len(avgArray)); tmpArray.fill(alpha)
            ax.scatter(tmpArray,avgArray)
            
        xdata.append(alpha)
        ydata.append(np.mean(avgArray))
        ydataErr.append(np.std(avgArray))
    if plotMean:
        ax.errorbar(xdata,ydata,ydataErr,color="red")   

    # The below limit setting is complicated, but works even for weird limits
    # (Consider encapsulating as a function)
    xmin  = min(xdata); xmin -= (abs(xmin)*0.2)
    xmax  = max(xdata); xmax += (abs(xmax)*0.2)

    ax.set_xlim([xmin,xmax])
    ax.set_xscale("log",basex=2)
    if graphName == False:
        outputDirectory = "./Graphs/"
        outputFileName  = "graphAvgRewardVsAlpha.pdf"
        plt.savefig(outputDirectory+outputFileName)
    else:
        plt.savefig(graphName)


def plotRewardVsTimestep(movingAvg=0):
    '''
    Graphs the reward obtained vs the timestep, with an optional moving average.
    Makes separate graphs for each value of alpha.
    '''
    parentdir  = "."
    logFileLst = getAllLogs(parentdir)
    logData    = groupByAlpha(logFileLst)
    outputDirectory = "./Graphs/"

    alphaLst   = sorted(logData.keys())
    if movingAvg > 0:
        movingAvg  = int(movingAvg)
        weights    = np.ones(movingAvg)
    for alpha in alphaLst:
        if alpha < (2**-6): # Had some runs where alpha was entered wrong
            continue
        fig, ax = plt.subplots(1)
        outputFileName  = "graphRewardVsTimestep_alpha-%0.4f.pdf"%alpha
        ax.set_title("Reward Obtained Vs. Timestep (for $\\alpha=%0.6f$)"%alpha)
        ax.set_xlabel("Timestep")
        ax.set_ylabel("Reward")

        alphaLog = logData[alpha]
        runData  = alphaLog[0]
        xdata = runData[:,0,0]
        ydata = np.mean(runData[:,2,:],axis=1,dtype=np.float64)
        
        # Plot the graph
        ax.plot(xdata,ydata,label="Reward obtained at timestep")

        # Plot moving average
        yavg = (np.convolve(ydata,weights,mode='valid'))/movingAvg
        offset = min(xdata)+movingAvg//2
        xavg = np.linspace(offset,offset+len(yavg),len(yavg))
        if movingAvg:
            ax.plot(xavg,yavg,color="red",
                    label="Average of last %d rewards"%movingAvg)

        # Set limits
        ax.set_xlim([min(xdata),max(xdata)])
        ax.set_ylim([-20,35])

        # Add text and legends
        leg = plt.legend(loc="upper left",bbox_to_anchor=(1,1))
        # Set figure properties
        defaultFigSize = (fig.get_size_inches())
        newFigSize = (defaultFigSize[0]+3,defaultFigSize[1])
        fig.set_size_inches(newFigSize)
        plt.savefig(outputDirectory+outputFileName,bbox_inches="tight")
    

def plotRunningAvgRewVsTimestep():
    '''
    Graphs the running average of the reward obtained vs the timestep.
    Makes separate graphs for each value of alpha.
    '''
    parentdir  = "."
    logFileLst = getAllLogs(parentdir)
    logData    = groupByAlpha(logFileLst)
    outputDirectory = "./Graphs/"

    alphaLst   = sorted(logData.keys())
    for alpha in alphaLst:
        if alpha < (2**-6): # Had some runs where alpha was entered wrong
            continue
        fig, ax = plt.subplots(1)
        outputFileName  = "graphRunningAvgRewVsTimestep_alpha-%0.4f.pdf"%alpha
        ax.set_title("Running Average Reward Vs. Timestep (for $\\alpha=%0.6f$)"%alpha)
        ax.set_xlabel("Timestep")
        ax.set_ylabel("Average Reward")

        alphaLog = logData[alpha]
        metaData = alphaLog[1]
        runData  = alphaLog[0]
        xdata = runData[:,0,0]
        ydata = np.cumsum(runData[:,2,:],axis=0,dtype=np.float64)
        yerr  = np.std(ydata,axis=1)/xdata
        print(yerr.shape)
        ydata = np.sum(ydata,axis=1,dtype=np.float64)/(xdata*ydata.shape[1])

        # Plot the graph
        ax.plot(xdata,ydata,label="Reward obtained at timestep")
        #ax.fill_between(xdata,ydata-yerr,ydata+yerr,facecolor='blue',alpha=0.5)
        # Set limits
        ax.set_xlim([min(xdata),max(xdata)])
        #ax.set_ylim([-20,35])

        # Add text and legends
        #leg = plt.legend(loc="upper left",bbox_to_anchor=(1,1))
        
        #statboxProps = dict(facecolor='white', alpha=0.5)
        #statboxText = "\
        #$\epsilon = 0.0$\n\
        #$\lambda = {Lambda}$\n\
        #$\\alpha_R = {Alpha-R}$\n\
        #$\Delta t = {Timestep} \, (\mu s)$\n".format(**metaData)
        #ax.text(1.02,1,statboxText,
        #        transform=ax.transAxes,fontsize=14,
        #        va = "top",
        #        ha = "left",
        #        bbox=statboxProps)

        # Set figure properties
        defaultFigSize = (fig.get_size_inches())
        newFigSize = (defaultFigSize[0]+3,defaultFigSize[1])
        fig.set_size_inches(newFigSize)
        plt.savefig(outputDirectory+outputFileName,bbox_inches="tight")

#################################################################################
#                      Miscellaneous Helper Functions
#################################################################################

def makeAvgArray(lst):
    """
    Obtains the average of the values of a bunch of arrays, passed as
    a list "lst". Used to plot the result of multiple runs as one graph.  
    For example, if given [[1,2,2],[9,-10,4]] it would return--> [5,-4,3]
    It tries to ensure that if there was a bad run (perhaps
    because it was terminated early) that the run is not used to calculate
    the average values. 
    """
    arraySize = len(lst[0][0])
    allData = np.array(lst[0][0])
    for i in range(1,len(lst)):
        if arraySize != len(lst[i][0]):
            print("Problem with array sizes")
        else:
            allData = np.dstack((allData,lst[i][0]))
    
    avgData = allData.mean(axis=2,dtype=np.float64)
    return avgData


    

################################################################################
#                    File finding and processing functions
################################################################################

def getAllLogs(parentDir):
    '''
    Walk through subdirectories, getting all files with 
    beginning with "logSarsa".
    '''
    lst = []
    logRegex = re.compile("logSarsa.*\.txt")
    #print("Looking for log files...")
    for (dirpath, dirnames, filenames) in os.walk(parentDir):
        for name in filenames:
            if logRegex.match(name):
                #print(os.path.join(dirpath,name))
                lst.append(os.path.join(dirpath,name))
    return lst


def parseFile(path):
    """
    Parse a file, getting the data as well as the metadata
    Could be better done, vis-a-vis eliminating the first regex, but 
    this is reasonably clean and safe and allows us to grab the actual
    data on the same pass
    """
    metaData = {}
    runData = []
    metaExp = re.compile("([\w\-]*)=(\S*)")
    badSplitExp = re.compile("\.\s+")
    f = open(path,"r")
    print(path)
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
            try:
                runData.append([int(tmp[0]),float(tmp[1]),int(tmp[2])])
            except ValueError as e:
                print(e)
            except IndexError as e:
                print(e)
    
    return np.array(runData), metaData


def separateByAlpha(flst):
    """
    Given a bunch of log file names (stored as a list), this returns a 
    dictionary of parsed log data, indexed by the value of alpha associated
    with each run, so runs with the same alpha are grouped together (but
    the data from each run remains distinct)
    """
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

def groupByAlpha(flst=None):
    """
    Given a bunch of log file names (stored as a list), this returns a
    dictionary of the form {alphaValue:[numpyArray,metaDataDct]}. 
    The actual run data from each run  is concatenated to form a new numpyArray.
    This may be problematic in the cases where other  metadata varies between 
    runs with the same alpha.
    """
    if flst == None:
        print("[INFO] flst is None, setting flst to getAllLogs('.')")
        flst = getAllLogs(".")
    
    data = {}
    alphaValLst = []
    for f in flst:
        rd, md = parseFile(f)
        alpha = float(md["Alpha"])
        if alpha not in alphaValLst:
            alphaValLst.append(alpha)
            data[alpha] = [rd,md]
        else:
            if rd.shape[:2] != data[alpha][0].shape[:2]:
                print("[DEBUG] groupByAlpha(): Problem with array shapes")
            data[alpha][0] = np.dstack((data[alpha][0],rd))
    return data

