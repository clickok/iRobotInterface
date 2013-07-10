import os
import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import re
import math

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
        if key < (2**-6):
            continue
        plotForAlpha(logData,key)

def plotAllCumulativeRewards():
    '''
    Plots the average cumulative reward (for each value of alpha) 
    vs. the timestep. It should provide a quick way to determine
    how well each algorithm is doing overall at each point in time
    and relative to the alternative values of alpha.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Cumulative Reward vs. Timestep for Varying Alpha (Averaged Over Multiple Runs)")
    ax.set_ylabel("Cumulative Reward")
    ax.set_xlabel("Timestep")
    for key in logData.keys():
        if key < (2**-6):
            continue
        avgData = makeAvgArray(logData[key])
        xdata = avgData[:,0]
        ydata = np.cumsum(avgData[:,2])
        ax.plot(xdata,ydata,label="$\\alpha = %f$"%key)
    leg = plt.legend(loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+4,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    plt.savefig("logPlotAllAlphasCumulative",bbox_inches="tight")
    
def plotEachCumulativeReward():
    '''
    Plots the cumulative reward vs. timestep for every run, but colors them
    differently. It should provide a quick way to estimate variance, among
    other things.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Cumulative Reward vs. Timestep for Varying Alpha")
    ax.set_ylabel("Cumulative Reward")
    ax.set_xlabel("Timestep")
    colors = iter(cm.rainbow(np.linspace(0,1,len(logData))))
    for key in logData.keys():
        if key < 2**(-6):
            continue
        alphaData = logData[key]
        c=next(colors)
        for run in alphaData:
            runData = run[0]
            xdata = runData[:,0]
            ydata = np.cumsum(runData[0:,2])
            ax.scatter(xdata,ydata,color=c,label="$\\alpha = %f$"%key)
    handles, labels = ax.get_legend_handles_labels()
    newHandles = []
    newLabels = []
    for i in zip(handles,labels):
        if i[1] not in newLabels:
            newHandles.append(i[0])
            newLabels.append(i[1])
    leg = plt.legend(newHandles,newLabels,loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+5,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    ax.set_xlim([0,50])
    ax.set_ylim([0,1000])
    plt.savefig("logPlotEachAlphasCumulative",
                dpi=300,
                bbox_inches="tight",
                pad_inches=0.5)
    
def plotAllAtOnce():
    '''
    Plots the average reward (averaged for each value of alpha)
    vs. the timestep. It should provide a quick way to look at how
    each value is doing, and how quickly they converge/what they
    converge to.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Average Reward vs. Timestep for Varying Alpha")
    ax.set_ylabel("Average Reward")
    ax.set_xlabel("Timestep")
    for key in logData.keys():
        if key < 2**-6:
            continue
        avgData = makeAvgArray(logData[key])
        xdata = avgData[:,0]
        ydata = np.cumsum(avgData[:,2])/avgData[:,0]
        ax.plot(xdata,ydata,label="$\\alpha = %f$"%key)
    #ax.set_ylim([0,20])
    leg = plt.legend(loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+2,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    plt.savefig("logPlotAllAlphas",bbox_inches="tight",dpi=600)

def plotAllByAlpha():
    '''
    Plots the cumulative reward (i.e., reward over an entire run)
    vs. alpha, as well as the transient reward (i.e., cumulative
    reward over the first half of the run) and the asymptopic 
    reward (cumulative reward over second half of the run).
    
    * Can be modified (within the code) to use logarithmic scale
    * instead of linear scale.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Total Cumulative Reward vs. Alpha")
    ax.set_ylabel("Total Reward")
    ax.set_xlabel("$\\alpha$")
    xdata = []
    totalReward = []
    transientReward = []
    asymptoteReward = []
    for key in logData.keys():
        #print(len(logData[key]))
        avgData = makeAvgArray(logData[key])
        if key < (2**-6):
            continue
        xdata.append(key)
        # Total cumulative reward
        separationThreshold = len(avgData[:,2])//2
        totalReward.append(np.sum(avgData[:,2]))
        transientReward.append(np.sum(avgData[:separationThreshold,2]))
        asymptoteReward.append(np.sum(avgData[separationThreshold:,2]))
    xdata, totalReward = (list(x) for x in zip(* sorted(zip(xdata,totalReward))))
    ax.plot(xdata,totalReward,label="Total Reward",color="blue")
    #ax.scatter(xdata,transientReward,label="Transient Reward",color="red")
    #ax.scatter(xdata,asymptoteReward,label="Asymptotic Reward",color="green")
    ax.set_xlim([min(xdata)/2,max(xdata)*1.1])
    ax.set_ylim([min(totalReward)*0.98,max(totalReward)*1.02])
    ax.set_xscale("log",basex=2)
    leg = plt.legend(loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+2,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    plt.savefig("logPlotTotalAvgRewByAlphaLogarithmicCloseLine",bbox_inches="tight")


def plotEachByAlpha():
    '''
    Plots the cumulative reward (i.e., reward over an entire run)
    vs. alpha, as well as the transient reward (i.e., cumulative
    reward over the first half of the run) and the asymptopic 
    reward (cumulative reward over second half of the run).
    DOES THIS FOR EACH RUN (i.e., NOT averaged!)
    
    * Can be modified (within the code) to use logarithmic scale
    * instead of linear scale.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Cumulative Rewards vs. Alpha")
    ax.set_ylabel("Cumulative Reward")
    ax.set_xlabel("$\\alpha$")
    colors = iter(cm.rainbow(np.linspace(0,1,len(logData))))
    for key in logData.keys():
        if key < (2**-6):
            continue
        alphaData = logData[key]
        c = next(colors)
        for run in alphaData:
            runData = run[0]
            xdata = (key)
            ydata = (np.sum(runData[0:,2]))
            ax.scatter(xdata,ydata,color=c,label="$\\alpha = %f$"%key)
    #ax.set_xlim([min(logData.keys())/2,max(xdata)*1.1])
    #ax.set_ylim([min(transientReward)*0.9,max(totalReward)*1.1])
    ax.set_xscale("log",basex=2)
    handles, labels = ax.get_legend_handles_labels()
    newHandles = []
    newLabels = []
    for i in zip(handles,labels):
        if i[1] not in newLabels:
            newHandles.append(i[0])
            newLabels.append(i[1])
    leg = plt.legend(newHandles,newLabels,loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+2,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    plt.savefig("logPlotEachRewByAlphaLogarithmic",bbox_inches="tight")

def plotEachTotalOverTimestepsByAlpha():
    '''
    Plots the cumulative reward (i.e., reward over an entire run)
    vs. alpha, as well as the transient reward (i.e., cumulative
    reward over the first half of the run) and the asymptopic 
    reward (cumulative reward over second half of the run).
    DOES THIS FOR EACH RUN (i.e., NOT averaged!)
    
    * Can be modified (within the code) to use logarithmic scale
    * instead of linear scale.
    '''
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Average Reward by Timestep vs. Alpha")
    ax.set_ylabel("Average Reward")
    ax.set_xlabel("$\\alpha$")
    colors = iter(cm.rainbow(np.linspace(0,1,len(logData))))
    
    alphaLst = []
    runAvgLst = []
    runAvgLstErr = []
    for key in logData.keys():
        if key < (2**-6):
            continue
        alphaData = logData[key]
        alphaLst.append(key)
        runLst = []
        c = next(colors)
        for run in alphaData:
            runData = run[0]
            timesteps= len(runData[0:,2])
            xdata = (key)
            ydata = (np.sum(runData[0:,2]))/timesteps
            ax.scatter(xdata,ydata,label="$\\alpha = %f$"%key,color="blue")
            runLst.append(ydata)
        runAvgLst.append(sum(runLst)/len(alphaData))
        runStdDev = np.std(np.array(runLst))
        runStdErr = runStdDev/math.sqrt(len(runLst))
        runAvgLstErr.append(runStdErr)
    alphaLst, runAvgLst = (np.array(x) for x in zip(*sorted(zip(alphaLst,runAvgLst))))
    ax.errorbar(alphaLst,runAvgLst,yerr=runAvgLstErr,
                label="Mean Average Reward",color="red")
    ax.set_xlim([min(alphaLst)*0.9,max(alphaLst)*1.1])
    #ax.set_ylim([min(transientReward)*0.9,max(totalReward)*1.1])
    ax.set_xscale("log",basex=2)
    handles, labels = ax.get_legend_handles_labels()
    newHandles = []
    newLabels = []
    for i in zip(handles,labels):
        if i[1] not in newLabels:
            newHandles.append(i[0])
            newLabels.append(i[1])
    #leg = plt.legend(newHandles,newLabels,loc="upper left",bbox_to_anchor=(1,1))
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+2,defaultFigSize[1])
    fig.set_size_inches(newFigSize)
    plt.savefig("logPlotEachRewOverTimestepsByAlphaLogarithmicErrorBars",bbox_inches="tight")


def plotRewardByTimeAveraged(alpha):
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    fig, ax = plt.subplots(1)
    ax.set_title("Reward vs. Timestep (Averaged Over Multiple Runs)")
    ax.set_ylabel("Reward")
    ax.set_xlabel("Timestep")
    
    avgData = makeAvgArray(logData[alpha])
    xdata = avgData[:,0]
    ydata = avgData[:,2]
    ax.plot(xdata,ydata)
    
    #ax.set_ylim([-25,30])
    defaultFigSize = (fig.get_size_inches())
    newFigSize = (defaultFigSize[0]+5,defaultFigSize[1])
    fig.set_size_inches(newFigSize)


    outputPlotName = ("logPlotRewardsAlpha-%f"%alpha).replace(".",",")
    plt.savefig(outputPlotName,dpi=300)

def plotAllRewardsByTimeAveraged():
    parentdir = "."
    logFileLst = getAllLogs(parentdir)
    logData = separateByAlpha(logFileLst)
    for key in logData.keys():
        plotRewardByTimeAveraged(key)
    
    


# Testing
parentdir = "."
logFileLst = getAllLogs(parentdir)
logData = separateByAlpha(logFileLst)

def get_default_args(func):
    import inspect
    '''
    Returns a dictionary of arg_name:default_values for the input function
    '''
    args, varargs, keywords, defaults = inspect.getargspec(func)
    print(args)
    print(varargs)
    print(keywords)
    print(defaults)
    #return dict(zip(args(reversed(args), reversed(defaults))))

# Example, unrelated code (used while helping others in the lab)
def printAgents():
    numAgents = 10
    import random
    array = [(random.random(),random.random()) for i in range(numAgents)]
    for i in range(len(array)):
        print(i,array[i])
    
def sumValues():
    import random
    array = np.array([[i*j+i for i in range(10)] for j in range(10)])
    print(array)
    print(array[3,:])
    print(sum(array[3,:]))
    print(array[:,3])
    print(sum(array[:,3]))
    print(sum(sum(array)))

def superAppend(n):
    lst = []
    import time, random
    start = time.clock()
    for i in range(n):
        lst.append(random.random())
    end = time.clock()
    print(lst)
    print(end-start)
    start = time.clock()
    lst = [random.random() for i in range(n)]
    print(lst)
    end = time.clock()
    print(end-start)
    start = time.clock()
    lst = random.sample(range(n),n)
    end = time.clock()
    print(lst)
    print(end-start)
