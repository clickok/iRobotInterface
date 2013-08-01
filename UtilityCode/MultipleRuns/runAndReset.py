from datetime import datetime
import os
import re
import shlex
import subprocess
import sys



def runAndReset(expParameters):
    print("Ensure that the calibration is correct in all source directories!")

    origWD = os.getcwd()

    resetDir      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/"
    resetFile     = "resetToDefault.out"
    
    expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    expFile = "AvgRewardSarsaReplacing.out"


    try:
        expArgString =  "./"+ expFile + " "  + " ".join( ["--"+str(key)+" "+str(val) for key, val in expParameters.items()])
        print(expArgString)
        os.chdir(os.path.expanduser(expDir))
        p = subprocess.Popen(shlex.split(expArgString))
        p.wait()
        
        resetParameters = {"p":"/dev/ttyUSB0"}
        resetArgString ="./" + resetFile + " " + " ".join(["--"+str(key)+" "+str(val) for key, val in resetParameters.items()])

        os.chdir(os.path.expanduser(resetDir))
        p = subprocess.Popen(shlex.split(resetArgString))
        p.wait()
    except:
        os.chdir(origWD)
    os.chdir(origWD)


def performRun(portname="/dev/ttyUSB0",
               Alpha=0.5,Epsilon=0.01,Lambda=0.9,Iterations=1200,Timestep=100000,
               robotname="<Unknown>",microname="<Unknown>",batteryname="<Unknown>"):
    params = {"port":portname,
              "alpha":Alpha,
              "epsilon":Epsilon,
              "lambda":Lambda,
              "iterations":Iterations,
              "timestep":Timestep,
              "robotname":robotname,
              "microworldname":microname,
              "batteryname":batteryname}
    origWD = os.getcwd()
    startTime = datetime.now()
    expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    expFile = "AvgRewardSarsaReplacing.out"
    print("Run start time:",str(startTime))
    try:
        os.chdir(os.path.expanduser(expDir))
        expArgString =  "./"+ expFile + " "  + " ".join( ["--"+str(key)+" "+str(val) for key, val in params.items()])
        print("Performing run with command:",expArgString)
        p = subprocess.Popen(shlex.split(expArgString))
        p.wait()
        endTime = datetime.now()
        print("Run end time:",str(endTime))
        print("Total time:",str(endTime-startTime))
    finally:
        os.chdir(origWD)
        


def performReset(portname):
    print("Performing reset to default position")
    print("Ensure that the calibration is correct in all source directories!")

    origWD = os.getcwd()

    resetDir      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/"
    resetFile     = "resetToDefault.out"

    resetParameters = {"p":portname}
    resetArgString ="./" + resetFile + " " + " ".join(["--"+str(key)+" "+str(val) for key, val in resetParameters.items()])
    try:
        os.chdir(os.path.expanduser(resetDir))
        p = subprocess.Popen(shlex.split(resetArgString))
        p.wait()
    except:
        os.chdir(origWD)
    finally:
        os.chdir(origWD)

def performTimestepTrials(trials=1,portname="/dev/ttyUSB0",
                          Alpha=0.5,Epsilon=0.01,Lambda=0.9,Iterations=1200,Timestep=100000,
                          robotname="<Unknown>",microname="<Unknown>",batteryname="<Unknown>"):
    params = {"port":portname,
              "alpha":Alpha,
              "epsilon":Epsilon,
              "lambda":Lambda,
              "iterations":Iterations,
              "timestep":Timestep,
              "robotname":robotname,
              "microworldname":microname,
              "batteryname":batteryname}
    startTime = datetime.now()
    print("Program start time:",str(startTime))
    performReset(params["port"])
    for key in params.keys():
        print(params[key])
    for i in range(trials):
        print("Trial number: %d"%(i))
        runAndReset(params)
    endTime = datetime.now()
    print("Program end time:",str(endTime))
    print("Total execution time:",str(endTime-startTime))


def differentValueTimestep(trials=1,portname="/dev/ttyUSB0",
                          Alpha=[0.5],Epsilon=0.01,Lambda=0.9,Iterations=1200,Timestep=[100000],
                          robotname="<Unknown>",microname="<Unknown>",batteryname="<Unknown>"):
    '''
    A modification of the code so that we can perform a series of trials
    where alpha varies from trial to trial. 
    '''
    startTime = datetime.now()
    print("Program start time:",str(startTime))
    performReset(portname)
    for i in range(trials):
        for a in list(Alpha):
            for t in list(Timestep):
                performRun(portname,a,\
                           Epsilon,Lambda,Iterations,t,robotname,microname,batteryname)
                performReset(portname)

            
            

def updatePrograms():
    origWD = os.getcwd()
    resetDir      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/"
    expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    try:
        os.chdir(os.path.expanduser(resetDir))
        subprocess.call("make all",shell=True)

        os.chdir(os.path.expanduser(expDir))
        subprocess.call("make all",shell=True)
    except Exception as e:
        print(e)
        print("An exception has occured!")
    finally:
        os.chdir(origWD)

def removeUnfinishedLogs(minEntries, expDir=None,printOnly=False):
    if expDir == None:
        expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    origWD = os.getcwd()
    badSplitExp = re.compile("\.\s+")
    try:
        os.chdir(os.path.expanduser(expDir))
        flst = getLogsInDir(".")
        for fpath in flst:
            count = 0
            f = open(fpath,"r")
            for line in f:
                if re.match(".*#",line):
                    pass
                else:
                    # A non-data line!
                    tmp = badSplitExp.sub("",line).split()
                    if tmp[0].isdigit():
                        count +=1
            print(fpath,count)
            if count < minEntries:
                if printOnly:
                    os.remove(fpath)
                print("DELETING")
    finally:
        os.chdir(origWD)

                
def getLogsInDir(dir):
    logRegex = re.compile("logSarsa.*\.txt")
    flst = [f for f in os.listdir(dir) if os.path.isfile(os.join(dir,f))]
    return flst
    

def getAllLogs(parentDir):
    '''
    Walk through subdirectories, getting all files with 
    beginning with "logSarsa".
    '''
    lst = []
    logRegex = re.compile("logSarsa.*\.txt")
    for (dirpath, dirnames, filenames) in os.walk(parentDir):
        for name in filenames:
            if logRegex.match(name):
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

if __name__ == "__main__":
    if len(sys.argv) < 2:
        pass
    else:
        performTrials(trials = int(sys.argv[1]))
