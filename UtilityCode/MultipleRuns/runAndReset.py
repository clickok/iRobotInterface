from datetime import datetime
import os
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

def performReset(portname):
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

def performTrials(trials=1,A=0.9,E=0.01,L=0.9,I=1200):
    params = {"port":"/dev/ttyUSB0","alpha":A,"epsilon":E,"lambda":L,"iterations":I}
    startTime = datetime.now()
    print("Program start time:",str(startTime))
    performReset(params["port"])
    for key in params.keys():
        print(params[key])
    for i in range(trials):
        runAndReset(params)
    endTime = datetime.now()
    print("Program end time:",str(endTime))
    print("Total execution time:",str(endTime-startTime))


def updatePrograms():
    origWD = os.getcwd()
    resetDir      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/"
    expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    try:
        os.chdir(resetDir)
        subprocess.call("make all",shell=True)
    except Exception as e:
        print(e)
        print("An exception has occured!")
    finally:
        os.chdir(origWD)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        pass
    else:
        performTrials(trials = int(sys.argv[1]))
