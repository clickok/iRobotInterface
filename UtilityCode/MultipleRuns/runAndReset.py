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

    expArgString =  "./"+ expFile + " "  + " ".join( ["--"+str(key)+" "+str(val) for key, val in expParameters.items()])
    
    os.chdir(os.path.expanduser(expDir))
    p = subprocess.Popen(shlex.split(expArgString))
    p.wait()

    resetParameters = {"p":"/dev/ttyUSB0"}
    resetArgString ="./" + resetFile + " " + " ".join(["--"+str(key)+" "+str(val) for key, val in resetParameters.items()])

    os.chdir(os.path.expanduser(resetDir))
    p = subprocess.Popen(shlex.split(resetArgString))
    p.wait()

    os.chdir(origWD)

def performTrials(trials=1,A=0.9,E=0.01,L=0.9,I=1200):
    params = {"alpha":A,"epsilon":E,"lambda":L,"iterations":I}
    for i in range(trials):
        runAndReset(params)


