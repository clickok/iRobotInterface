import os
import shlex
import subprocess
import sys


def runAndReset():
    print("Ensure that the calibration is correct in all source directories!")

    origWD = os.getcwd()

    resetDir      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/"
    resetFile     = "resetToDefault.out"
    
    expDir = "~/git/iRobotInterface/ControlExperiments/AverageRewardSarsa/TimestepExperiments/"
    expFile = "AvgRewardSarsaReplacing.out"

    expParameters = {"p":"/dev/ttyUSB0","i":"200"}
    expArgString =  "./"+ expFile + " "  + " ".join( ["-"+str(key)+" "+str(val) for key, val in expParameters.items()])
    
    os.chdir(os.path.expanduser(expDir))
    p = subprocess.Popen(shlex.split(expArgString))
    p.wait()

    resetParameters = {"p":"/dev/ttyUSB0"}
    resetArgString ="./" + resetFile + " " + " ".join(["-"+str(key)+" "+str(val) for key, val in resetParameters.items()])

    os.chdir(os.path.expanduser(resetDir))
    p = subprocess.Popen(shlex.split(resetArgString))
    p.wait()

    os.chdir(origWD)
