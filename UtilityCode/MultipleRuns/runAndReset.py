import os
import shlex
import subprocess
import sys


def runAndReset():
    print("Ensure that the calibration is correct in all source directories!")
    path_Reset      = "~/git/iRobotInterface/UtilityCode/ResetToDefault/resetToDefault.out"
    path_Experiment = "~/git/iRobotInterface/ControlExperiments/.AverageRewardSarsa/TimestepExperiments/AvgRewardSarsaReplacing.out"

    expParameters = {"p":"/dev/ttyUSB0"}
    expArgString = path_Experiment + ["-"+str(key)+" "+str(val) for key, val in expParameters.items()]
    p = subprocess.Popen(cmdLineArgsExp)
