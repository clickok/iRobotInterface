import numpy as np
import matplotlib.pyplot as plt

import basicHistory


def makeIdealPred(data,gamma):
    T     = len(data) 
    ideal = np.zeros(T)
    for i in range(T-2,-1,-1):
        ideal[i] = data[i+1] + gamma*ideal[i+1]
    return ideal

def getSquaredDiff(pred1,pred2):
    diff = np.sqrt(np.power((pred1-pred2),2))
    return diff

def normalize(pred):
    high = np.max(pred)
    low  = np.min(pred)
    ret  = pred + low
    ret  = ret/(high-low)
    return ret

# Loading data from a run's file, and making an ideal prediction based on the 
# rightmost bumper with a gamma value of 0.85

def sweep(data,historyRange,alphaRange):
    filename = "paramSweepH%d-%dA%d-%d.txt"%(historyRange[0],historyRange[-1],\
                                             alphaRange[0],alphaRange[-1])
    ideal = makeIdealPred(data,0.85)
    normIdeal = normalize(ideal)
    length = len(normIdeal)
    with open(filename,"w") as f:
        for k in historyRange:
            for a in alphaRange:
                alpha = a/alphaRange[-1]
                weights, pred = basicHistory.basicHistory(data,k,alpha)
                normPred = normalize(pred)
                totalError = np.sum(np.sqrt(np.power((normIdeal-normPred),2)))
                avgError   = totalError/length
                f.write(" ".join([str(x) for x in [k,alpha,totalError,avgError]]))
                f.write("\n")


def main():
    data  = np.loadtxt("turnrobot.dat")[:,3:]
    #sweep(data,range(10,301,10),range(1,11))

if __name__ == "__main__":
    main()
