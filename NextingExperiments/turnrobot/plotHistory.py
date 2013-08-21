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
    return pred/(np.max(pred)-np.min(pred))

def plotAndShow(*plst):
    ax, fig = plt.subplots(1)
    for p in plst:
        fig.plot(p)
    plt.show()


def normalHistory(data,h,alpha):
    w,pred = basicHistory.basicHistory(data,h,alpha)
    normPred = normalize(pred)
    return normPred

    
data  = np.loadtxt("turnrobot.dat")
ideal = makeIdealPred(data[:,3:],0.85)
w,pred  = basicHistory.basicHistory(data[:,3:],100,0.5)
normIdeal= normalize(ideal)
normPred = normalize(pred)



