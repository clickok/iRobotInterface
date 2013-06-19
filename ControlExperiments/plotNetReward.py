#!/Library/Frameworks/EPD64.framework/Versions/Current/bin/python
import pylab as P
import sys
import bisect
inputlogfile=sys.argv[1]

print "Proccessing logfile ",inputlogfile
lines=open(inputlogfile,"r").readlines()

metadata=[x for x in lines if x[0]=="#"] #select meta data
fields=[x.split()[2:] for x in  metadata if x.split()[1]=="Fields:"][0]
print fields
R_index=fields.index("R")
Clock_index=fields.index("CLOCKTIME")

q=[x for x in lines if x[0]!="#"] #remove meta data
q=q[:-1] #last line might be malformed
print "first line: ",q[0]
R=P.array([float(x.split()[R_index]) for x in q])
Realtime=P.array([float(x.split()[Clock_index]) for x in q])
N=len(R)
ns=range(1,N+1)
avgR=P.cumsum(R)/ns
n50=range(50,len(R))
l50=[P.mean(R[i-50:i]) for i in n50]
P.plot(ns,avgR,"r")
P.plot(n50,l50,"b")
P.text(N/2, avgR[N/4]-1, "Cumulative Average reward",color="r")
P.text(N/2, P.amax(l50) +1, "Average of last 50 rewards",color="b")
ytime=P.amin(avgR[:20]) +1
for i in P.arange(60,P.amax(Realtime),60):
    ind=bisect.bisect(Realtime,i)
    P.text(ind,ytime,".\n%d minute"%(i/60))
P.xlabel("Timestep")
P.savefig(inputlogfile+".pdf")
