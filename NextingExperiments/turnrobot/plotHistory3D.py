import mpl_toolkits.mplot3d.axes3d as axes3d
import numpy as np
from matplotlib import cm
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import basicHistory

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    filename = "paramSweepH140-300A1-20"
    data = np.loadtxt(filename+".txt")

    x = data[:,0]
    y = data[:,1]
    z = data[:,3]

    xi = np.linspace(min(x), max(x))
    yi = np.linspace(min(y), max(y))


    Xs, Ys = np.meshgrid(xi, yi)
    Zs = griddata(x, y, z, xi, yi)

    surf = ax.plot_surface(Xs, Ys, Zs, rstride=4, cstride=4, alpha=0.4,cmap=cm.jet)
    #surf = ax.plot_wireframe(Xs, Ys, Zs)

    ax.set_zlim3d(np.min(Zs), np.max(Zs))
    fig.colorbar(surf)


    plt.savefig(filename+".png")

if __name__ == "__main__":
    main()
