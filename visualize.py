from mpl_toolkits.mplot3d import axes3d, Axes3D
import matplotlib.pyplot as plt
import numpy as np


def plotTransformation(transformation):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(0, 0.2)
    ax.set_ylim3d(0, 0.2)
    ax.set_zlim3d(0, 0.2)

    x = np.array([0, 0, 0, transformation[0, 3], transformation[0, 3], transformation[0, 3]])
    y = np.array([0, 0, 0, transformation[1, 3], transformation[1, 3], transformation[1, 3]])
    z = np.array([0, 0, 0, transformation[2, 3], transformation[2, 3], transformation[2, 3]])


    u = np.concatenate([np.array([1, 0, 0]), transformation[:3, 0]])
    v = np.concatenate([np.array([0, 1, 0]), transformation[:3, 1]])
    w = np.concatenate([np.array([0, 0, 1]), transformation[:3, 2]])

    red = np.array([1, 0, 0])
    green = np.array([0, 1, 0])
    blue = np.array([0, 0, 1])

    colors = np.array([red, green, blue, red, green, blue])

    q = ax.quiver(x, y, z, u, v, w, length=0.1, colors=colors, lw=1)

    plt.plot([x[0], x[3]], [y[0], y[3]], [z[0], z[3]], '--', color='black')

    plt.show()