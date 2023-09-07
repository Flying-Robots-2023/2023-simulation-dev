import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
import numpy as np

def limit_physical_boundary(u_1, p, physical_bounds):   
    out_of_bounds = np.array([False, False, False])
    min_x = physical_bounds[0]
    max_x = physical_bounds[1]
    min_y = physical_bounds[2]
    max_y = physical_bounds[3]
    min_z = physical_bounds[4]
    max_z = physical_bounds[5]

    if((p[0] < min_x and u_1[0] < 0) or (p[0] > max_x and u_1[0] > 0)):
        u_1[0] = 0
        out_of_bounds[0] = True
    if((p[1] < min_y and u_1[1] < 0) or (p[1] > max_y and u_1[1] > 0)):
        u_1[1] = 0
        out_of_bounds[1] = True
    if((p[2] < min_z and u_1[2] < 0) or (p[2] > max_z and u_1[2] > 0)):
        u_1[2] = 0
        out_of_bounds[2] = True
    return u_1, out_of_bounds

def get_bounds(target, p_D, p_A, sensing_range):
    scalex = np.max([np.max(np.abs(p_D[0]-target[0])),np.max(np.abs(p_A[0]-target[0])),sensing_range])
    scaley = np.max([np.max(np.abs(p_D[1]-target[1])),np.max(np.abs(p_A[1]-target[1])),sensing_range])
    scalexy = max([scalex,scaley])
    scalez = np.max([np.max(np.abs(p_D[2]-target[2])),np.max(np.abs(p_A[2]-target[2]))])

    bounds = np.array([-scalexy, scalexy, -scalexy, scalexy, 0, scalez])  + np.array([target[0], target[0], target[1], target[1], target[2], target[2]])
    return bounds
def Extract(lst, pos):
    return list(list(zip(*lst))[pos])

# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])