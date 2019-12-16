import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import quaternion as quat
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, vec, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = [[0.0, vec[0]], [0.0, vec[1], [0.0, vec[2]]]]

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

def drawCoord(ax, quat):
    rot = quat.as_rotation_matrix(quat)
    arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->',shrinkA=0, shrinkB=0)
    a = Arrow3D(rot[:,0], **arrow_prop_dict, color='r')
    ax.add_artist(a)
    a = Arrow3D(rot[:,1], **arrow_prop_dict, color='g')
    ax.add_artist(a)
    a = Arrow3D(rot[:,2], **arrow_prop_dict, color='b')
    ax.add_artist(a)

def animateOrientation(quaternions, ax, fig):
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, drawCoord, ax, fargs=(quaternions), interval=200, blit=True)

def draw(reference, X, U, horizon):
    numPlotPoints = len(U)
    time = np.linspace(0, horizon, numPlotPoints)

    position = np.zeros([numPlotPoints, 3])
    velocity = np.zeros([numPlotPoints, 3])
    omega = np.zeros([numPlotPoints, 3])
    thrust = np.zeros([numPlotPoints, 1])
    ratesMagn = np.zeros([numPlotPoints,1])
    orientation = np.zeros([numPlotPoints, 4])

    position_c = np.zeros([numPlotPoints, 3])
    velocity_c = np.zeros([numPlotPoints, 3])
    omega_c = np.zeros([numPlotPoints, 3])
    thrust_c = np.zeros([numPlotPoints, 1])
    ratesMagn_c = np.zeros([numPlotPoints,1])
    orientation_c = np.zeros([numPlotPoints, 4])

    for i in range(numPlotPoints):
        t = time[i]
        position[i, :] = reference[i][:3]
        orientation[i, :] = reference[i][3:7]
        velocity[i, :] = reference[i][7:10]
        omega[i, :] = reference[i][11:14]
        thrust[i] = reference[i][10]
        ratesMagn[i] = np.linalg.norm(reference[i][11:14])
        position_c[i, :] = X[i][:3]
        orientation_c[i, :] = X[i][3:7]
        velocity_c[i, :] = X[i][7:10]
        omega_c[i, :] = U[i][1:]
        thrust_c[i] = U[i][0]
        ratesMagn_c[i] = np.linalg.norm(U[i][1:])


    figStates, axes = plt.subplots(3,1,sharex=True,figsize=(26,16))
    gs = gridspec.GridSpec(6, 3)
    axPos = plt.subplot(gs[0:2, 0])
    axVel = plt.subplot(gs[2:4, 0])
    axAcc = plt.subplot(gs[4:6, 0])

    for ax,yvals in zip([axPos, axVel, axAcc], [position,velocity,omega]):
        cols = ['r','g','b']
        labs = ['x','y','z']
        for i in range(3):
            ax.plot(time,yvals[:,i],cols[i],label=labs[i])

    for ax,yvals in zip([axPos, axVel, axAcc], [position_c,velocity_c,omega_c]):
        cols = ['c','m','y']
        labs = ['x_c','y_c','z_c']
        for i in range(3):
            ax.plot(time,yvals[:,i],cols[i],label=labs[i])

    axPos.set_ylabel('Pos [m]')
    axVel.set_ylabel('Vel [m/s]')
    axAcc.set_ylabel('Omega [m/s^2]')
    axAcc.set_xlabel('Time [s]')
    axPos.legend()
    axPos.set_title('States')

    infeasibleAreaColour = [1,0.5,0.5]
    axThrust = plt.subplot(gs[0:3, 1])
    axOmega  = plt.subplot(gs[3:6, 1])
    axThrust.plot(time,thrust,'k', label='command')
    axThrust.plot(time,thrust_c,'r', label='command_mpc')

    fmin = 0
    fmax = 2.0 * 9.81
    wmax = 6.28
    axThrust.plot([0,horizon],[fmin,fmin],'r--', label='fmin')
    axThrust.fill_between([0,horizon],[fmin,fmin],-1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.fill_between([0,horizon],[fmax,fmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.plot([0,horizon],[fmax,fmax],'r-.', label='fmax')

    axThrust.set_ylabel('Thrust [m/s^2]')
    axThrust.legend()

    axOmega.plot(time, ratesMagn,'k',label='command magnitude')
    axOmega.plot(time, ratesMagn_c,'r',label='command magnitude mpc')
    axOmega.plot([0,horizon],[wmax,wmax],'r--', label='wmax')
    axOmega.fill_between([0,horizon],[wmax,wmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axOmega.set_xlabel('Time [s]')
    axOmega.set_ylabel('Body rates [rad/s]')
    axOmega.legend()

    axThrust.set_title('Inputs')

    #make the limits pretty:
    axThrust.set_ylim([min(fmin-1,min(thrust)), max(fmax+1,max(thrust))])
    axOmega.set_ylim([0, max(wmax+1,max(ratesMagn))])

    # Animate the orientation.

    axRefq = plt.subplot(gs[0:3, 2],projection='3d')
    axStateq  = plt.subplot(gs[3:6, 2],projection='3d')

    qRef = quaternion.as_quat_array(orientation)
    qState = quaternion.as_quat_array(orientation_c)
    animateOrientation(qRef,figStates,axRefq)
    animateOrientation(qState,figStates,axStateq)

    plt.show()

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import quaternion
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, vec, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = [[0.0, vec[0]], [0.0, vec[1]], [0.0, vec[2]]]
        self.renderer = None
    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.renderer = renderer
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

    def reset(self,vec):
        self._verts3d = [[0.0, vec[0]], [0.0, vec[1]], [0.0, vec[2]]]
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))

from matplotlib import animation

def animateOrientation(quaternions, fig, ax):
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    def animate(i):
        rot = quaternion.as_rotation_matrix(quaternions[i])
        a.reset(rot[:,0])
        b.reset(rot[:,1])
        c.reset(rot[:,2])
        cap2.set_position([rot[:,0][0],rot[:,0][1],rot[:,0][2]+0.1])
        cap3.set_position([rot[:,1][0],rot[:,1][1],rot[:,1][2]+0.1])
        cap4.set_position([rot[:,2][0],rot[:,2][1],rot[:,2][2]+0.1])
        title0.set_text(u"Current Step: {}".format(i))
        return a,b,c,cap1,cap2,cap3,cap4,title0
    def drawCoord():
        rot = quaternion.as_rotation_matrix(quaternion.one)
        arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->',shrinkA=0, shrinkB=0)
        a = Arrow3D(rot[:,0], **arrow_prop_dict, color='r')
        ax.add_artist(a)
        b = Arrow3D(rot[:,1], **arrow_prop_dict, color='g')
        ax.add_artist(b)
        c = Arrow3D(rot[:,2], **arrow_prop_dict, color='b')
        ax.add_artist(c)
        cap1 = ax.text(0.0, 0.0, -0.15, r'$o$')
        cap2 = ax.text(rot[:,0][0],rot[:,0][1],rot[:,0][2]+0.1,r'$x$')
        cap3 = ax.text(rot[:,1][0],rot[:,1][1],rot[:,1][2]+0.1,r'$y$')
        cap4 = ax.text(rot[:,2][0],rot[:,2][1],rot[:,2][2]+0.1,r'$z$')
        title0 = ax.set_title('Current Step: 0.0')
        return a,b,c,cap1,cap2,cap3,cap4, title0

    a,b,c,cap1,cap2,cap3,cap4,title0 = drawCoord()
    anim = animation.FuncAnimation(fig, animate, frames =len(quaternions), init_func=drawCoord, interval=200, blit=True)

def test():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    quaternions = quaternion.as_quat_array(np.random.random((300, 4)))
    animateOrientation(quaternions,fig,ax)
    plt.show()

def test_q(orientation):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    q = quaternion.quaternion(orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val)
    rot = quaternion.as_rotation_matrix(q)
    arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->',shrinkA=0, shrinkB=0)
    a = Arrow3D(rot[:,0], **arrow_prop_dict, color='r')
    ax.add_artist(a)
    b = Arrow3D(rot[:,1], **arrow_prop_dict, color='g')
    ax.add_artist(b)
    c = Arrow3D(rot[:,2], **arrow_prop_dict, color='b')
    ax.add_artist(c)
    cap1 = ax.text(0.0, 0.0, -0.15, r'$o$')
    cap2 = ax.text(rot[:,0][0],rot[:,0][1],rot[:,0][2]+0.1,r'$x$')
    cap3 = ax.text(rot[:,1][0],rot[:,1][1],rot[:,1][2]+0.1,r'$y$')
    cap4 = ax.text(rot[:,2][0],rot[:,2][1],rot[:,2][2]+0.1,r'$z$')
    plt.show()
