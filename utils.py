import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

from agent import *
from safeAgent import *
from safeSim import *




def plotStuff(evader, pursuer):
    plt.cla()
    
    plt.scatter(evader._state[0], evader._state[1])
    circle = plt.Circle((evader._state[0], evader._state[1]), radius=evader._safe_radius, fill=False, edgecolor='blue',linestyle='dotted',linewidth='2.2')
    plt.gca().add_patch(circle)

    if evader._agent_type == 'aacas':
        plt.scatter(evader._state[0], evader._state[1])
        circle = plt.Circle((evader._state[0], evader._state[1]), radius=evader.safe_dist, fill=False, edgecolor='red',linestyle='dotted',linewidth='2.2')
        plt.gca().add_patch(circle)

        plt.plot([evader._state[0], evader._state[0]+evader._safe_radius*np.cos(evader._state[2])], 
                 [evader._state[1], evader._state[1]+evader._safe_radius*np.sin(evader._state[2])])

    if pursuer._agent_type == 'aacas':
        plt.scatter(pursuer._state[0], pursuer._state[1])
        circle = plt.Circle((pursuer._state[0], pursuer._state[1]), radius=pursuer._safe_radius, fill=False, edgecolor='blue',linestyle='dotted',linewidth='2.2')
        plt.gca().add_patch(circle)

        plt.scatter(pursuer._state[0], pursuer._state[1])
        circle = plt.Circle((pursuer._state[0], pursuer._state[1]), radius=pursuer.safe_dist, fill=False, edgecolor='red',linestyle='dotted',linewidth='2.2')
        plt.gca().add_patch(circle)

    plt.scatter(evader._goal[0], evader._goal[1])
    # plt.scatter(pursuer._goal[0], pursuer._goal[1])

    plt.scatter(pursuer._state[0], pursuer._state[1])

    # plt.xlim([evader._state[0]-evader.safe_dist-5,evader._state[0]+evader.safe_dist+5])
    # plt.ylim([evader._state[1]-evader.safe_dist-5,evader._state[1]+evader.safe_dist+5])

    dist = - 5 + evader._goal[0]+5
    plt.xlim([-5,evader._goal[0]+5])
    plt.ylim([-dist/2,dist/2])
    plt.draw()
    plt.pause(0.0000001)



def plotStateSpace(state_space, p_x_r, p_y_r, disc, num_ticks=5, goal=[20,0]):
    fig, ax = plt.subplots(1,1)
    plt.imshow(1-state_space, cmap='Greys')

    # Set X Axes
    places = np.linspace(0,disc, num_ticks+2)
    x_ticks = np.round(np.linspace(p_x_r[0], p_x_r[1], num_ticks+2),1)
    ax.set_xticks(places)
    ax.set_xticklabels(x_ticks)

    # Set Y Axes
    places = np.linspace(0,disc, num_ticks+2)
    y_ticks = np.round(np.linspace(p_y_r[0], p_y_r[1], num_ticks+2),1)
    ax.set_yticks(places)
    ax.set_yticklabels(y_ticks)

    # Set Goal Point
    gx = disc/(p_x_r[1] - p_x_r[0]) * (goal[0]- p_x_r[0])
    gy = disc/(p_y_r[1] - p_y_r[0]) * (goal[1]- p_y_r[0])
    plt.scatter(gx, gy)

    # Set Start Point
    gx = disc/(p_x_r[1] - p_x_r[0]) * (-p_x_r[0])
    gy = disc/(p_y_r[1] - p_y_r[0]) * (-p_y_r[0])
    plt.scatter(gx, gy)

    rad = 2
    rx = disc/(p_x_r[1] - p_x_r[0]) * (rad)*2
    ry = disc/(p_y_r[1] - p_y_r[0]) * (rad)*2

    circle = patches.Ellipse(([gx], [gy]), rx, ry, fill=False, edgecolor='red',linestyle='dotted',linewidth='2.2')
    plt.gca().add_patch(circle)

    # ax.set_xticks(places)
    plt.show()
    


def plotVelocitySpace(state_space, ev, pv, disc, num_ticks=5, goal=[20,0]):
    fig, ax = plt.subplots(1,1)
    # plt.imshow(1-state_space)
    ax.imshow(1-state_space, cmap='Greys')

    # Set X Axes
    places = np.linspace(0,disc, num_ticks+2)
    x_ticks = np.round(np.linspace(ev[0], ev[1], num_ticks+2),1)
    ax.set_xticks(places)
    ax.set_xticklabels(x_ticks)

    # Set Y Axes
    places = np.linspace(0,disc, num_ticks+2)
    y_ticks = np.round(np.linspace(pv[0], pv[1], num_ticks+2),1)
    ax.set_yticks(places)
    ax.set_yticklabels(y_ticks)

    plt.xlabel('Evader Velocity')
    plt.ylabel('Pursuer Velocity')

    # ax.set_xticks(places)
    plt.show()


def printResults(state_space):
    _, counts = np.unique(state_space, return_counts=True)
    colls = counts[0]
    avoids= state_space.size - colls
    print('Number of Collisions: %i' %(colls))
    print('Number of Avoids: %i' %(avoids))
    print('Avoid Rate: %.2f%%' %(100*avoids/(avoids+colls)))