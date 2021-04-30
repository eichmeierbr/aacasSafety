import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

from agent import *
from safeAgent import *

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
    plt.scatter(pursuer._goal[0], pursuer._goal[1])

    plt.scatter(pursuer._state[0], pursuer._state[1])

    plt.xlim([evader._state[0]-evader.safe_dist-5,evader._state[0]+evader.safe_dist+5])
    plt.ylim([evader._state[1]-evader.safe_dist-5,evader._state[1]+evader.safe_dist+5])

    # plt.xlim([-5,20])
    # plt.ylim([-15,15])
    plt.draw()
    plt.pause(0.0000001)

def plotStateSpace(state_space, p_x_r, p_y_r, p_th_r, disc, num_ticks=5, goal=[20,0]):
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



def checkEndConditions(evader, puruser, time=0, printer=False):
    if np.linalg.norm(evader._state[:2]-puruser._state[:2]) < evader._safe_radius:
        if printer:
            print('Safe Radius Violated')
        return -1
    elif np.linalg.norm(evader._state[:2] - evader._goal[:2]) < 1:
        if printer:
            print('Destination Reatched')
        return 1
    elif time > 15:
        if printer:
            print('Time Exceeded')
        return 1
    return 0


def run_game(evader, pursuer, plot=True, printer=False):
    dt = 0.01
    time = 0
    count = 0
    plot_rate = 10

    end_condition = 0
    while end_condition == 0:
        evader.act(dt, args=[pursuer])
        if pursuer._agent_type == 'aacapursuer.safe_dists':
            pursuer.act(dt, args=[evader])
        else:
            pursuer.act(dt, evader)

        if plot and count%plot_rate == 0:
            plotStuff(evader, pursuer)
        time += dt
        count += 1
        end_condition = checkEndConditions(evader, pursuer, time, printer)

    return end_condition

    

def sim_velocity_space():

    vels = [1, 11]

    goal = [100, 0]

    disc = 50
    vs = np.linspace(vels[0], vels[1], num=disc)

    state_space = np.zeros([disc, disc])

    for i, ev in enumerate(vs):
        for j, pv in enumerate(vs):

            evader_start  = [0.0, 0.0, 0.0, ev]
            pursuer_start = [20.0, 0.0, np.pi, pv]

            # evader = base_agent(state=evader_start, goal=goal)
            evader = aacas_agent(state=evader_start, goal=goal, vmax=ev)
            pursuer = pursuer_agent(state=pursuer_start, goal=evader._state, vmax=pv)
            # pursuer = pursuer_agent(goal=evader._state[:2])

            state_space[i,j] = run_game(evader, pursuer, plot=False)

            # if state_space[i,j] == -1:
            #     evader = aacas_agent(state=evader_start, goal=goal)
            #     pursuer = pursuer_agent(state=pursuer_start, goal=evader._state[:2])
    
            #     state_space[i,j] = run_game(evader, pursuer, plot=True)

        print('Done row %i/%i' %(i+1,disc))

    unique, counts = np.unique(state_space, return_counts=True)
    colls = counts[0]
    avoids= state_space.size - colls
    print('Number of Collisions: %i' %(colls))
    print('Number of Avoids: %i' %(avoids))
    print('Avoid Rate: %.2f%%' %(100*avoids/(avoids+colls)))


    plotVelocitySpace(state_space.T, vels, vels, disc)

def sim_state_space():

    p_x_r = [0, 20]
    p_y_r = [-10, 10]
    p_th_r = [-np.pi, np.pi]
    vel_max = 3
    evader_vel = 4
    pursuer_vel= 4

    goal = [20, 0]
    evader_start = [0.0, 0.0, 0.0,evader_vel]
    pursuer_start = [20.0, 0.0, 0.0,pursuer_vel]

    disc = 50
    xs = np.linspace(p_x_r[0], p_x_r[1], num=disc)
    ys = np.linspace(p_y_r[0], p_y_r[1], num=disc)
    ths = np.linspace(p_th_r[0], p_th_r[1], num=disc)

    state_space = np.zeros([disc, disc])

    for i, px in enumerate(xs):
        for j, py in enumerate(ys):
            # for pth in ths:
            pth = np.pi
            pth = np.arctan2(py,px) + np.pi

            pursuer_start= [px, py, pth, vel_max]

            # evader = base_agent(state=evader_start, goal=goal)
            evader = aacas_agent(state=evader_start, goal=goal, vmax=evader_vel)
            pursuer = pursuer_agent(state=pursuer_start, goal=evader._state, vmax=pursuer_vel)
            # pursuer = pursuer_agent(goal=evader._state[:2])

            state_space[i,j] = run_game(evader, pursuer, plot=False)

            # if state_space[i,j] == -1:
            #     evader = aacas_agent(state=evader_start, goal=goal)
            #     pursuer = pursuer_agent(state=pursuer_start, goal=evader._state[:2])
    
            #     state_space[i,j] = run_game(evader, pursuer, plot=True)

        print('Done row %i/%i' %(i+1,disc))

    unique, counts = np.unique(state_space, return_counts=True)
    colls = counts[0]
    avoids= state_space.size - colls
    print('Number of Collisions: %i' %(colls))
    print('Number of Avoids: %i' %(avoids))
    print('Avoid Rate: %.2f%%' %(100*avoids/(avoids+colls)))

    plotStateSpace(state_space.T, p_x_r, p_y_r, p_th_r, disc)
    

def easy_run():
    plt.ion()

    goal = [20, 0]
    vel_max = 5
    evader_vel = 4
    pursuer_vel= 4
    evader_start = [0.0, 0.0, 0.0, evader_vel]
    pursuer_start= [10, 10.0, np.pi, pursuer_vel]
    # pursuer_start= [10, -10.0, np.pi/2, pursuer_vel]
    # pursuer_start[2] = np.arctan2(pursuer_start[1],pursuer_start[0])

    aacas = aacas_agent(state=evader_start, goal=goal, vmax=evader_vel)
    # pursuer = aacas_agent(state=pursuer_start, goal=[0,0], vmax=pursuer_vel)
    # pursuer = base_agent(state=pursuer_start, goal=[0,0], vmax=pursuer_vel)
    pursuer = pursuer_agent(state=pursuer_start, goal=aacas._state, vmax=pursuer_vel)
    # pursuer = pursuer_agent(goal=evader._state[:2])
    
    run_game(aacas, pursuer, printer=True)


# easy_run()
# sim_state_space()
sim_velocity_space()