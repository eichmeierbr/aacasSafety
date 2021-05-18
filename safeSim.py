import numpy as np
import matplotlib.pyplot as plt

from agent import *
from safeAgent import *
from utils import *



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
        if pursuer._agent_type == 'aacas':
            pursuer.act(dt, args=[evader])
        else:
            pursuer.act(dt, evader)

        if plot and count%plot_rate == 0:
            plotStuff(evader, pursuer)
        time += dt
        count += 1
        end_condition = checkEndConditions(evader, pursuer, time, printer)

    return end_condition

    

def sim_velocity_space(vel_range, goal, discretization, show_failures=False):

    ## Initialize velocity
    vels = vel_range
    disc = discretization

    ## Initialize test velocities
    vs = np.linspace(vels[0], vels[1], num=disc)

    ## Initialize state space
    state_space = np.zeros([disc, disc])

    for i, ev in enumerate(vs):
        for j, pv in enumerate(vs):

            evader_start  = [0.0, 0.0, 0.0, ev]
            pursuer_start = [20.0, 0.0, np.pi, pv]

            ## Choose the Agents
            # evader = base_agent(state=evader_start, goal=goal)
            evader = aacas_agent(state=evader_start, goal=goal, vmax=ev)
            pursuer = pursuer_agent(state=pursuer_start, goal=evader._state, vmax=pv)

            state_space[i,j] = run_game(evader, pursuer, plot=False)

            ## Optional code to visualize collisions
            if state_space[i,j] == -1 and show_failures:
                evader = aacas_agent(state=evader_start, goal=goal)
                pursuer = pursuer_agent(state=pursuer_start, goal=evader._state[:2])
    
                state_space[i,j] = run_game(evader, pursuer, plot=True)

        print('Done row %i/%i' %(i+1,disc))

    ## Print results
    printResults(state_space)

    plotVelocitySpace(state_space.T, vels, vels, disc)


def sim_state_space(evader_vel, pursuer_vel, goal, discretization, x_range=None, y_range=None, show_failures=False):

    ## Initialize Evader and pursuer
    evader_start = [0.0, 0.0, 0.0,evader_vel]
    pursuer_start = [30.0, 0.0, 0.0,pursuer_vel]

    ## Set bounds for pursuer starting points
    if x_range == None:
        p_x_r = [0, goal[0]]
    else:
        p_x_r = x_range
    if y_range == None:
        p_y_r = [-goal[0]/2, goal[0]/2]
    else:
        p_y_r = y_range

    ## Discretize pursuer starting points
    disc = discretization
    xs = np.linspace(p_x_r[0], p_x_r[1], num=disc)
    ys = np.linspace(p_y_r[0], p_y_r[1], num=disc)

    state_space = np.zeros([disc, disc])

    for i, px in enumerate(xs):
        for j, py in enumerate(ys):

            ## Precompute optimal theta and set pursuer start            
            pth = np.arctan2(py,px) + np.pi
            pursuer_start= [px, py, pth, pursuer_vel]
        
            ## Initialize evader
            # evader = base_agent(state=evader_start, goal=goal)
            evader = aacas_agent(state=evader_start, goal=goal, vmax=evader_vel)

            ## Initialize pursuer
            pursuer = pursuer_agent(state=pursuer_start, goal=evader._state, vmax=pursuer_vel)
            pursuer.set_opt_theta()

            state_space[i,j] = run_game(evader, pursuer, plot=False)

            ## Optional code to show failures
            if state_space[i,j] == -1 and show_failures:
                evader = aacas_agent(state=evader_start, goal=goal)
                pursuer = pursuer_agent(state=pursuer_start, goal=evader._state[:2])
                state_space[i,j] = run_game(evader, pursuer, plot=True)
     

        print('Done row %i/%i' %(i+1,disc))

    ## Compute Collision Statistics
    printResults(state_space)

    plotStateSpace(state_space.T, p_x_r, p_y_r, disc)
    

def easy_run():
    plt.ion()

    ## Set agent velocities
    evader_vel = 1
    pursuer_vel= 10

    ## Set agent start and end points
    goal = [8.0*(evader_vel+pursuer_vel), 0]
    evader_start = [0.0, 0.0, 0.0, evader_vel]
    pursuer_start= [20.0, 0.0, np.pi, pursuer_vel]
    # pursuer_start= [10, -10.0, np.pi/2, pursuer_vel]
    # pursuer_start[2] = np.arctan2(pursuer_start[1],pursuer_start[0])

    ## Initialize Agents
    aacas = aacas_agent(state=evader_start, goal=goal, vmax=evader_vel)
    # pursuer = aacas_agent(state=pursuer_start, goal=[0,0], vmax=pursuer_vel)
    # pursuer = base_agent(state=pursuer_start, goal=[0,0], vmax=pursuer_vel)
    pursuer = pursuer_agent(state=pursuer_start, goal=aacas._state, vmax=pursuer_vel)
    
    run_game(aacas, pursuer, printer=True)


if __name__ == "__main__":
    # easy_run()
    # sim_state_space(evader_vel=4, pursuer_vel=4, goal=[30,0], discretization=10)
    sim_velocity_space(vel_range=[1,11], goal=[100,0], discretization=10)