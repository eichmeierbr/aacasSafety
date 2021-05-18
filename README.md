# Safety Analysis for AACAS

The work in this project was done for the final project in the Safe Robotics course 16-883 taught by Professor Changliu Liu at Carnegie Mellon University. The paper attached below utilizes an adversarial evader-pursuer game to test the safety characteristics of the controller used in the [AACAS project](https://mrsdprojects.ri.cmu.edu/2020teame/). The paper explores modest improvements to the algorithm to adaptively resolve assumptions made in the previous work.


## [Report](./aacasSafetyAnalysisandExtension.pdf)
<object data="aacasSafetyAnalysisandExtension.pdf" type="application/pdf" width="700px" height="700px">
    <embed src="aacasSafetyAnalysisandExtension.pdf">
        <p>This browser does not support PDFs. Please download the PDF to view it: <a href="aacasSafetyAnalysisandExtension.pdf">Download PDF</a>.</p>
    </embed>
</object>

## Running the Code

The code is set up to run a simple kinematic simulation with two dubin's car agents. The main function is contained in [safeSim.py](./safeSim.py). The bottom of the file gives 3 options to run the code:

### easy_run (lines 145-165)

Easy run performs a single simulation. The user can readily select the evader and puruser from the three agent options (AACAS, adversarial, and base_agent).

### Sim_state_space (lines 90-142)

This setting discretizes the area in front of the evader until the end goal. A simulation runs with the pursuer beginning at each point in this discretized grid. All required arguments are passed in from the main function

### Sim_velocity_space (lines 50-87)

This setting performs an avoidance test by discretizing the a range of possible velocity values. The velocity space is then filled by testing all pairs of evader and puruser velocities. The evader is initialized pointed towards the goal, and the puruser is initialized at the [20,0] (line 66) looking towards the evader.

## Adjusting the Agents:

### Base Agent (agent.py)

The base_agent runs the kinematics and simulation interface for the evader and pursuer. It also implements a simple go-to-goal dubins car agent. 

An important parameter of interest is the self.max_command (line 13) that restricts the maximum allowable command sent to the controller.

### Pursuer (agent.py)

The pursuer is nearly identical to the base agent except that it sets its goal as a point in front of the evading agent. At each time step it computes an adversarial collision point for its goal, then proceeds to act as if it were the base agent.

### Safety Agent - AACAS (safeAgent.py)

The functionality of the AACAS controller is described in the paper. The parameters of interest for running the code can all be found in the constructor (lines 7-25). The main parameters of interest are the velocity_avoid_mod, the change_orbit_wait, and the ignore_behind_mod.

The implemented safety behaviors can all be found in the getSafeDistance function (lines 199-203). It computes the safety radius and orbit convergence coefficient for each obstacle in the test scenario. You can disable the adaptive functionality by commenting out this function (or simply entering a return on line 200).