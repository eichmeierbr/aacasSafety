import numpy as np


class base_agent:
    def __init__(self, state = np.zeros(4), goal = np.array([10,10]), vmax=3, kp=[1,1], safe_rad=2):
        self._state = np.array(state)
        self._goal  = np.array(goal)
        self._vmax  = vmax
        self._kp    = np.array(kp)*2
        self._state[3] = self._vmax
        self._safe_radius = safe_rad
        self._agent_type = 'base'



    def kinematics(self, u, dt):
        self._state[0] += self._state[3] * np.cos(self._state[2])*dt
        self._state[1] += self._state[3] * np.sin(self._state[2])*dt
        self._state[2] += u[0] * dt
        self._state[3] += u[1] * dt

        self._state[2] = self.wrap2pi(self._state[2])
        
    def act(self, dt, args=None):

        command = self.getAction()
        self.kinematics(command, dt)


    def getAction(self):
        g = self._goal[:2] - self._state[:2]
        
        # Velocity Control
        dist2goal = np.linalg.norm(g)
        v_g = self._vmax * (1- np.exp(-dist2goal**2/1))
        v_d = self._kp[1] * (v_g - self._state[3])

        
        # Heading Control
        th_des = np.arctan2(g[1], g[0])
        th_err = self.wrap2pi(th_des-self._state[2])
        w_d = self._kp[0]*th_err

        return np.array([w_d, v_d])



    def wrap2pi(self, ang):
        return (ang + np.pi) % (2*np.pi) - np.pi
    

class pursuer_agent(base_agent):
    def __init__(self, state = np.zeros(4), goal = np.array([10,10]), vmax=3, kp=[1,1], safe_rad=2):
        super().__init__(state, goal, vmax, kp, safe_rad)
        # self._state[3] = self._vmax
        self._agent_type = "pursuer"

    def get_goal(self, evader_state):
        dist = np.linalg.norm(evader_state[:2] - self._state[:2])
        d_vel = self._state[3] + evader_state[3]
        lead = min(dist, d_vel * (1.0 - np.exp(-dist)))
        gx = evader_state[0] + lead*np.cos(evader_state[2])
        gy = evader_state[1] + lead*np.sin(evader_state[2])
        return np.array([gx, gy])
        
    def act(self, dt, evader):

        self._goal = self.get_goal(np.copy(evader._state))
        # self._goal = evader._state[:2]
        command = self.getAction()
        self.kinematics(command, dt)


