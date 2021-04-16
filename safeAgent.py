import numpy as np
import operator
from agent import *

class aacas_agent(base_agent):
    def __init__(self, state = np.zeros(4), goal = np.array([10,10]), vmax=3, kp=[1,1], safe_rad=5):
        super().__init__(state, goal, vmax, kp)

        self._agent_type = "aacas"

        self.pos = self._state[:2]

        self.velocity_avoid_mod = 1.75
        self.min_safe_dist = safe_rad
        self.safe_dist = safe_rad
        self.time = 0

        self.closeObjects = []
        self.detections = []

        self.orbit_changes = {}
        self.orbit_dict = {}
        self.change_orbit_wait_ = .0
        self.k_conv = 0.01

        
    def act(self, dt, args=[]):
        pursuers = args
        self.updateDetections(pursuers)
        self.pos = self._state[:2]

        command = self.getAction()
        self.kinematics(command, dt)
        self.time += dt

    def getAction(self):
        vel_xy = self.getVelDes()

        # Velocity Control
        dist2goal = np.linalg.norm(self._goal-self._state[:2])
        v_g = self._vmax * (1- np.exp(-dist2goal**2/1))
        v_d = self._kp[1] * (v_g - self._state[3])
        
        # Heading Control
        th_des = np.arctan2(vel_xy[1], vel_xy[0])
        th_err = self.wrap2pi(th_des-self._state[2])
        w_d = self._kp[0]*th_err

        return np.array([w_d, v_d])


    def getVelDes(self, switchGoal=True):
        velDes = np.zeros(2)

        # Check if we are close to an object
        closeObjects, avoid = self.getCloseObjects()
        
        # If close to object, orbit
        if avoid:
            vels = []

            closeObjects.sort(key=operator.attrgetter('distance'))
            for i in range(len(closeObjects)):
                lastChangeTime = self.time - self.orbit_changes[closeObjects[i].id]
                if self.change_orbit_wait_ < lastChangeTime:
                    self.decideOrbitDirection(closeObjects[i])
                    closeObjects[i].orbit = self.freq
                    self.orbit_dict[closeObjects[i].id] = self.freq
    
                    for j in range(i):
                        dist= np.linalg.norm(closeObjects[j].position - closeObjects[i].position)
                        if dist < self.min_gap:
                            closeObjects[i].orbit = copy.copy(closeObjects[j].orbit)
                            self.orbit_dict[closeObjects[i].id] = closeObjects[i].orbit
                            break

                self.orbit_changes[closeObjects[i].id] = self.time

            for i in range(len(closeObjects)):
                if not closeObjects[i].id in self.orbit_dict: continue

                self.getSafeDistance(closeObjects[i])
                
                self.freq = self.orbit_dict[closeObjects[i].id]
                vel = self.getOrbit(closeObjects[i].position)
                mod = 1/(closeObjects[i].distance)
                # mod = np.exp(-1/(3*obstacle.distance))
                vels.append(vel * mod)
            velDes[:3] = np.sum(vels,axis=0)
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self._vmax
            # print('AVOID')

        ## If there are no nearby objects to avoid
        else: # Go to goal 
            velDes = self._goal[:2] - self._state[:2]
            # print('GO')

        return velDes


    def getCloseObjects(self):
        closeObjects = []
        move = False

        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()    
        
        for obst in self.in_detections:
            obs_pos = obst.position[:]
            self.getSafeDistance(obst)

            pos = np.array([obs_pos[0], obs_pos[1], 1])
            obst_trans = np.matmul(T_vo, pos)
            if obst_trans[1] > -0.5 and obst.distance < self.safe_dist:
                closeObjects.append(obst)
                move = True
        return closeObjects, move
        

    def transformToGoalCoords(self):

        dp = self._goal - self.pos

        th = np.arctan2(dp[1],dp[0]) - np.pi/2
        th = np.arctan2(np.sin(th), np.cos(th))
        R_ov = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t_ov = self.pos[:2]

        tempVect = np.matmul((-1 * R_ov.T),  t_ov)
        T_vo = np.array([[R_ov[0,0], R_ov[1,0], tempVect[0]], [R_ov[0,1], R_ov[1,1], tempVect[1]], [0, 0, 1]])

        return T_vo
    

    def updateDetections(self, pursuers=[]):
        # in_detections = self.query_detections_service_(vehicle_position=self.pos_pt, attitude=self.quat)
        in_detections = pursuers[:]
        self.in_detections = []
        for idx, obj in enumerate(in_detections):

            newObj = Objects()
            newObj.position = obj._state[:2]
            newObj.velocity = np.array([obj._state[3]*np.cos(obj._state[2]),obj._state[3]*np.sin(obj._state[2])])
            newObj.acceleration = np.zeros(2)
            newObj.id = idx
            newObj.distance = np.linalg.norm(newObj.position-self._state[:2])
            if not newObj.id in self.orbit_dict:
                self.orbit_dict[newObj.id] = newObj.orbit
                self.orbit_changes[newObj.id] = -1000
            else:
                newObj.orbit = self.orbit_dict[newObj.id]
                newObj.last_orbit_change_ = self.orbit_changes[newObj.id]
            self.in_detections.append(newObj)

    def decideOrbitDirection(self, ob):
        # Note: All directions are assuming the vehicle is looking
        # straight at the goal

        obst_vel = ob.velocity[:]
        obst_pos = ob.position[:]
        
        
        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()
        
        trans_vel = np.matmul(T_vo, [obst_vel[0], obst_vel[1], 0])
        trans_pos = np.matmul(T_vo, [obst_pos[0], obst_pos[1], 1])


        t_y = trans_pos[1]/(self._vmax - trans_vel[1])
        trans_pos[0] += self.velocity_avoid_mod*trans_vel[0]*t_y

        if(trans_pos[0] >= 0):  # If object is to the right
            self.freq = 1       # Orbit CW
        else:                   # If object is to the left
            self.freq = -1      # Orbit CCW

        self.last_orbit_change_ = self.time
        ob.last_orbit_change_ = self.last_orbit_change_
        ob.orbit = self.freq


    def getOrbit(self, center):
        xhat = self.pos[:2] - center[:2] # Change to orbit coords
        gam = self.k_conv*(self.safe_dist**2 - np.matmul(xhat, xhat) ) # Convergence to orbit

        A = np.array([[gam, self.freq], [-self.freq, gam]]) # Modified harmonic oscillator
        g = np.matmul(A, xhat[:2])   #  Calculate nominal velocity
        
        # Scale the vector field
        v_g = np.linalg.norm(g)
        g = self._vmax/v_g * g 
        
        # Pad output with z-vel
        velDes = np.array([g[0], g[1]])

        return velDes

    def getSafeDistance(self, obstacle):
        vel_sum = self._state[3] + np.linalg.norm(obstacle.velocity)
        self.safe_dist = max(self.min_safe_dist, vel_sum)
        self.k_conv = 0.0001 + np.exp(-obstacle.distance)
        # self.safe_dist = self.min_safe_dist


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), acc=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.acceleration = acc
        self.distance = dist
        self.last_orbit_change_ = -1000
        self.orbit = -1
