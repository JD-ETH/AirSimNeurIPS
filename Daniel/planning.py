import airsimneurips as asim
import os.path
import numpy as np
import math
import quaternion
from collections import deque

import quadrocoptertrajectory as quadtraj

def computeQuaternion(normal):
    zaxis = np.array([0,0,-1])
    # Compute the orientation. how z axis
    q = np.zeros(4)
    q[1:] = np.cross(zaxis, normal)
    q[0] = np.dot(zaxis, normal)
    assert(abs(np.linalg.norm(q) - 1) < 0.000001)
    return q

class RapidPlanner(object):
    def __init__(self, fmin, fmax, wmax, sample_rate, resolution=0.05):
        self.fmin = fmin
        self.fmax = fmax
        self.wmax = wmax
        self.sample_rate = sample_rate
        self.resolution = resolution
        # Define how gravity lies:
        self.gravity = [0,0,9.81]
        self.g = 9.81
        # Best and worst case scenerio.
        self.amax = 10 # m/ss
        self.vmin = 0.2 # m/s
        self.vmax = 20
        self.acc_dist = self.vmin * (self.vmin / self.amax) / 2.0
        self.minTimeSec = 0.05
        self.minPlanningTime = 0.8
        self.goalPosition = None
        self.goalVelocity = None
        self.goalAccerleration = None

    def getExtendedPath(self, goal, set_velz=None):
        assert(len(self.goalPosition)>0)
        traj = quadtraj.RapidTrajectory(self.goalPosition.tolist(),
            self.goalVelocity.tolist(),
            self.goalAccerleration.tolist(),
            self.gravity)
        goal_pos = goal[:3]
        traj.set_goal_position(goal_pos)
        traj.set_goal_acceleration([0,0,0])

        if set_velz:
            print("Special case with set vz: ",set_velz*goal[3:])
            traj.set_goal_velocity(set_velz*goal[3:])

        # print("PLANNING: Start: ", self.goalPosition.tolist())
        # print("PLANNING: Vel: ", self.goalVelocity.tolist())
        # print("PLANNING: Acc: ", self.goalAccerleration.tolist())
        # print("PLANNING: Goal: ", goal_pos)

        distance =  np.linalg.norm(self.goalPosition - np.array(goal_pos))

        return self.binaryCompute(traj, distance)

    def binaryCompute(self, traj, distance):
        # Have a first estimate of the best and worst performance.
        slowest_t = (distance-self.acc_dist)/self.vmin + self.vmin/self.amax
        fastest_t = math.sqrt(2*distance/self.amax)
        fastest_t = max(fastest_t, distance/self.vmax)
        assert(slowest_t > fastest_t)
        traj.generate(slowest_t)
        feasibility = traj.check_input_feasibility(self.fmin, self.fmax, self.wmax, self.minTimeSec)
        while  feasibility != 0:
            slowest_t *= 2
            if slowest_t > 10000:
                traj.unset_orientation_bound()
            assert(slowest_t < 1e10)
            traj.generate(slowest_t)
            feasibility = traj.check_input_feasibility(self.fmin, self.fmax, self.wmax, self.minTimeSec)
        # Binary search.
        count = 0
        mid_t = 0
        #
        while abs(slowest_t - fastest_t)>self.resolution :
            mid_t = slowest_t/2.0 + fastest_t/2.0
            traj.generate(mid_t)
            feasibility = traj.check_input_feasibility(self.fmin, self.fmax, self.wmax, self.minTimeSec)
            count +=1
            if feasibility==0:
                slowest_t = mid_t
            else :
                fastest_t = mid_t + self.resolution

        if feasibility!= 0:
            mid_t = slowest_t
            traj.generate(mid_t)
            feasibility = traj.check_input_feasibility(self.fmin, self.fmax, self.wmax, self.minTimeSec)

        # Return the shortest path.
        time = [i for i in np.arange(0,mid_t,self.sample_rate)]
        num_samples = len(time)
        reference = np.zeros((num_samples, 14))
        for i in range(num_samples):
            t = time[i]
            reference[i, :3] = traj.get_position(t)
            reference[i, 3:7] = computeQuaternion(traj.get_normal_vector(t))
            reference[i, 7:10] = traj.get_velocity(t)
            reference[i, 10] = traj.get_thrust(t)
            # This is still in inertial frame
            q = quaternion.quaternion(reference[i, 3],reference[i, 4],reference[i, 5],reference[i, 6])

            reference[i, 11:14] = quaternion.rotate_vectors(q.conj(),traj.get_body_rates(t))

        final_t = mid_t
        self.goalPosition = traj.get_position(final_t)
        self.goalVelocity = traj.get_velocity(final_t)
        self.goalAccerleration = traj.get_acceleration(final_t)
        return reference

    def getShortestPath(self, start_state, goal, set_velz = None):
        traj = quadtraj.RapidTrajectory(start_state.position.to_numpy_array().tolist(),
            start_state.linear_velocity.to_numpy_array().tolist(),
            start_state.linear_acceleration.to_numpy_array().tolist(),
            self.gravity)

        goal_pos = goal[:3]
        traj.set_goal_position(goal_pos)
        distance =  np.linalg.norm(start_state.position.to_numpy_array() - np.array(goal_pos))
        traj.set_goal_acceleration([0,0,0])
        if set_velz:
            print("Special case with set vz: ",set_velz*goal[3:])
            traj.set_goal_velocity(set_velz*goal[3:])
        # traj.set_goal_acceleration_in_axis(2,0)

        # print("PLANNING: Start: ", start_state.position.to_numpy_array().tolist())
        # print("PLANNING: Start: ", start_state.linear_velocity.to_numpy_array().tolist())
        # print("PLANNING: Start: ", start_state.linear_acceleration.to_numpy_array().tolist())
        # print("PLANNING: g: ", self.gravity)
        # print("PLANNING: Goal: ", goal_pos)
        # print("PLANNING: Limits: ", self.fmin, self.fmax, self.wmax, self.minTimeSec)

        return self.binaryCompute(traj, distance)
