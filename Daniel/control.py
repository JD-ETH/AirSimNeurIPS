import airsimneurips as asim
import numpy as np
import quaternion
import math
from termcolor import colored

import sys, os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),'mpc_control/quadrotor_mpc_codegen'))
import acado
# solver for optimal control.

class ControlInput(object):
    def __init__(self, throttle, roll_rate, pitch_rate, yaw_rate):
        self.throttle = throttle
        self.yaw_rate = -yaw_rate
        self.pitch_rate = -pitch_rate
        self.roll_rate = roll_rate

class MpcControl(object):
    def __init__(self, horizon):
        # Iteration for approximating mpc.
        self.max_iteration = 12
        # Cost of mpc.
        self.Q = np.diag([100.0, 100.0, 300.0, 100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 30.0, 1.0, 1.0, 1.0, 1.0])
        self.Qf = self.Q[:10, :10]
        # Step size and horizon
        self.horizon = horizon
        # Copy from acado.
        self.NX = 10
        self.NY = 14
        self.NYN = 10
        self.NU = 4
        self.reference = None
        self.THRESHOLD = 0.01
        self.use_ith_control = 1
        # Internal checks.
        self.max_deviation_dist = 2.0
        self.max_deviation_angle = 12.0
        self.max_sudden_angle = 1.2
        self.acc_deviation_angle = 0
        self.discount = 0.25
        self.g = 9.81
        self.last_checked = None
        self.saturated = False
    def is_ready(self):
        return len(self.reference)>self.horizon+2

    def tracking_status(self, current):
        # 0 is healty.
        # 1 is too short, needs append.
        # 2 is orientation error, replan.
        # 3 is output saturation, replan.
        # 4 is position error, replan to emergency.
        # Find closest and pop front.
        if current != self.last_checked:
            self.last_checked = current
            mindist = sys.float_info.max
            # Find last position that decreases.
            for ind in range(len(self.reference)):
                ref = self.reference[ind]
                cur_dist = np.linalg.norm(ref[:3] - current.position.to_numpy_array())
                if cur_dist < mindist :
                    mindist = cur_dist
                else:
                    break

            # Check it's not too far off
            if  mindist > self.max_deviation_dist:
                print("Deviating from path: cur min: ", mindist)
                print("Shortest reference at: ", self.reference[0])
                print("Now at at: ", current.position)
                return 4

            if self.saturated:
                return 3

            # Remove indices up to that point.
            if ind>1:
                self.reference = self.reference[ind-1:]
                q_cur = quaternion.quaternion(current.orientation.w_val,current.orientation.x_val,current.orientation.y_val,current.orientation.z_val)
                q_ref = quaternion.quaternion(self.reference[0,3],self.reference[0,4],self.reference[0,5],self.reference[0,6])
                q_diff = q_cur/q_ref
                q_diff.normalized()
                if (abs(q_diff.w) >=1):
                    print(colored("Why is quaternion out of range?.", 'red'))
                    q_diff = quaternion.one
                angle =  2*math.acos(q_diff.w)
                if angle > self.max_sudden_angle:
                    self.acc_deviation_angle += angle
                else:
                    self.acc_deviation_angle -= self.discount
                    self.acc_deviation_angle = max(0,self.acc_deviation_angle)

            if self.acc_deviation_angle > self.max_deviation_angle:
                return 2

        if self.reference is None or len(self.reference) <= self.horizon*1.25:
            return 1

        return 0
    def reset_errors(self):
        self.acc_deviation_angle = 0

    def set_traj(self, traj, quat):
        q_init = quaternion.quaternion(quat.w_val, quat.x_val, quat.y_val, quat.z_val).normalized()
        assert(abs(q_init.norm()-1)<0.0001)
        for quat in traj:
            converted = q_init * quaternion.quaternion(quat[3],quat[4],quat[5],quat[6])
            quat[3:7] = [converted.w,converted.x,converted.y,converted.z]
            # quat[11:] = quaternion.rotate_vectors(converted, quat[11:])

        self.reference = traj
        self.reset_errors()

    def append_traj(self, traj):
        # Add the difference coming most likely from yaw.
        q_diff = quaternion.quaternion(self.reference[-1][3],self.reference[-1][4],self.reference[-1][5],self.reference[-1][6])/quaternion.quaternion(traj[0][3],traj[0][4],traj[0][5],traj[0][6])
        for quat in traj:
            converted = q_diff * quaternion.quaternion(quat[3],quat[4],quat[5],quat[6])
            quat[3:7] = [converted.w,converted.x,converted.y,converted.z]
            # quat[11:] = quaternion.rotate_vectors(converted, quat[11:])

        self.reference =np.concatenate((self.reference, traj))
        self.reset_errors()

    def getReference(self, current):
        # Check if it makes sense
        assert(self.NY == self.reference.shape[1])
        yn = np.zeros((1, self.NYN))
        assert(len(self.reference)>self.horizon)
        yn[0,:] = self.reference[self.horizon][:10]
        Y = self.reference[:self.horizon]
        return (Y, yn)

    def getInput(self, state):
        X, U = self.getFullMpcOutput(state)
        u_body_0 =  U[0,1:]
        T_0 = U[0,0]/2/self.g
        u_body =  U[self.use_ith_control,1:]
        T = U[self.use_ith_control,0]/2/self.g
        self.saturated = T_0 + abs(u_body_0[0])/40+ abs(u_body_0[1])/40 > 1.0
        return ControlInput(T,u_body[0],u_body[1],u_body[2])

        # Convert to body frame
        # q = quaternion.quaternion(state.orientation.w_val,state.orientation.x_val,
        # state.orientation.y_val,state.orientation.z_val)
        # u_body = quaternion.rotate_vectors(q.conj(), U[use_ith_control,1:])
        # return ControlInput(U[use_ith_control,0]/2/self.g,u_body[0],u_body[1],u_body[2])

    def getFullMpcOutput(self, state):
        # finish iter
        X = np.zeros((self.horizon+1, self.NX))
        U = np.zeros((self.horizon, self.NU))
        prev_x = np.zeros((self.horizon+1, self.NX))
        ref_traj, terminal_state = self.getReference(state)
        for i in range(self.max_iteration):
            X, U = acado.mpc(0, 1, self.toAcadoState(state), X, U, ref_traj, terminal_state,
                    np.transpose(np.tile(self.Q,self.horizon)), self.Qf, 0)
            if (np.linalg.norm(X-prev_x) < self.THRESHOLD):
                # print("CONTROL: Input mpc terminating iteration at ", i)
                break
            prev_x = X #Update prev
        return (X,U)

    def toAcadoState(self, state):
        x = np.zeros((1, self.NX))
        x[0,0] = state.position.x_val
        x[0,1] = state.position.y_val
        x[0,2] = state.position.z_val
        x[0,3] = state.orientation.w_val
        x[0,4] = state.orientation.x_val
        x[0,5] = state.orientation.y_val
        x[0,6] = state.orientation.z_val
        x[0,7] = state.linear_velocity.x_val
        x[0,8] = state.linear_velocity.y_val
        x[0,9] = state.linear_velocity.z_val
        return x
