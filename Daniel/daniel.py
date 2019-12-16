from argparse import ArgumentParser
import airsimneurips as asim
import cv2
import threading
import time
import utils
import numpy as np
import math
import control
import planning
from termcolor import colored

# Ideas:
# Remove the odometry call restrictions or increase the frequency.
# Remove acceleration constraints on last ascent.
# Use immediate output instead.

class Daniel(object):
    def __init__(self, drone_name = "drone_1", viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0]):
        # Settings.
        self.drone_name = drone_name
        self.gate_poses = False
        self.gates_passed = 0
        # Conenction.
        self.client = asim.MultirotorClient()
        self.client.confirmConnection()
        self.odom = asim.MultirotorClient()
        self.odom.confirmConnection()

        self.ctl_period = 0.05
        # Planner related.
        self.g = 9.81
        self.fmin = 0.2 * self.g
        self.fmax = 2.0 * self.g # w.r.t to weight
        self.wmax = 6.0

        # 50 Hz control, 20 ms callback.
        self.next_input = None
        self.horizon = 20 # 0.8 second look ahead.
        self.mpc_control = None

        # Threads.
        # Odometry thread calls back current pose and finds optimal contro input.
        self.odometry_callback_thread = threading.Thread(
            target=self.repeat_timer_odometry_callback, args=(self.odometry_callback, ))
        self.is_odometry_thread_active = False
        self.control_thread = threading.Thread(
            target=self.apply_control, args=(self.ctl_period,))
        self.is_control_thread_active = False
        self.is_race_finished = False

        # Bug fix.
        self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS = 10

        # Init planner and controller.
        self.rapid_planner = planning.RapidPlanner(self.fmin, self.fmax, self.wmax, self.ctl_period)
        self.mpc_control = control.MpcControl(self.horizon)

        # Check beyond goal:
        #                     up   -   -   [stack1 2 3]-     -   tilt  down   -      cube1 2   3    4
        # Gate parameters at  0    1   2    3    4     5     6   7   8   9    10     11   12   13   14   15   16     17   18     19   20   21
        self.special_checks = []
        self.velocity_constraints = []
        self.default_look_ahead_s = 0.405
        self.default_distance_thresh = 8.0
        self.default_velocity = None
        self.fake_gate=[]
        self.passed = -1

        # load goals;
        self.goal_z_shift = 0.5
    def advance(self):
        self.gates_passed+=1
        self.gate_poses = self.gate_poses[1:]
        self.velocity_constraints = self.velocity_constraints[1:]
        self.special_checks = self.special_checks[1:]
        print("Advancing, Constraint at ", self.gates_passed, " speed ",
            self.velocity_constraints[0], " checks: ", self.special_checks[0],
            " goal at ", self.gate_poses[0])

    def beyond_goal(self, drone_state,look_ahead_s = 0.35, check_thresh_m = 2.0):
        beyond_current = False
        # print("Curret passed: ", self.gates_passed, ", ahead, dist", look_ahead_s, check_thresh_m)
        if (look_ahead_s < 0):
            beyond_current = self.odom.simGetLastGatePassed(vehicle_name="drone_1") == self.passed
            if beyond_current and self.passed not in self.fake_gate:
                self.passed += 1
            return beyond_current

        if np.linalg.norm(drone_state.position.to_numpy_array() - self.gate_poses[0,:3]) > check_thresh_m:
            return beyond_current

        drone_vec = drone_state.position.to_numpy_array() + \
                    look_ahead_s * drone_state.linear_velocity.to_numpy_array() -\
                    self.gate_poses[0,:3]
        beyond_current = np.dot(drone_vec, self.gate_poses[0,3:]) > 0
        if beyond_current and self.passed not in self.fake_gate:
            self.passed += 1
        return beyond_current

    def done(self):
        return self.is_race_finished

    # loads desired level
    def load_level(self, level_name, sleep_sec = 2.0):
        self.level_name = level_name
        self.client.simLoadLevel(self.level_name)
        self.client.confirmConnection() # failsafe
        time.sleep(sleep_sec) # let the environment load completely

    # Starts an instance of a race in your given level, if valid
    def start_race(self, tier=3):
        self.client.simStartRace(tier)

    # Resets a current race: moves players to start positions, timer and penalties reset
    def reset_race(self):
        self.client.simResetRace()


    # Load gates
    def load_traj(self):

        self.gate_poses = self.get_ground_truth_gate_poses()
        # Add intermediate gates.
        # Modifying gate 12.
        gate12 = self.gate_poses[11,:]
        gate12[:3] = gate12[:3] + 2 * gate12[3:]
        gate12[2] -= 2.0
        gate12[3:] = [0,0,-1]

        # Add one between 17 and 18.
        modify_ind = 17
        gate18 = np.zeros(6)
        diff = self.gate_poses[modify_ind,:3] -self.gate_poses[modify_ind-1,:3]
        gate18[:2] = self.gate_poses[modify_ind-1,:2] + diff[:2] * 0.7
        gate18[2] = self.gate_poses[modify_ind-1,2] + diff[2] * 0.8
        gate18[3:] = diff/np.linalg.norm(diff)
        self.gate_poses = np.insert(self.gate_poses, modify_ind, gate18, 0)
        self.fake_gate.append(modify_ind)

        # Add multiple gates between 20 and 21.
        # modify_ind = len(self.gate_poses)-1
        self.gate_poses[-1,2] -= 1.25
        # diff = self.gate_poses[modify_ind,:3] -self.gate_poses[modify_ind-1,:3]
        # ratio_climbing_section = 0.4
        # max_climb_distance = 22.5
        # max_distance_horizontal = 25
        # allow_height_drop = -2
        #
        # added = 0
        # # Compute the location of transition.
        # terminal_gate = np.copy(self.gate_poses[modify_ind-1])
        # terminal_gate[:2] = self.gate_poses[modify_ind-1,:2] + diff[:2] * (1-ratio_climbing_section)
        # terminal_gate[2] += allow_height_drop
        #
        # # Climbing section
        # dir = self.gate_poses[modify_ind, :3] - terminal_gate[:3]
        # dir = dir/np.linalg.norm(dir)
        # cur_gate = np.copy(self.gate_poses[modify_ind])
        # # while (np.linalg.norm(terminal_gate[:3] - cur_gate[:3]) > max_climb_distance):
        # #     added += 1
        # #     cur_gate[:3] = cur_gate[:3] - dir*max_climb_distance
        # #     self.gate_poses = np.insert(self.gate_poses, modify_ind, cur_gate, 0)
        # cur_gate[:3] = terminal_gate[:3] + dir*max_climb_distance
        # self.gate_poses = np.insert(self.gate_poses, modify_ind, cur_gate, 0)
        # added += 1
        # self.gate_poses = np.insert(self.gate_poses, modify_ind, terminal_gate, 0)
        #
        # # rush section
        # dir = terminal_gate[:3] - self.gate_poses[modify_ind-1, :3]
        # dir = dir/np.linalg.norm(dir)
        # # while (np.linalg.norm(self.gate_poses[modify_ind-1,:3]-terminal_gate[:3])) > max_distance_horizontal:
        # #     added += 1
        # #     terminal_gate[:3] = terminal_gate[:3] -dir * max_distance_horizontal
        # #     self.gate_poses = np.insert(self.gate_poses, modify_ind, terminal_gate, 0)
        # terminal_gate[:3] = self.gate_poses[modify_ind-1, :3] + dir * max_distance_horizontal
        # self.gate_poses = np.insert(self.gate_poses, modify_ind, terminal_gate, 0)
        # # print(self.gate_poses)
        # print("Added inbetween gates ", added)

        # Add zero gate.
        startoff_height = 2.5
        drone_state = self.odom.getMultirotorState().kinematics_estimated
        takeoff = np.zeros(6)
        takeoff[:3] = drone_state.position.to_numpy_array()
        takeoff[2] -= startoff_height
        takeoff[3:] = self.gate_poses[0,3:]
        self.gate_poses = np.vstack((takeoff, self.gate_poses))


        # final target beyond goal.
        final_dist = 30
        gate_beyond = np.zeros(6)
        gate_beyond[3:] = self.gate_poses[-1,3:]
        direct = self.gate_poses[-1,:3] - self.gate_poses[-2, :3]
        direct = direct /np.linalg.norm(direct)
        gate_beyond[:3] = direct * final_dist + self.gate_poses[-1,:3]
        gate_beyond[2] -= 10
        self.gate_poses = np.vstack((self.gate_poses, gate_beyond))

        length = len(self.gate_poses)
        self.special_checks = [(self.default_look_ahead_s, self.default_distance_thresh)] * (length)
        # self.special_checks[5] = (-0.1,1)
        # self.special_checks[9] = (-0.1,1.0)
        self.special_checks[12] = (-0.1,1.0)
        self.special_checks[13] = (-0.1,1.0)
        for i in range(17,len(self.special_checks)):
            self.special_checks[i] = (0.75,10.0)

        self.special_checks[-2] = (1,15.0)
        self.special_checks[-1] = (-1,5.0)
        # for i in range(22,len(self.special_checks)-3):
        #     self.special_checks[i] = (1.25,25.0)

        self.velocity_constraints = [self.default_velocity] * (length)
        self.velocity_constraints[8] = 2  # dropping
        # self.velocity_constraints[10] = 3.5
        self.velocity_constraints[11] = 3  # Getting in cube slow.
        self.velocity_constraints[12] = 2  # moving up befake gate
        self.velocity_constraints[13] = 2  # moving down fake gate
        # self.velocity_constraints[18] = 6
        # self.velocity_constraints[22] = 5

    # Arms drone and get optimal trajectory.
    def initialize_drone(self):
        self.client.enableApiControl(vehicle_name=self.drone_name)
        self.client.arm(vehicle_name=self.drone_name)

    def takeoffAsync(self):
        self.client.takeoffAsync()

    def odometry_callback(self):
        # get current state.0
        drone_state = self.odom.getMultirotorState().kinematics_estimated
        # check whether replanning is needed.
        # Updating the best action.
        # print("Drone wx: %.3f, wy: %.3f, wz: %.3f" %(drone_state.angular_velocity.x_val,drone_state.angular_velocity.y_val,drone_state.angular_velocity.z_val))
        # print("Drone velocity: %.5f" %(drone_state.linear_velocity.distance_to(asim.Vector3r())))

        # print("Drone velocity: %.5f vx, vy, vz: [%.5f, %.5f,%.5f]" %(drone_state.linear_velocity.distance_to(asim.Vector3r()),drone_state.linear_velocity.x_val,drone_state.linear_velocity.y_val,drone_state.linear_velocity.z_val))
        # print("Drone px, py, pz: [%.5f, %.5f,%.5f]" %(drone_state.position.x_val,drone_state.position.y_val,drone_state.position.z_val))
        # print("Drone qx, qy, qz, qw: [%.5f, %.5f, %.5f,  %.5f]: " %(drone_state.orientation.x_val,drone_state.orientation.y_val,drone_state.orientation.z_val,drone_state.orientation.w_val))

        # Immediately replan if finished.
        if self.beyond_goal(drone_state , self.special_checks[0][0], self.special_checks[0][1]):
            if self.odom.simGetLastGatePassed(vehicle_name="drone_1")==21:
                self.is_race_finished = True
                print(colored(("Race finished"), 'green'))
                return
            self.advance()
            # print("Current goal reached, gates left: ", len(self.gate_poses)-1, " Replanning.")
            # print("Current goal: ", self.gate_poses[0,:3])
            # print("Current state: ", drone_state)
            print(colored("Controller needs replan due to finished goal.", 'green'))
            traj = self.rapid_planner.getShortestPath(drone_state,self.gate_poses[0],self.velocity_constraints[0])
            self.mpc_control.set_traj(traj, drone_state.orientation)
            print(colored(("Planning finished"), 'green'))
            drone_state = self.odom.getMultirotorState().kinematics_estimated

        self.need_replan(drone_state)

    def plan_first(self):
        print(colored("Controller planning for first goal.", 'green'))
        drone_state = self.odom.getMultirotorState().kinematics_estimated
        traj = self.rapid_planner.getShortestPath(drone_state,
            self.gate_poses[0],self.velocity_constraints[0])
        self.mpc_control.set_traj(traj, drone_state.orientation)
        print(colored(("Planning finished for: ", self.gates_passed), 'green'))

    def need_replan(self, drone_state):
        tracking_performance = self.mpc_control.tracking_status(drone_state)
        while tracking_performance != 0:
            if tracking_performance == 4:
                print(colored("Controller needs replan due to deviation, enter stablizing mode.", 'red'))
                recover_pose = np.zeros(6)
                recover_pose[:3] = drone_state.position.to_numpy_array()
                recover_pose[2] -= 2.0
                recover_pose[5] = -1
                self.gate_poses = np.vstack((recover_pose, self.gate_poses))
                traj = self.rapid_planner.getShortestPath(drone_state,
                    self.gate_poses[0], 0)
                self.mpc_control.set_traj(traj, drone_state.orientation)
                print(colored("Planning finished.", 'red'))
            elif tracking_performance == 3 :
                print(colored("Controller needs replan due to controller saturation, replan.", 'yellow'))
                # print("Extending to new goal: ", self.gate_poses[0])
                # print("Current state: ", drone_state.position, " orient ", drone_state.orientation)
                traj = self.rapid_planner.getShortestPath(drone_state,
                    self.gate_poses[0],self.velocity_constraints[0])
                self.mpc_control.set_traj(traj, drone_state.orientation)
                print(colored("Planning finished.", 'yellow'))
            elif tracking_performance == 2 :
                print(colored("Controller needs replan due to orientation deviation, replan.", 'yellow'))
                # print("Extending to new goal: ", self.gate_poses[0])
                # print("Current state: ", drone_state.position, " orient ", drone_state.orientation)
                traj = self.rapid_planner.getShortestPath(drone_state,
                    self.gate_poses[0],self.velocity_constraints[0])
                self.mpc_control.set_traj(traj, drone_state.orientation)
                print(colored("Planning finished.", 'yellow'))
            elif tracking_performance == 1 :
                print(colored("Controller needs replan due to reference too short.", 'blue'))
                state = asim.KinematicsState()
                if len(self.gate_poses)==1:
                    self.is_race_finished = True
                    print(colored("Race Finished.", 'green'))
                    traj = self.rapid_planner.getShortestPath(
                        self.gate_poses[0],0)
                    self.mpc_control.set_traj(traj, drone_state.orientation)
                else:
                    traj = self.rapid_planner.getExtendedPath(
                        self.gate_poses[1],self.velocity_constraints[1])
                    self.mpc_control.append_traj(traj)
                # print("Extending to new goal: ", self.gate_poses[1, :3])
                # print("Current state: ", drone_state.position)

                print(colored("Planning finished.", 'blue'))

            tracking_performance = self.mpc_control.tracking_status(drone_state)

    def repeat_timer_odometry_callback(self, task):
        while self.is_odometry_thread_active:
            task()
            time.sleep(0.005)

    def start_odometry_callback_thread(self):
        if not self.is_odometry_thread_active:
            self.is_odometry_thread_active = True
            self.odometry_callback_thread.start()
            print("Started odometry callback thread")

    def stop_odometry_callback_thread(self):
        if self.is_odometry_thread_active:
            self.is_odometry_thread_active = False
            self.odometry_callback_thread.join()
            print("Stopped odometry callback thread.")



    # Apply the optimal control input.
    def apply_control(self, period):
        while self.is_control_thread_active:
            while not self.mpc_control.is_ready():
                time.sleep(0.005)
                print(colored("Controller misses reference, waiting...", 'red'))
            drone_state = self.client.getMultirotorState().kinematics_estimated
            input = self.mpc_control.getInput(drone_state)
            control_drone = self.client.moveByAngleRatesThrottleAsync(input.roll_rate,
                    input.pitch_rate, input.yaw_rate,
                    input.throttle, period, vehicle_name=self.drone_name)
            # print("T ", input.throttle, " r ", input.roll_rate, " p ", input.pitch_rate)
            control_drone.join()

    def start_control_thread(self):
        if not self.is_control_thread_active:
            self.is_control_thread_active = True
            self.control_thread.start()
            print("Started Control thread")

    def stop_control_thread(self):
        if self.is_control_thread_active:
            self.is_control_thread_active = False
            self.control_thread.join()
            print("Stopped control thread.")

    # stores gate ground truth poses as a list of airsim.Pose() objects in self.gate_poses_ground_truth
    def get_ground_truth_gate_poses(self):
        gate_names_sorted_bad = sorted(self.client.simListSceneObjects("Gate.*"))
        # gate_names_sorted_bad is of the form `GateN_GARBAGE`. for example:
        # ['Gate0', 'Gate10_21', 'Gate11_23', 'Gate1_3', 'Gate2_5', 'Gate3_7', 'Gate4_9', 'Gate5_11', 'Gate6_13', 'Gate7_15', 'Gate8_17', 'Gate9_19']
        # we sort them by their ibdex of occurence along the race track(N), and ignore the unreal garbage number after the underscore(GARBAGE)
        gate_indices_bad = [int(gate_name.split('_')[0][4:]) for gate_name in gate_names_sorted_bad]
        gate_indices_correct = sorted(range(len(gate_indices_bad)), key=lambda k: gate_indices_bad[k])
        gate_names_sorted = [gate_names_sorted_bad[gate_idx] for gate_idx in gate_indices_correct]
        # x,y,z,ux,uv,uz
        gate_poses_ground_truth = np.zeros((len(gate_names_sorted),6))
        ind = 0
        for gate_name in gate_names_sorted:
            curr_pose = self.client.simGetObjectPose(gate_name)
            # Small hack to account far getting stuck
            counter = 0
            while (math.isnan(curr_pose.position.x_val) or math.isnan(curr_pose.position.y_val) or math.isnan(curr_pose.position.z_val)) and (counter < self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS):
                print(f"DEBUG: {gate_name} position is nan, retrying...")
                counter += 1
                curr_pose = self.client.simGetObjectPose(gate_name)
            assert not math.isnan(curr_pose.position.x_val), f"ERROR: {gate_name} curr_pose.position.x_val is still {curr_pose.position.x_val} after {counter} trials"
            assert not math.isnan(curr_pose.position.y_val), f"ERROR: {gate_name} curr_pose.position.y_val is still {curr_pose.position.y_val} after {counter} trials"
            assert not math.isnan(curr_pose.position.z_val), f"ERROR: {gate_name} curr_pose.position.z_val is still {curr_pose.position.z_val} after {counter} trials"
            gate_poses_ground_truth[ind,3:] = get_gate_facing_vector_from_quaternion(curr_pose.orientation)
            gate_poses_ground_truth[ind,:3]= curr_pose.position.to_numpy_array()
            gate_poses_ground_truth[ind,2] -= self.goal_z_shift
            ind +=1
        gate_poses_ground_truth[-1,2] += 1.1
        return gate_poses_ground_truth

def get_gate_facing_vector_from_quaternion(airsim_quat):
    if abs(airsim_quat.w_val)>1:
        return np.array([airsim_quat.x_val,airsim_quat.y_val,airsim_quat.z_val])
    q = np.array([airsim_quat.w_val, airsim_quat.x_val, airsim_quat.y_val, airsim_quat.z_val], dtype=np.float64)
    n = np.dot(q, q)
    if n < np.finfo(float).eps:
        return airsim.Vector3r(0.0, 1.0, 0.0)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rotation_matrix = np.array([[1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
                                [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
                                [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])
    return rotation_matrix[:,1]

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    daniel = Daniel(drone_name="drone_1", viz_traj=args.viz_traj, viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0])
    daniel.load_level(args.level_name)
    if args.level_name == "Final_Tier_1":
        args.race_tier = 1
    if args.level_name == "Final_Tier_2":
        args.race_tier = 2
    if args.level_name == "Final_Tier_3":
        args.race_tier = 3
    daniel.load_traj()
    daniel.plan_first()
    # Starting to plan.
    daniel.start_race(args.race_tier)
    print("Race Started.")
    # Start API.
    daniel.initialize_drone()
    # Takes off.
    daniel.takeoffAsync()
    daniel.start_odometry_callback_thread()
    daniel.start_control_thread()

    # Wait until the race finishes.
    while not daniel.done():
        time.sleep(0.25)

    daniel.stop_control_thread()
    daniel.stop_odometry_callback_thread()
    print("Race Finished, start hovering.")
    time.sleep(1.0)
    daniel.reset_race()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard",
        "Qualifier_Tier_1", "Qualifier_Tier_2", "Qualifier_Tier_3", "Final_Tier_1","Final_Tier_2", "Final_Tier_3"], default="ZhangJiaJie_Medium")
    parser.add_argument('--enable_viz_traj', dest='viz_traj', action='store_true', default=False)
    parser.add_argument('--race_tier', type=int, choices=[1,2,3], default=1)
    args = parser.parse_args()
    main(args)
