import control
import planning
import airsimneurips as asim
import numpy
import time

# Ideas:
#  include drag:https://github.com/microsoft/AirSim/blob/18b36c7e3ea3d1e705c3938a7b8462d44bd81297/AirLib/include/vehicles/multirotor/MultiRotor.hpp#L191
# linear_drag_coefficient = 1.3f / 4.0f;  air_density = 1.225f;
# Inclination limit makes big difference
# predictive over goal
# Input constraints modify at mpc
# increase resolution

if __name__ == "__main__":

# Extending to new goal:  [  14.24904251 -155.98028564   10.91945133    0.34202044    0.32139261
#     0.88302254]
# Current state:  <Vector3r> {   'x_val': 13.273137092590332,
#     'y_val': -156.78729248046875,
#     'z_val': 9.852869033813477}  orient  <Quaternionr> {   'w_val': 0.8830069899559021,
#     'x_val': -0.2556034326553345,
#     'y_val': 0.3794059455394745,
#     'z_val': 0.10496046394109726}
# Special case with set vz:  [1.02606132 0.96417784 2.64906762]

    # planner = planning.RapidPlanner(0,19.72,6.28,0.02,0.05)
    # planner.getShortestPath(state, numpy.array( [  6.3731294 , 81.437416 , -43.879955 ]))
    #
# Current state:  <KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': 166.14730834960938,
#     'y_val': -1.4056991338729858,
#     'z_val': 1.1516906023025513},
#     'angular_velocity': <Vector3r> {   'x_val': 2.1352334022521973,
#     'y_val': 0.6188440322875977,
#     'z_val': 0.017196346074342728},
#     'linear_acceleration': <Vector3r> {   'x_val': 0.6800534725189209,
#     'y_val': -0.46433937549591064,
#     'z_val': 1.6281375885009766},
#     'linear_velocity': <Vector3r> {   'x_val': -6.9823994636535645,
#     'y_val': 4.4047698974609375,
#     'z_val': -0.12542438507080078},
#     'orientation': <Quaternionr> {   'w_val': 0.9962534308433533,
#     'x_val': -0.01565200462937355,
#     'y_val': -0.020513707771897316,
#     'z_val': -0.08254285156726837},
#     'position': <Vector3r> {   'x_val': -17.122417449951172,
#     'y_val': 45.57592010498047,
#     'z_val': -47.230995178222656}}


    planner = planning.RapidPlanner(0,19.72,6.28,0.05,0.05)
    state = asim.KinematicsState()
    state.position = asim.Vector3r(77.43971252, -96.87151337,  -5.48000002)
    # state.linear_velocity = asim.Vector3r(7.5843505859375,  0.7305274605751038, 6.088780403137207)
    # state.linear_acceleration =  asim.Vector3r(-2.003018379211426, -0.30256304144859314, -5.39830207824707)
    # state.orientation =  asim.Quaternionr(-0.012809118255972862, 0.05772315710783005, -0.03406976908445358,0.9976689219474792)
    # state.position.x_val = 6.373129367828369
    # state.position.y_val = 81.43741607666016
    # state.position.z_val = -42.87995529174805

    # path = planner.getShortestPath(state, numpy.array( [  6.373129367828369 , 81.43741607666016 , -43.87995529174805]))

    path = planner.getShortestPath(state, numpy.array([-111.93122864, 120.21295929 , -46.08000031 ,  -0.64279248 ,   0.76604036,0.]))
    
    # path2 = planner.getExtendedPath(numpy.array([ 0,  -1 , -20]))
    # raw_path = numpy.concatenate((path, path2))
    # path2 = planner.getExtendedPath(numpy.array([ 10.388415,  80.77406, -43.579998]))
    # path3 = planner.getExtendedPath(numpy.array([ 18.110466  ,76.26078,  -43.579998]))
    mpc_control = control.MpcControl(20)
    state_read = asim.KinematicsState()

    # state_read.orientation.w_val =  0.70710678118
    # state_read.orientation.z_val =  0.70710678118

    # state_read.orientation.w_val =  0.984807550907135
    # state_read.orientation.x_val =  0.0008127406472340226
    # state_read.orientation.y_val =  -0.0021525132469832897
    # state_read.orientation.z_val =  -0.17364919185638428
    # state_read.angular_velocity.x_val =  -0.030123945325613022
    # state_read.angular_velocity.y_val =  0.0011088978499174118
    # state_read.angular_velocity.z_val = 7.625947910128161e-05
    # state_read.linear_velocity.x_val =  0.0
    # state_read.linear_velocity.y_val =  0.0
    # state_read.linear_velocity.z_val =  -0.24393419921398163
    # state_read.position.z_val =  -0.07
    # state_read.angular_velocity.y_val =   1.946580171585083
    # state_read.angular_velocity.z_val =  -0.0002931684139184654

    # state_read.position.x_val = 6.373129367828369
    # state_read.position.y_val = 81.43741607666016
    # state_read.position.z_val = -42.87995529174805 #-43.689579010009766

    mpc_control.set_traj(path, state.orientation)
    # mpc_control.append_traj(path2)
    assert(mpc_control.tracking_status(state)==0)
    # cur = time.time()
    # for i in range(1000):
    #     u = mpc_control.getInput(state)
    # print("Spent time", (time.time()-cur)/1000)

    ref,_ = mpc_control.getReference(state)
    X, U = mpc_control.getFullMpcOutput(state)

    # print(ref[:,3:7]-X[:20,3:7])
    import visualization
    visualization.draw(path, X, U, 10.0)
