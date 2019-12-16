import numpy as np
import acado
import time

# Time horizon
T=40
# Number of differential state variables.
NX=10
# Number of control inputs.
NU=4
# Number of input reference.
NY=14
# Number of final stage cost.
NYN = 10
# Initial state.
x0 = np.zeros((1,NX))
x0[0,3] = 1.0
# Unknowns.
X = np.zeros((T+1, NX))
U = np.zeros((T,NU))
# Reference hover state with input. Move to x equals 2.
xref = np.zeros((1,NX))
xref[0,2] = -1.0
xref[0,3] = 1.0
Y=np.zeros((T,NY))
for i in range(0,T):
  Y[i,2]=-1
  Y[i,3]=1
  Y[i,10]=1

# Final value.
yN=np.zeros((1,NYN))
yN[0,2] = -1
yN[0,3] = 1
Q = np.diag([100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0])
Qf = Q[:10, :10]


counter = 0
# x0 [1, NX]
# X [N+1, NX]
# U [N, NU]
# Y [N, NY]
# yN [1, NYN]
for i in range(0,1000):
  X, U = acado.mpc(0, 1, x0,X,U,Y,yN, np.transpose(np.tile(Q,T)), Qf, 0)
  if counter % 1000 == 0:
    print(counter)
  counter = counter + 1

print('U', U)
print('X', X)
