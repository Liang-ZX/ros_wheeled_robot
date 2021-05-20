import math
import numpy as np


M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

############################################################### 
################## System description ########################
############################################################### 
# x_k = f(x_{k-1}, u_k) + w_k
# y_k = h(x_k) + v_k
# where h() and f() are nonlinear function
# x_{k-1}, x_k are the state vector at timestamp k-1 and k
# w_k and v_k are system noise and observation noise
# y_k is the observation at timestamp k
# Notation: Jf(x) and Jh(x) are the Jacobian matrix of f(x) and h(x) 
#######################################################################

# EKF system noise density: Qx = E[w_k * w_k']
Qx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2

class EKF():
    def __init__(self):
        pass
    
    # input: posteriori estimated state and its covariance at timestamp k, system input u
    # output: posteriori estimated state and its covariance at timestamp k+1
    def estimate(self, xEst, PEst, z, u):

        # Predict
        # motion model: odom_model
        # uncertainty propagation: jacob_motion

        # Update for each observation
        return xEst, PEst

    # input: posteriori estimated state at timestamp k, system input u
    # output: priori estimated state at timestamp k+1 
    def odom_model(self, x, u):
        """
            x = [x,y,w,...,xi,yi,...]T
            u = [ox,oy,ow]T
        """
        pass


    # input: posteriori estimated state at timestamp k, system input u
    # output: Jacobian matrix of f about x_k and n_k: Jf(x_k), Jf(n_k)
    def jacob_motion(self, x, u):
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        pass

    # Calculate the innovation S = H*P*H' + R
    def calc_innovation(self, lm, xEst, PEst, z, LMid):
        pass

    # Calculate the measurement Jacobian: H
    def jacob_h(self, q, delta, x, i):
        pass

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
