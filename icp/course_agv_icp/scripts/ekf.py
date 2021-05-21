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
Qy = np.diag([0.1, 0.1, np.deg2rad(5.0)]) ** 2

class EKF():
    def __init__(self):
        pass
    
    def fusion(self, xEst, PEst, z, u):
        xPred = self.odom_model(xEst, u)
        Jfx, _ = self.jacob_motion(xEst, u)
        PPred = (Jfx.dot(PEst)).dot(Jfx.T) + Qx
        # zPred = self.observ_model(xPred, z)
        # jH = np.eye(2)
        # y = np.array(z).T - zPred
        y = z
        S = Qy
        print(xPred)
        print(y)
        xEst = np.linalg.inv(PPred**2 + S**2).dot((S**2).dot(xPred) + (PPred**2).dot(y))
        PEst = (np.linalg.inv(PPred**2 + S**2)).dot(PPred**2).dot(S**2)
        return xEst, PEst

    # input: posteriori estimated state and its covariance at timestamp k, system input u
    # output: posteriori estimated state and its covariance at timestamp k+1
    def estimate(self, xEst, PEst, z, u):
        # Predict
        S = STATE_SIZE
        # motion model: odom_model
        xEst[0:S] = self.odom_model(xEst[0:S], u)
        # uncertainty propagation: jacob_motion
        Jfx, Jfn = self.jacob_motion(xEst, u)
        PEst = (Jfx.dot(PEst)).dot(Jfx.T) + (Jfn.dot(Qx)).dot(Jfn.T)
        # Update for each observation
        for iz in range(z.shape[0]):
            pass
        nLM = self.calc_lm_number(xEst)

        return xEst, PEst

    # input: posteriori estimated state at timestamp k, system input u
    # output: priori estimated state at timestamp k+1 
    def odom_model(self, x, u):
        """
            x = [x,y,w]T
            u = [ox,oy,ow]T
        """
        x[0,0] = x[0,0] + np.cos(x[2,0])*u[0,0] - np.sin(x[2,0])*u[1,0]
        x[1,0] = x[1,0] + np.sin(x[2,0])*u[0,0] + np.cos(x[2,0])*u[1,0]
        x[2,0] = x[2,0] + u[2,0]
        return x

    def observ_model(self, x, z):
        zp = np.zeros((2,1))
        zp[0,0] = x[0,0] + z[0] * np.cos(x[2,0] + z[1])
        zp[1,0] = x[1,0] + z[0] * np.sin(x[2,0] + z[1])
        return zp

    def calc_lm_number(self, x):
        return int((x.shape[0]-STATE_SIZE) / LM_SIZE)

    # input: posteriori estimated state at timestamp k, system input u
    # output: Jacobian matrix of f about x_k and n_k: Jf(x_k), Jf(n_k)
    def jacob_motion(self, x, u):
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        Jfx = np.eye(x.shape[0])
        Jfx[0,2] = -np.sin(x[2,0])*u[0,0] - np.cos(x[2,0])*u[1,0]
        Jfx[1,2] = np.cos(x[2,0])*u[0,0] - np.sin(x[2,0])*u[1,0]
        Jfn = np.zeros((x.shape[0],3))
        Jfn[0:STATE_SIZE] = np.eye(STATE_SIZE)
        return Jfx, Jfn

    # Calculate the innovation S = H*P*H' + R
    def calc_innovation(self, lm, xEst, PEst, z, lm_id):
        delta = lm - xEst[0:2]
        pass

    # Calculate the measurement Jacobian: H
    def jacob_h(self, q, delta, x, i):
        pass

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
