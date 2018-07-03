import numpy as np
import numpy.linalg as npl
from numpy import sin, cos, tan
from math import pi

from scipy.integrate import odeint
import matplotlib.pyplot as plt

import PD



class Quadrotor(object):
    """docstring for Quadrotor."""
    def __init__(self, mass=0.5, MomentOfInertiaTotal=np.array([[3.2e-3,0.0,0.0],[0.0,3.2e-3,0.0],[0.0,0.0,5.5e-5]]), MomentOfInertiaProp=np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,1.5e-5]])):
        super(Quadrotor, self).__init__()
        self.m = mass
        self.ITotal = MomentOfInertiaTotal
        self.IProp  = MomentOfInertiaProp
        self.IBody  = MomentOfInertiaTotal - 4 * MomentOfInertiaProp
        self.IBodyT = np.transpose(self.IBody)
        self.PQR    = np.zeros(3)            # Angular Velocity [rad/s]
        self.PQRDot = np.zeros(3)            # Angular Acceleration[rad/s^2]
        self.YPR    = np.zeros(3)            # Euler Angle [rad]
        self.YPRDot = np.zeros(3)            # Euler Angular Velocity [rad/s]
        self.R      = np.zeros((3,3))        # TODO Rotation Matrix X_E = R X_B
        self.RDot   = np.zeros((3,3))        # TODO Derivative of Rotation Matrix
        self.Q      = np.zeros(4)            # TODO Quartanion
        self.QDot   = np.zeros(4)            # TODO Derivative of Quartanion
        self.M      = np.zeros(3)            # Moments [Nm]
        self.F      = np.zeros(3)            # TODO Force [N]
        self.PQRProp  = np.zeros((3,4))      # TODO Rotation of Propellers, in body frame
        self.CTRL   = PD.Controller(omega=5, zeta=0.5) # Controller
        self.POSCTRL = PD.Controller(omega=1.0, zeta = 0.7) # Position Controller
        # self.ALLC   =                      # TODO Allocator
        self.kt     = 1.69e-2                # coefficient from the thrust to the reaction torque
        self.arm    = 0.17                   # Moment arms between GoM and each propellers
        self.normality  = np.ones(4)         # fault degree of the each propellers: zero -> completely fault

        self.gamma  = 2.75e-3                # rotational drag

        self.XYZ    = np.zeros(3)            # Position [m]
        self.UVW    = np.zeros(3)            # Velocity [m/s]
        self.UVWDot = np.zeros(3)            # Acceleration [m/s^2]

    def calcPQRDot(self, moment):
        M = moment

        Iwpdot = np.zeros(3)
        Iwb = np.dot(self.IBody,self.PQR)
        Iwp = np.dot(self.IProp,(4 * self.PQR + np.sum(self.PQRProp[2,:])))
        gyro = np.cross(self.PQR,Iwb+Iwp)
        drag = np.array([0,0,-self.gamma]) * self.PQR

        PQRDot = np.dot(self.IBodyT, moment - Iwpdot + gyro + drag)
        return PQRDot

    def calcEulerDot(self, PQR, YPR):
        p = PQR[0]
        q = PQR[1]
        r = PQR[2]
        phi = YPR[0]
        the = YPR[1]
        psi = YPR[2]

        phiDot = p + q * sin(phi) * tan(the) + r * cos(phi) * tan(the)
        theDot = q * cos(phi) - r * sin(phi)
        psiDot = q * sin(phi) / cos(the) + r * cos(phi) / cos(the)

        YPRDot = np.array([phiDot, theDot, psiDot])
        self.YPRDot = YPRDot
        return YPRDot

    def calcRDot(R, pqr):
        hat = np.array([
        [      0, -pqr[2],  pqr[1]],
        [ pqr[2],       0, -pqr[0]],
        [-pqr[1],  pqr[0],       0]
        ])

        RDot = np.dot(R, hat)
        self.RDot = RDot
        return RDot

    def calcInputFrom(self, moment, totalForce):
        kt = self.kt
        d  = self.arm
        ktd = kt * d
        kt2 = 2 * kt

        FM = np.hstack((totalForce, moment))

        input = np.dot(np.array([
        [-ktd, 0, kt2, d],
        [-ktd, kt2, 0, -d],
        [-ktd, 0, -kt2, d],
        [-ktd, -kt2, 0, -d]
        ]), FM) / 4/ ktd

        return input

    def calcFMFrom(self, input):
        kt = self.kt
        d  = self.arm

        FM = np.dot(np.array([
        [-1,-1,-1,-1],
        [0,d,0,-d],
        [d,0,-d,0],
        [kt,-kt,kt,-kt]
        ]),input)

        return FM[0], FM[1:4]

    def calcReducedInputFrom(self, moment, totalForce):
        d = self.arm

        FM = np.hstack((totalForce,moment[0:2]))

        input = np.dot(np.array([
        [-d,-1, 1],
        [ 0, 2, 0],
        [-d,-1,-1]
        ]), FM) / 2/ d

        input = np.hstack((input,0))

        return input

    def responseConsidered(self, input, display=False):
        output = input * self.normality
        if display:
            print(f"Input: {output}")
        return output

    def RFrom(self, EulerAngles):
        phi = EulerAngles[0]
        the = EulerAngles[1]
        psi = EulerAngles[2]

        RPhi = np.array([
        [1, 0, 0],
        [0, cos(phi), -sin(phi)],
        [0, sin(phi),  cos(phi)]
        ])

        RTheta = np.array([
        [cos(the), 0, sin(the)],
        [0, 1, 0],
        [-sin(the), 0, cos(the)]
        ])

        RPsi = np.array([
        [cos(psi), -sin(psi), 0],
        [sin(psi),  cos(psi), 0],
        [0, 0, 1]
        ])

        return np.dot(RPhi, np.dot(RTheta,RPsi))

    def calcUVWDot(self, force):
        R = self.R
        D = np.array([0,0,9.81 * self.m]) # TODO:
        self.UVWDot = np.dot(R,np.array([0,0,force])) / self.m + D / self.m
        # print(self.UVWDot)
        return self.UVWDot

    def fEuler(self, x, t):
        self.preprocess(x,t)

        YPR = x[0:3]
        PQR = x[3:6]

        moment = x[6:9] + np.dot(npl.inv(self.IBody),self.CTRL.getMReq(YPR, PQR))
        force = 0.5 * 9.81
        # input = self.calcInputFrom(moment, force)
        input = self.calcReducedInputFrom(moment, force)
        f, moment = self.calcFMFrom( self.responseConsidered( input, display=False))
        # print(f'F: {f}, M: {moment}')

        return np.hstack((self.calcEulerDot(PQR, YPR), self.calcPQRDot(moment), np.zeros(3)))


    def fPosition(self, x, t):
        self.preprocess(x,t)

        YPR = x[0:3]
        PQR = x[3:6]
        x   = x[9:12]
        xDot = x[12:15]

        moment = np.dot(npl.inv(self.IBody),self.CTRL.getMReq(YPR, PQR))
        force = -0.5 * 9.81
        # input = self.calcInputFrom(moment, force)
        input = self.calcReducedInputFrom(moment, force)

        # f, moment = self.calcFMFrom(input)
        f, moment = self.calcFMFrom( self.responseConsidered( input, display=False))
        # print(f'F: {f}, M: {moment}')
        # print(self.calcEulerDot(PQR, YPR), self.calcPQRDot(moment), np.zeros(3),self.UVW, self.calcUVWDot(f), np.zeros(3))
        return np.hstack((self.calcEulerDot(PQR, YPR), self.calcPQRDot(moment), np.zeros(3),self.UVW, self.calcUVWDot(f), np.zeros(3)))

    def preprocess(self, x, t):
        # print(x[9:18])
        self.YPR = x[0:3]
        self.PQR = x[3:6]
        self.R   = self.RFrom(self.YPR)
        if np.size(x) >7:
            self.XYZ = x[9:12]
            self.UVW = x[12:15]
        pass

if __name__ == '__main__':
    uav = Quadrotor()
    uav.normality = np.array([1,1,1,0])

    x0 = np.zeros(18)
    # x0[0] = pi / 6
    x0[1] = -pi / 6
    # x0[2] = pi/12
    t = np.arange(0,30.0,0.001)
    # x = odeint(uav.fEuler, x0, t)
    x = odeint(uav.fPosition, x0, t)

    fig = plt.figure()
    # plt.plot(t,x[:,0],label='Roll  [rad]')
    # plt.plot(t,x[:,1],label='Pitch [rad]')
    # plt.plot(t,x[:,2],label='Yaw   [rad]')

    # plt.plot(t,x[:,3],label='P [rad/s]')
    # plt.plot(t,x[:,4],label='Q [rad/s]')
    # plt.plot(t,x[:,5],label='R [rad/s]')

    # plt.plot(t,x[:, 9],label='X [m]')
    # plt.plot(t,x[:,10],label='Y [m]')
    # plt.plot(t,x[:,11],label='Z [m]')

    plt.plot(t,x[:,12],label='U [m/s]')
    plt.plot(t,x[:,13],label='V [m/s]')
    plt.plot(t,x[:,14],label='W [m/s]')

    # plt.plot(t,x[:,15],label='$\dot{U}[m/s^2]$ ')
    # plt.plot(t,x[:,16],label='$\dot{V}[m/s^2]$ ')
    # plt.plot(t,x[:,17],label='$\dot{W}[m/s^2]$ ')

    plt.legend()
    plt.grid()
    plt.show()

    print(x[:,9:12])
