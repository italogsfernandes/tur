# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANDIA
# Faculty of Electrical Engineering
# Biomedical Engineering Lab
#------------------------------------------------------------------------------
# Author: Italo Fernandes
# Contact: italogsfernandes@gmail.com
# URL: www.biolab.eletrica.ufu.br
# Git: www.github.com/italogfernandes
#------------------------------------------------------------------------------
# Decription: Sersor fusion with ideal accel and ideal mag
#------------------------------------------------------------------------------
import numpy as np
from mag_ahrs import mag_ahrs
from accel_ahrs import accel_ahrs
#------------------------------------------------------------------------------
class accel_mag_ahrs():
    def __init__(self):
        self.mag_sensor = mag_ahrs()
        self.accel_sensor = accel_ahrs()

    def myMethod(self):
        print("Yeah")

    def gradientF(self,seQk, eD, sS):
        return self.jacob(seQk)*self.objFunction(seQk,eD,sS)

    def objFunction(self,q, b, m):
        f =  [[accel_sensor.objFunction],[mag_sensor.objFunction]]
        '''
        f[0][0] = 2*b[0]*(0.5 - q[2]*q[2]  - q[3]*q[3]) + 2*b[2]*(      q[1]*q[3] - q[0]*q[2]) - m[0]
        f[0][1] = 2*b[0]*(      q[1]*q[2]  - q[0]*q[3]) + 2*b[2]*(      q[0]*q[1] + q[2]*q[3]) - m[1]
        f[0][2] = 2*b[0]*(      q[0]*q[2]  + q[1]*q[3]) + 2*b[2]*(0.5 - q[1]*q[1] - q[2]*q[2]) - m[2]
        '''

        '''
        f[0][0] = 2*bx*(0.5 - q3*q3 - q4*q4) + 2*bz*(q2*q4 - q1*q3) - mx
        f[0][1] = 2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4) - my
        f[0][2] = 2*bx*(q1*q3 + q2*q4) + 2*bz*(0.5 - q2*q2 - q3*q3) - mz
        '''

        return f

    def jacob(self,q):
        f =  [[accel_sensor.jabob],[mag_sensor.jabob]]
        '''
        j[0][0] = -2*bz*q3
        j[0][1] = 2*bz*q4
        j[0][2] = -4*bx*q3 - 2*bz*q1
        j[0][3] = -4*bx*q4 + 2*bz*q2

        j[1][0] = -2*bx*q4 + 2*bz*q2
        j[1][1] = 2*bx*q3 + 2*bz*q1
        j[1][2] = 2*bx*q2 + 2*bz*q4
        j[1][3] = -2*bx*q1 + 2*bz*q3

        j[2][0] = 2*bx*q3
        j[2][1] = 2*bx*q4 - 4*bz*q2
        j[2][2] = 2*bx*q1 - 4*bz*q3
        j[2][3] = 2*bx*q2
        '''

        return j

#------------------------------------------------------------------------------
#Tests
if __name__ == "__main__":
    seQk = np.array([1.0, 0.0, 0.0, 0.0])   #Quaternion Atual
    eB = np.array([0.0, 0.0, 0.0, 1.0])     #Earth Noth Direction
    sM = np.array([0.0, 0.0, 0.0, 1.0])     #Sensor Magnetic Values
    u = ?   #gradient_descent_algorithm Step

    magaxis = accel_ahrs()
    while(True):
        print('-------------------------------')
        print('Menu')
        print('1 - myMethod')
        print('2 - jacob([1, 0, 0, 0])')
        print('3 - objFunction([1, 0, 0, 0], [0, 0, 1)')
        print('Q - Quit')
        print('-------------------------------')
        strkey = raw_input()
        if strkey == '1':
            magaxis.myMethod()
        elif strkey == '2':
            magaxis.jacob([1,0,0,0])
        elif strkey == '3':
            magaxis.objFunction([1,0,0,0],[0,0,1])
        elif strkey == 'Q':
            break

#------------------------------------------------------------------------------
