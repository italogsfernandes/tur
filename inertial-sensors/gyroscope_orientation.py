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
# Decription: Calculates a quaternion with ideal gyroscope data
#------------------------------------------------------------------------------
import numpy as np
import quaternion as quat
#------------------------------------------------------------------------------
class gyroscope_orientation():
    def __init__(self):
        print("Started")
        self.qest = np.array([1,0,0,0])
        self.wt = np.array([0.0,0.0,0.0,np.pi])
        self.delta_t = 0.5

    def myMethod(self):
        print("Yeah")

    def calcQwt(self, qest,wt,delta_t):
        print self.calcqqwt(qest,wt)
        q1 = qest
        cq1= quat.conjugate(q1)
        q2 = self.calcqqwt(qest,wt)*delta_t
        cq2 = quat.conjugate(q2)
        print q1+q2
        return 2

    def calcqqwt(self, qest,wt):
        return quat.product(0.5*qest,wt)


#------------------------------------------------------------------------------
#Tests
if __name__ == "__main__":
    gyro_ahrs = gyroscope_orientation()
    while(True):
        print('-------------------------------')
        print('Menu')
        print('1 - myMethod')
        print('2 - calcQwt')
        print('3 - calcqqwt')
        print('q - Quit')
        print('-------------------------------')
        strkey = raw_input()
        if strkey == 'q':
            break
        elif strkey == '1':
            gyro_ahrs.myMethod()
        elif strkey == '2':
            print gyro_ahrs.calcQwt(gyro_ahrs.qest, gyro_ahrs.wt, gyro_ahrs.delta_t)
        elif strkey == '3':
            print gyro_ahrs.calcqqwt(gyro_ahrs.qest,gyro_ahrs.wt)

#------------------------------------------------------------------------------
