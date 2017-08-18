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
# Decription: Implementation of a gradient_descent_algorithm for finding the
# minimal rotation from an accel vector to a gravity reference.
#------------------------------------------------------------------------------
import numpy as np
#------------------------------------------------------------------------------
class accel_ahrs():
    def __init__(self):
        print("Started")

    def myMethod(self):
        print("Yeah")

    def gradientF(self,seQk, eD, sS):
        return self.jacob(seQk)*self.objFunction(seQk,eD,sS)

    def objFunction(self,q, a):
        f = np.zeros((3,1))
        # TODO: rewrite and verify
        f[0][0] = 2*(      q[1]*q[3] - q[0]*q[2]) - a[0]
        f[0][1] = 2*(      q[0]*q[1] + q[2]*q[3]) - a[1]
        f[0][2] = 2*(0.5 - q[1]*q[1] - q[2]*q[2]) - a[2]
        return f

    def jacob(self,q):
        j = np.zeros((3,3))
        # TODO: rewrite and verify
        j[0] = [ -2*q[2],   2*q[3],     -2*q[0],    2*q[1] ]
        j[1] = [ 2*q[1],    2*q[0],      2*q[3],    2*q[2] ]
        j[2] = [ 0,        -4*q[1],     -4*q[2],    0      ]
        return j

#------------------------------------------------------------------------------
#Tests
if __name__ == "__main__":
    seQk = np.array([1.0, 0.0, 0.0, 0.0])   #Quaternion Atual
    eG = np.array([0.0, 0.0, 0.0, 1.0])     #Earth Gravity
    sA = np.array([0.0, 0.0, 0.0, 1.0])     #Sensor Accel
    u = ?   #gradient_descent_algorithm Step

    threaxis = accel_ahrs()
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
            threaxis.myMethod()
        elif strkey == '2':
            threaxis.jacob([1,0,0,0])
        elif strkey == '3':
            threaxis.objFunction([1,0,0,0],[0,0,1])
        elif strkey == 'Q':
            break

#------------------------------------------------------------------------------
