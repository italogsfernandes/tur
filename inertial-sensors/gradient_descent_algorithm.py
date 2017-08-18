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
# minimal rotation from a vector to a reference.
#------------------------------------------------------------------------------
import numpy as np
#------------------------------------------------------------------------------
class gradient_descent_algorithm():
    def __init__(self):
        print("Started")

    def myMethod(self):
        print("Yeah")

#------------------------------------------------------------------------------
#Tests
if __name__ == "__main__":
    gda = gradient_descent_algorithm()
    while(True):
        print('-------------------------------')
        print('Menu')
        print('1 - myMethod')
        print('Q - Quit')
        print('-------------------------------')
        strkey = raw_input()
        if strkey == '1':
            gda.myMethod()
        elif strkey == 'Q':
            break
#------------------------------------------------------------------------------
