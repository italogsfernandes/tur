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
# Decription: 
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
class classname():
    def __init__(self):
        print("Started")

    def myMethod(self):
        print("Yeah")

#------------------------------------------------------------------------------
#Tests
if __name__ == "__main__":
    objectname = classname()
    while(True):
        print('-------------------------------')
        print('Menu')
        print('1 - myMethod')
        print('Q - Quit')
        print('-------------------------------')
        strkey = raw_input()
        if strkey == '1':
            gyro_ahrs.myMethod()
        elif strkey == 'Q':
            break
#------------------------------------------------------------------------------
