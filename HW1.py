# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
import sympy as sym
import math

R11 = 0.5
R23 = -0.5
R33 = 0.5
R12,R13,R21,R22,R31,R32 = sym.symbols('R_12 R_13 R_21 R_22 R_31 R_32')
rotation_Matrix = np.array([[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]])
horizontal_Eq0 = sym.Eq(np.dot(rotation_Matrix[0],rotation_Matrix[0]),1)
horizontal_Eq1 = sym.Eq(np.dot(rotation_Matrix[1],rotation_Matrix[1]),1)
horizontal_Eq2 = sym.Eq(np.dot(rotation_Matrix[2],rotation_Matrix[2]),1)
horizontal_Eq3 = sym.Eq(np.dot(rotation_Matrix[0],rotation_Matrix[1]),0)
horizontal_Eq4 = sym.Eq(np.dot(rotation_Matrix[0],rotation_Matrix[2]),0)
horizontal_Eq5 = sym.Eq(np.dot(rotation_Matrix[1],rotation_Matrix[2]),0)
vertical_Eq0 = sym.Eq(np.dot(rotation_Matrix[:,0],rotation_Matrix[:,0]),1)
vertical_Eq1 = sym.Eq(np.dot(rotation_Matrix[:,1],rotation_Matrix[:,1]),1)
vertical_Eq2 = sym.Eq(np.dot(rotation_Matrix[:,2],rotation_Matrix[:,2]),1)
vertical_Eq3 = sym.Eq(np.dot(rotation_Matrix[:,0],rotation_Matrix[:,1]),0)
vertical_Eq4 = sym.Eq(np.dot(rotation_Matrix[:,0],rotation_Matrix[:,2]),0)
vertical_Eq5 = sym.Eq(np.dot(rotation_Matrix[:,1],rotation_Matrix[:,2]),0)

solution_Set = sym.solve([horizontal_Eq0,horizontal_Eq1,horizontal_Eq2,horizontal_Eq3,horizontal_Eq4,horizontal_Eq5,vertical_Eq0,vertical_Eq1,vertical_Eq2,vertical_Eq3,vertical_Eq4,vertical_Eq5],[R12,R13,R21,R22,R31,R32])

for i in range (len(solution_Set)):
    R12,R13,R21,R22,R31,R32 = solution_Set[i]
    rotation_Matrix = np.array([[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]])
    M = sym.Matrix(rotation_Matrix)
    if abs(M.det()-1) <0.001:
        print(M)
    
    