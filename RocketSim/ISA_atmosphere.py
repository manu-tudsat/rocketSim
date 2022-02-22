# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:18:23 2020

@author: Sascha
"""

import math

class ISA_atmosphere():
    '''ISA standard atmosphere. Valid from 0m to 32000m.'''
    
    def __init__(self):
        self.g = 9.80665 # gravity
        self.kappa = 1.4 # isentropic exponent
    
    #%%
    def density(self,altitude):
        '''returns density[Kg/m³] at given altitude[m]'''
        if altitude <= 11000:
            dTdz = - 0.0065   # temperature gradient [K/m]
            rho_i = 1.225   # reference density
            T_i = 288.15    # reference temperature
            z_i = 0 # reference altitude
            Bereich = 1 
        elif altitude > 11000 and altitude <= 20000:
            dTdz = 0
            rho_i = 0.36392
            T_i = 216.65
            z_i = 11000
            Bereich = 2
        elif altitude > 20000 and altitude <= 32000:
            dTdz = 0.001
            rho_i = 0.088035
            T_i = 228.65
            z_i = 20000
            Bereich = 3
            
        R = 287.1#p_i/(rho_i*T_i) #gas constant
        
        n = (1+(dTdz*(R/self.g)))    # Polytropic exponent
        
        if Bereich == 2:
            rho = rho_i*(math.exp(-((self.g/(R*T_i))*(altitude-z_i))))
        else:
            rho = rho_i*((1-(((n-1)/n)*(self.g/(R*T_i))*(altitude-z_i)))**(n/(n-1)))
        
        return rho
    #%%
    def pressure(self,altitude):
        '''returns pressure[Kg/ms² = Pa] at given altitude[m]'''
        if altitude <= 11000:
            dTdz = - 0.0065   # temperature gradient [K/m]
            p_i = 101325    # reference pressure
            T_i = 288.15    # reference temperature
            z_i = 0 # reference altitude
            Bereich = 1 
        elif altitude > 11000 and altitude <= 20000:
            dTdz = 0
            p_i = 22632
            T_i = 216.65
            z_i = 11000
            Bereich = 2
        elif altitude > 20000 and altitude <= 32000:
            dTdz = 0.001
            p_i = 5474.88
            T_i = 228.65
            z_i = 20000
            Bereich = 3
            
        R = 287.1#p_i/(rho_i*T_i) #gas constant
        
        n = (1+(dTdz*(R/self.g)))    # Polytropic exponent
        
        if Bereich == 2:
            p = p_i*(math.exp(-((self.g/(R*T_i))*(altitude-z_i))))
        else:
            p = p_i*((1-(((n-1)/n)*(self.g/(R*T_i))*(altitude-z_i)))**(n/(n-1)))
        
        return p
    #%%
    def temperature(self,altitude):
        '''returns temperature[K] at given altitude[m]'''
        if altitude <= 11000:
            dTdz = - 0.0065   # temperature gradient [K/m]
            T_i = 288.15    # reference temperature
            Bereich = 1 
        elif altitude > 11000 and altitude <= 20000:
            dTdz = 0
            T_i = 216.65
            Bereich = 2
        elif altitude > 20000 and altitude <= 32000:
            dTdz = 0.001
            T_i = 228.65
            Bereich = 3
        
        if Bereich == 2:
            T = T_i
        else:
            T = T_i + (dTdz*altitude) 
                
        return T
    #%%
    def sonic_velocity(self,altitude):
        '''returns sonic velocity[m/s] at given altitude[m]'''    
        if altitude <= 11000:
            dTdz = - 0.0065   # temperature gradient [K/m]
            T_i = 288.15    # reference temperature
            Bereich = 1 
        elif altitude > 11000 and altitude <= 20000:
            dTdz = 0
            T_i = 216.65
            Bereich = 2
        elif altitude > 20000 and altitude <= 32000:
            dTdz = 0.001
            T_i = 228.65
            Bereich = 3
            
        R = 287.1#p_i/(rho_i*T_i) #gas constant
        
        if Bereich == 2:
            T = T_i
        else:
            T = T_i + (dTdz*altitude) 
        
        a = math.sqrt(self.kappa*R*T)    # sonic velocity
        
        return a