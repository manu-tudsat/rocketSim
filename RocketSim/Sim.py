# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 13:27:57 2020
Modified on Tue Feb 22 2022

@author: Sascha
@author: Manu
"""

from Models import Models
import numpy as np

class PhysicsEngine():
    #%% EoM
    @staticmethod
    def EquationsOfMotion(x, rocket, atmosphere, time):
        '''Derives equations of motion and returns rates of change of state-vector'''
        
        #What is this doing?
        rocket.altitude = x[0]
        rocket.velocity = x[1]
        
        rocket.engine.calculate_thrust(rocket, atmosphere)
        Models.drag(rocket, atmosphere, time, False)
        gravity_force = -rocket.mass()*9.80665
        force = rocket.engine.thrust + rocket.drag + gravity_force
        rocket.acceleration = force/rocket.mass()
        
        x_p = np.array([rocket.velocity, rocket.acceleration])
        
        return x_p
        
    #%% Solver
    @staticmethod
    def Solve(x  ,rocket, atmosphere, time, method, timestep):
        #Supports all methods, but Euler_exp is fastest and therefore default
        dt = timestep
        #x = np.array([rocket.altitude, rocket.velocity])
        
        
        if method == 'Euler_exp':
            x_p = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, time)
            x_new = x + (x_p*dt)
        elif method == 'Heun':
            x_p_predictor = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, time) 
            x_predictor = x + (x_p_predictor*dt)
            x_p_corrector = PhysicsEngine.EquationsOfMotion(x_predictor, rocket, atmosphere, time+dt)
            x_new = x + (((x_p_predictor + x_p_corrector)/2)*dt)
        elif method == 'Euler_imp':
            dt = 1/10000
            x_exp = x + dt*PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, time)
            x_ini = x + dt*PhysicsEngine.EquationsOfMotion(x_exp, rocket, atmosphere, time+dt)
            
            x_prev = x_ini
            TOL = 1
            while TOL >= 1e-12:
                x_iter = x + dt*PhysicsEngine.EquationsOfMotion(x_prev, rocket, atmosphere, time+dt)
                TOL = max(abs(x_iter - x_prev))
                # print(TOL)
                x_prev = x_iter
            x_new = x_iter
        else:
            print('Solver method not available')
        
        # [rocket.altitude, rocket.velocity] = x_new
        
        rocket.engine.update_properties(rocket, atmosphere, dt)
        return x_new
    
    @staticmethod
    def Solve_adaptive(x, rocket, atmosphere, time, timestep, method):
        #currently unsupported
        # x = np.array([rocket.altitude, rocket.velocity])
        h = timestep
        t = time
        h_max = 1/1
        h_min = 1/10000
        err_allowed = 1e-12
        check = False
        test = 0
        if method == 'RKF':
            '''Runge-Kutta-Fehlberg method. '''
            while check == False:
            
                f_0 = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, t)
                f_1 = PhysicsEngine.EquationsOfMotion((x + ((h/4)*f_0)),rocket, atmosphere, t + (h/4))
                f_2 = PhysicsEngine.EquationsOfMotion((x + (((3*h)/32)*f_0) + (((9*h)/32)*f_1)),rocket, atmosphere, t + (3*h)/8)
                f_3 = PhysicsEngine.EquationsOfMotion((x + (((1932*h)/2197)*f_0) - (((7200*h)/2197)*f_1) + ((7296*h)/2197)*f_2),rocket, atmosphere, t + (12*h)/13)
                f_4 = PhysicsEngine.EquationsOfMotion((x + ((439*h)/216)*f_0 - (8*h)*f_1 + ((3680*h)/513)*f_2 - ((845*h)/4104)*f_3),rocket, atmosphere, t + h)
                f_5 = PhysicsEngine.EquationsOfMotion((x - ((8*h)/27)*f_0 + (2*h)*f_1 - ((3544*h)/2565)*f_2 + ((1859*h)/4104)*f_3 - ((11*h)/40)*f_4),rocket, atmosphere, t + h/2)
                
                approx_4th_order = x + h*((25/216)*f_0 + (1408/2565)*f_2 + (2197/4104)*f_3 - (1/5)*f_4)
                approx_5th_order = x + h*((16/135)*f_0 + (6656/12825)*f_2 + (28561/56430)*f_3 - (9/50)*f_4 + (2/55)*f_5)
                
                approx_err_vec = approx_4th_order - approx_5th_order
                approx_err = max(approx_err_vec)
                
                if approx_err == 0:
                    h_ideal = h_max
                else:
                    h_ideal = h*((abs(approx_err)/err_allowed)**(-(1/5)))
                
                if approx_err > err_allowed: # repeat with decrease timestep to get error-staisfying solution
                    if h_ideal < h_min:
                        h = h_min
                    else:
                        h = h_ideal
                else:
                    check = True
                    
                    if h_ideal > h_max:
                        h_ideal = h_max
                    
            x_new = approx_4th_order
            h_used = h
            spacer = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, t)
                
            PhysicsEngine.Update_properties(rocket, h)
        elif method == 'RKDP':
            '''Dormand-Prince(DOPRI) method. '''
            while check == False:
                
                f_0 = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, t)
                f_1 = PhysicsEngine.EquationsOfMotion((x + ((h/5)*f_0)),rocket, atmosphere, t + (h/5))
                f_2 = PhysicsEngine.EquationsOfMotion((x + (((3*h)/40)*f_0) + (((9*h)/40)*f_1)),rocket, atmosphere, t + (3*h)/10)
                f_3 = PhysicsEngine.EquationsOfMotion((x + (((44*h)/45)*f_0) - (((56*h)/15)*f_1) + ((32*h)/9)*f_2),rocket, atmosphere, t + (4*h)/5)
                f_4 = PhysicsEngine.EquationsOfMotion((x + ((19372*h)/6561)*f_0 - (((25360*h)/2187)*f_1) + ((64448*h)/6561)*f_2 - ((212*h)/729)*f_3),rocket, atmosphere, t + ((8*h)/9))
                f_5 = PhysicsEngine.EquationsOfMotion((x + ((9017*h)/3168)*f_0 - (((355*h)/33)*f_1) + ((46732*h)/5247)*f_2 + ((49*h)/176)*f_3 - ((5103*h)/18656)*f_4),rocket, atmosphere, t + h)
                f_6 = PhysicsEngine.EquationsOfMotion((x + ((35*h)/384)*f_0 + (0*f_1) + ((500*h)/1113)*f_2 + ((125*h)/192)*f_3 - ((2187*h)/6784)*f_4) + (((11*h)/84)*f_5),rocket, atmosphere, t + h)                
                
                approx_5th_order = x + ((35*h)/384)*f_0 + (0*h*f_1) + ((500*h)/1113)*f_2 + ((125*h)/192)*f_3 - ((2187*h)/6784)*f_4 + (((11*h)/84)*f_5)
                alternative_solution = x + h*((5179/57600)*f_0 + (0*f_1) + (7571/16695)*f_2 + (393/640)*f_3 - (92097/339200)*f_4 + (187/2100)*f_5 + ((1/40)*f_6))
                
                approx_err_vec = approx_5th_order - alternative_solution
                approx_err = np.linalg.norm(approx_err_vec)
                # print(approx_err)
                if approx_err == 0:
                    h_ideal = h_max
                else:
                    h_ideal = h*((abs(approx_err)/err_allowed)**(-(1/5)))
                
                if approx_err > err_allowed: # repeat with decrease timestep to get error-staisfying solution
                    h = h/2
                    if h < h_min:
                        h = h_min
                        test += 1
                        if test > 3:
                            approx_5th_order = PhysicsEngine.Solve(x, rocket, atmosphere, time, 'Euler_imp')
                            check = True
                            # print('implicit @ ' + str(time))
                    # if h_ideal < h_min:
                    #     h = h_min
                    # else:
                    #     h = h_ideal
                else:
                    check = True
                    if h_ideal > h_max:
                        h_ideal = h_max
                    
            x_new = approx_5th_order
            h_used = h
            spacer = PhysicsEngine.EquationsOfMotion(x, rocket, atmosphere, t)
                
            #PhysicsEngine.Update_properties(rocket, h)
            rocket.engine.update_properties(rocket, atmosphere, h)
        else:
            print('Adaptive solver method not available')
        return x_new, h_ideal, h_used, approx_err
            
    #%% Update properties
    @staticmethod
    def Update_properties(rocket, dt):
        #rocket.mass_propellant_old = rocket.mass_propellant
        if rocket.pressurized == True:
            rocket.pressurant_mass = rocket.pressurant_mass - (dt*rocket.pressurant_massflow)
        rocket.mass_propellant = rocket.mass_propellant - (dt*rocket.massflow_propellant)
        if rocket.mass_propellant < 0:
            rocket.mass_propellant = 0
    
    #%% new Solver
    @staticmethod
    def Step(rocket, atmosphere, time, timestep):
        rocket.engine.calculate_thrust(rocket, atmosphere)
        Models.drag(rocket, atmosphere, time, False)
        gravity_force = -rocket.mass()*9.80665
        force = rocket.engine.thrust + rocket.drag + gravity_force
        
        rocket.acceleration = force/rocket.mass()
        rocket.velocity += rocket.acceleration * timestep
        rocket.altitude += rocket.velocity * timestep
        
        rocket.engine.update_properties(rocket, atmosphere, timestep)
    
    
    #%% Save properties
    # @staticmethod
    # def Save_properties(rocket):
        