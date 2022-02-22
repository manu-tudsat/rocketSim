# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 15:06:49 2020
Modified on Sat Feb 12 2022

@author: Sascha
@author: Manu
"""

import math
import numpy as np
from CoolProp.CoolProp import PropsSI

class Models():
    
    #%% Thrust (unused, old)
    def Thrust(rocket, atmosphere):
        
        if rocket.engine == 'NOCTUA':
            v_e = 1977.6112646470153 # exhaust velocity
            rocket.massflow_propellant =  1.019295722556449
            A_e = 0.0023453342157077834 # exhaust area
            p_e = 1013.25*1e2
            if rocket.mass_propellant <= 0:
                rocket.mass_propellant = 0
                rocket.massflow_propellant = 0
                rocket.thrust = 0
                thrust = 0
            else:
                thrust = rocket.massflow_propellant*v_e + A_e*(p_e - atmosphere.pressure(rocket.altitude))
                rocket.thrust = thrust
            
            
        elif rocket.engine == 'water-blowdown':
            
            A_e = 0.021**2 * math.pi / 4 # exhaust area
            V_gas_initial = rocket.tank_vol() - (rocket.mass_propellant_initial / 1000)#rocket.rho_propellant)
            #print('rocket.tank_vol() ' + str(rocket.tank_vol()))
            p_initial = 6*1e5
            kappa_air = 1.4
            loss = 0.9
            length_launchtube = 1
            diameter_launchtube = 0.012
            area_launchtube = diameter_launchtube**2 * math.pi / 4
            F_stat = 0 # static pressure on launch tube area
            m_p_lt = 0 # massflow through launch tube
            temp_r = 288.15 # temperature in Kelvin
            R_air = 287.1 # gas constant
            air_mass_initial = p_initial * V_gas_initial / (R_air * temp_r) #ideal gas law
            rho_air_initial = air_mass_initial/V_gas_initial
            #rocket.air_runout = False
            
            
            if rocket.air_runout == True: # see air thrust computation
                rocket.mass_propellant = 0
                rocket.massflow_propellant = 0
                rocket.thrust = 0
                thrust = 0
            else:
                if rocket.prop_vol() > 0 and rocket.water_runout == False: # water thrust
                    rocket.tank_pressure = p_initial/((rocket.air_vol()/V_gas_initial)**kappa_air)
                    dp = rocket.tank_pressure - atmosphere.pressure(rocket.altitude)
                    if dp < 0:
                        dp = 0
                    if rocket.altitude <= length_launchtube:
                        F_stat = area_launchtube * rocket.tank_pressure
                        A_e = A_e - area_launchtube #while on launchtube exit area is obstructed
                        V_p_lt = rocket.velocity * area_launchtube
                        m_p_lt = V_p_lt*rocket.rho_propellant
                    v_e = loss * math.sqrt((2 * dp) / rocket.rho_propellant)
                    rocket.massflow_propellant = ((v_e * A_e) * rocket.rho_propellant) + m_p_lt
                    thrust = F_stat + ((rocket.massflow_propellant-m_p_lt) * v_e) + 0*(A_e * dp) + (m_p_lt*rocket.velocity)
                    rocket.thrust = thrust
                else: # air thrust
                    if rocket.water_runout == False: # propellant mass is virtually refilled by air mass as water propellant runs out
                        rocket.mass_propellant = air_mass_initial
                        #rocket.mass_propellant_old = rocket.mass_propellant
                        rocket.tank_pressure = air_mass_initial / rocket.tank_vol() * R_air * temp_r
                        rocket.p_initial_air = rocket.tank_pressure
                        rocket.water_runout = True
                    rocket.rho_propellant = rocket.tank_pressure / (R_air * temp_r)  #ideal gas law
                    rocket.tank_pressure = rocket.p_initial_air / (rho_air_initial / (rocket.mass_propellant/rocket.tank_vol()))**kappa_air
                    dp = rocket.tank_pressure - atmosphere.pressure(rocket.altitude)
                    if dp < 0:
                        dp = 0
                        rocket.air_runout = True
                    v_e = loss * math.sqrt((2 * dp) / rocket.rho_propellant)
                    
                    rocket.massflow_propellant = v_e * A_e * rocket.rho_propellant
                    thrust = rocket.massflow_propellant * v_e + A_e * dp
                    rocket.thrust = thrust
                
        elif rocket.engine == 'paintball':
            loss = 0.9
            A_nozzle = 0.021**2 * math.pi / 4
            p_ambient = atmosphere.pressure(rocket.altitude)
            T = 288
            
            if rocket.mass_propellant > 0:
                dp = rocket.maintank_pressure - p_ambient
                v_e = loss * math.sqrt((2 * dp) / rocket.rho_propellant)
                rocket.massflow_propellant = v_e*A_nozzle*rocket.rho_propellant
                thrust = rocket.massflow_propellant*v_e
                rocket.thrust = thrust
                
                maintank_gas_volume = rocket.tank_vol() - rocket.prop_vol()
                maintank_gas_density = PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant)
                maintank_gas_mass = maintank_gas_volume*maintank_gas_density
                rocket.pressurant_mass_pt = rocket.pressurant_mass_initial - maintank_gas_mass
                #negative mass fix
                if rocket.pressurant_mass_pt < 0:
                    rocket.pressurant_mass_pt =1e-10
                #end negative mass fix
                pressurant_density = rocket.pressurant_mass_pt/rocket.pressurant_volume
                rocket.pressurant_pressure = PropsSI('P','T',T,'D',pressurant_density, rocket.pressurant)
                rocket.pressurant_massflow = 0
            elif rocket.mass_propellant <= 0 and rocket.pressurant_pressure > rocket.maintank_pressure:
                dp = rocket.maintank_pressure - p_ambient
                v_e = loss * math.sqrt((2 * dp) / PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant))
                # print(v_e)
                maintank_gas_volume = rocket.tank_vol() - rocket.prop_vol()
                maintank_gas_density = PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant)
                maintank_gas_mass = maintank_gas_volume*maintank_gas_density
                rocket.pressurant_mass_pt = rocket.pressurant_mass - maintank_gas_mass
                pressurant_density = rocket.pressurant_mass_pt/rocket.pressurant_volume
                rocket.pressurant_pressure = PropsSI('P','T',T,'D',pressurant_density, rocket.pressurant)
                
                rocket.pressurant_massflow = v_e*A_nozzle*PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant)
                thrust = rocket.pressurant_massflow*v_e
                rocket.thrust = thrust
                rocket.massflow_propellant = 0
            elif rocket.mass_propellant <= 0 and rocket.pressurant_mass > 1*(rocket.tank_vol() + rocket.pressurant_volume)*PropsSI('D','T',T,'P',atmosphere.pressure(0), rocket.pressurant):
                dp = rocket.maintank_pressure - p_ambient
                if dp < 0:
                    dp = 0
                # print(rocket.maintank_pressure)
                gas_volume = rocket.tank_vol() + rocket.pressurant_volume
                pressurant_density = rocket.pressurant_mass/gas_volume
                rocket.pressurant_pressure = PropsSI('P','T',T,'D',pressurant_density, rocket.pressurant)
                rocket.maintank_pressure = rocket.pressurant_pressure
               
                # print(dp)
                v_e = loss * math.sqrt((2 * dp) / PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant))
                rocket.pressurant_massflow = v_e*A_nozzle*PropsSI('D','T',T,'P',rocket.maintank_pressure, rocket.pressurant)
                thrust = rocket.pressurant_massflow*v_e
                rocket.thrust = thrust
                rocket.massflow_propellant = 0
            else:
                thrust = 0
                rocket.thrust = thrust
                rocket.massflow_propellant = 0
                rocket.pressurant_massflow = 0
            
        else:
            print('thrust model does not exist')
        
        # print(thrust)
        return thrust
    #%% Drag
    @staticmethod
    def drag_coefficient(rocket, atmosphere):
        #only required at trans/supersonic speeds (also old code)
        C_W0 = 0.3
        Ma = abs(rocket.velocity/atmosphere.sonic_velocity(rocket.altitude))
        # Definitions
        R_d = 1.1
        A_0 = 0.75
        M_1 = 1.2
        M_2 = 1.325
        # functions
        A_1 = (1-A_0)/((M_1-1)**4)
        A_2 = 1-(A_1*((M_2-M_1)**4))
        if Ma <= 1:
            f_d = A_0*Ma**4
        elif Ma > 1 and Ma <= M_2:
            f_d = 1-(A_1*((Ma-M_1)**4))
        elif Ma > M_2:
            f_d = A_2*((Ma+1-M_2)**(-1))
    
        C_W = C_W0*(1+R_d*f_d)
        return C_W
    
    @staticmethod
    def drag(rocket, atmosphere, time, goes_supersonic):
        if goes_supersonic:
            #old code
            surface = math.pi*((rocket.diameter/2)**2)
            C_W = Models.drag_coefficient(rocket, atmosphere)
            
            drag = -np.sign(rocket.velocity)*C_W*(atmosphere.density(rocket.altitude)/2)*(rocket.velocity**2)*surface
            rocket.drag = drag
            return drag
            
        else:
            #new implementation | cd increase due to speed not considered
            #effective area is area times cd applied to that area
            effective_area = rocket.effective_area
            #parachute calculations
            if rocket.has_parachute:
                #selecting deploy_point on detecting negative velocity >5m/s
                if rocket.deploy_point == -1 and rocket.velocity < -5:
                    rocket.deploy_point = time + rocket.deploy_delay
                #calculating parachute drag
                elif rocket.deploy_point != -1 and rocket.deploy_point < time:
                    #parachute drag during deploy
                    if (rocket.deploy_point + rocket.deploy_time) > time:
                        effective_area += (-(rocket.deploy_point - time)/rocket.deploy_time) * rocket.effective_area_parachute
                    #parachute drag when fully deployed
                    else:
                        effective_area = rocket.effective_area_parachute
            
            #print(rocket.altitude)
            #print(time)
            rocket.drag = -np.sign(rocket.velocity) * effective_area * (atmosphere.density(rocket.altitude)/2) * (rocket.velocity ** 2)
            
