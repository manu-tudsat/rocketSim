# -*- coding: utf-8 -*-
#%% Info
"""
Created on Fri Feb 14 15:00:21 2020
Modified on Tue Feb 22 2022

@author: Sascha Dengler
@author: Manu

TU Darmstadt Space Technology e.V.
Rocket Development
1-D vertical Flight Simulation Tool

Modules:
-Sim: PhysicsEngine to derive and solve equations of motion
-Models: Physical models used in Sim
-Rocket: Current state of rocket
-ISA_atmosphere: Definition of atmosphere class (ISA standard)
-Engine: Current State and Behaviour of rocket engine

Undergoing severe overhaul, many functions currently unsupported

Recommended Use:
    - PreDesign of rockets to determine system-specifications to achieve requirements
    - Derivation of loads of given rocket during ascend and descend
    - Parameter studies for system optimization
    (- possibly hardware in the loop application in avionics flight computer)
    
    - As an example to get familiar with Python :)

Disclaimer:
All units given in SI-system:

distance - m
area - m²
volume - m³
mass - kg
time - s
velocity - m/s
acceleration - m/s²
force - N
temperature - K
pressure - Pa

"""

#%% Imports
# ----------------------------
# internal
from Sim import PhysicsEngine
# import Models
from Rocket import Rocket
from Engine import Engine
from ISA_atmosphere import ISA_atmosphere

# external
import math
import numpy as np
import matplotlib.pyplot as plt
import time as timer

#%% Instructions
# ----------------------------
"""
To simulate a rocket the rocket itself as well as the propellants and the engine
need to be specified with one dictionary each. Those currently include:
    
Rocket:
    General
    - mass_empty - dry mass of the vehicle
    Drag
    - diameter - diameter of the vehicle
    - height - height of the vehicle
    - fin_surface - total surface area of the fins
    - drag_coefficient_perpendicular - Cd value for all surfaces
        perpendicular to the airflow (currently nose derived from diameter)
    - drag_coefficient_parallel - Cd value for all surfaces parallel to the
        airflow (currently skin surface derived from diameter and height plus fin surface)
    Parachute
    - has_chute - whether or not the rocket has a parachute equipped
    - parachute_diameter - diameter of the fully deployed parachute
    - drag_coefficient_parachute - Cd value for the parachute
    - deploy_delay - delay between detecting descent and deployment of the parachute
        (detection currently means speed of greater than 5m/s downwards)
    -deploy_time - time the chut takes to deploy (currently modeled
        as a linear increase of drag during that time)
    
Propellant:
    Tank
    - tank_volume - total volume of the tank
    Propellant properties
    - propellant_name - name of the propellant (used for connection with CoolProp)
    - propellant_pressure - pressure the propellant is stored at
    - propellant_temperature - temperature of the propellant
    - propellant_state - state of the propellant (only used for mass calculation)
    - propellant_volume - volume of the loaded propellant (liquids only)

Engine:
    Paintball (these are engine specific)
    - pressure_max - pressure regulators lockup pressure
    - pressure_efficiency - how much of the pressure gets converted into flow
    - exhaust_efficiency - how much of the mass flow gets converted into thrust
    - exhaust_area - crosssectional area of the exhaust
"""

#%% Initialization
# ----------------------------

# Athena02
athena_02_paintball = {
    "tank_volume": 0.0002,
    "propellant_name": "Air",
    "propellant_pressure": 200e5,
    "propellant_temperature": 288,
    "propellant_state": "gas"
}

athena_02_main_tank = {
    "tank_volume": 0.0031,
    "propellant_name": "Water",
    "propellant_volume": 0.0031,
    "propellant_pressure": 20e5,
    "propellant_temperature": 288,
    "propellant_state": "liquid"
}

athena_02_properties = {
    "mass_empty": 0.85 + 0.25,
    "diameter": 0.078,
    "height": 0.9,
    "fin_surface": 0.01 * 4,
    "drag_coefficient_perpendicular": 0.3,
    "drag_coefficient_parallel": 0.005,
    "has_parachute": True,
    "parachute_diameter": 1,
    "drag_coefficient_parachute": 0.8,
    "deploy_delay": 1.5,
    "deploy_time": 0.5
}

athena_02_engine_properties = {
    "pressure_max": 20e5,
    "pressure_efficiency": 0.9,
    "exhaust_efficiency": 0.95,
    "exhaust_area": 0.012 ** 2 * math.pi / 4
}
athena_02_engine = Engine("paintball", athena_02_engine_properties)

athena_02 = Rocket(athena_02_properties, [athena_02_main_tank, athena_02_paintball], athena_02_engine)

# Atmosphere:
atmosphere = ISA_atmosphere()

#%% Simulation
#-------------------------------
# Initial parameter
t_start = 0
#

rocket = athena_02

logging = True

solve_adaptive = False
timestep = 1/10000
#ini save
time = t_start

#Logging setup
if logging:
    log = []
    log.append([0, rocket.altitude, rocket.velocity, rocket.acceleration, rocket.mass(), rocket.drag, rocket.engine.thrust, rocket.engine.exhaust_velocity, rocket.engine.mass_flow])

# Start of Simulation
# start timer
start = timer.time()

while rocket.altitude >= 0 and time < 60:
    
    if solve_adaptive == True:
        #Currently broken
        # solve
        #x_new,h_next,h_used,err = PhysicsEngine.Solve_adaptive(x, rocket, atmosphere, time, h, 'RKDP')

        
        # update
        #x = x_new
        #time += h_used
        #h = h_next
        
        # print(t)
        
        # save
        break
        
    
    else:
        #x_new = PhysicsEngine.Solve(x, rocket, atmosphere, t, 'Euler_exp',h_ini)
        PhysicsEngine.Step(rocket, atmosphere, time, timestep)
        # update
        time = time + timestep
        
        if logging:
            log.append([time, rocket.altitude, rocket.velocity, rocket.acceleration, rocket.mass(), rocket.drag, rocket.engine.thrust, rocket.engine.exhaust_velocity, rocket.engine.mass_flow])
        
            
        
    
end = timer.time()
print('elapsed calculation time: ' + str(round(end-start,3)) + ' s')
#%% Results
# -------------------------------
# plot etc
if logging:
    #plot 1
    plot1, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
    
    log = np.array(log)
    ax1.plot(log[:,0],log[:,1], color="darkred")
    ax2.plot(log[:,0],log[:,2], color="red")
    ax3.plot(log[:,0],log[:,3], color="orange")
    ax4.plot(log[:,0],log[:,5], color="blue")
    
    ax1.set_ylabel("alt")
    ax2.set_ylabel("vel")
    ax3.set_ylabel("acc")
    ax4.set_ylabel("drag")
    
    plt.show()
    
    #plot 2
    plot2, (ax5, ax6, ax7, ax8) = plt.subplots(4, 1, sharex=True)
    
    ax5.plot(log[:,0],log[:,4], color="yellow")
    ax6.plot(log[:,0],log[:,6], color="limegreen")
    ax7.plot(log[:,0],log[:,7], color="green")
    ax8.plot(log[:,0],log[:,8], color="darkgreen")
    
    ax5.set_ylim([-0.1,5.1])
    plt.xlim([-0.1,2.1])
    
    ax5.set_ylabel("mass")
    ax6.set_ylabel("thr")
    ax7.set_ylabel("v_e")
    ax8.set_ylabel("massflow")
    
    
    
    plt.show()
    
    print("Apogee: " + str(round(max(log[:,1]),3)) + " m")
    print("Max Vel: " + str(round(max(log[:,2]),3)) + " m")
    print("Max Acc: " + str(round(max(log[:,3]),3)) + " m")
    print("Touchdown Vel: " + str(round(abs(log[-1,2]),3)) + " m")
    print("Max Thrust: " + str(round(max(log[:,6]),3)) + " N")
