# -*- coding: utf-8 -*-
"""
Created on Thu Feb 10 2022

@author: Manu
"""

import math
from CoolProp.CoolProp import PropsSI

class Engine():
    
    def __init__(self, engine_type, parameters):
        self.thrust = 0
        self.engine_type = engine_type
        
        if engine_type=="paintball":
            self.max_pressure = parameters["pressure_max"]
            self.pressure_efficiency = parameters["pressure_efficiency"]
            self.exhaust_efficiency = parameters["exhaust_efficiency"]
            self.exhaust_area = parameters["exhaust_area"]
            self.pressure_rate = parameters["pressure_rate"]
            self.engine_on = True
            
            self.exhaust_velocity = 0
            self.flow_volume = 0
            self.mass_flow = 0
            self.thrust = 0
            
        elif engine_type=="noctua":
            self.awesomeness = 100
            
    def calculate_thrust(self, rocket, atmosphere):
        
        if self.engine_type=="paintball":
            water = rocket.propellants[0]
            
            if self.engine_on:
                ambient_pressure = atmosphere.pressure(rocket.altitude)
                pressure_difference = water.pressure - ambient_pressure
                if pressure_difference > 0:
                    self.exhaust_velocity = self.pressure_efficiency * math.sqrt((2 * pressure_difference) / water.density)
                    self.volume_flow = self.exhaust_velocity * self.exhaust_area
                    self.mass_flow = self.volume_flow * water.density
                    self.thrust = self.mass_flow * self.exhaust_velocity * self.exhaust_efficiency
                else:
                    self.exhaust_velocity = 0
                    self.flow_volume = 0
                    self.mass_flow = 0
                    self.thrust = 0
                    self.engine_on = False
                    
            else:
                self.exhaust_velocity = 0
                self.flow_volume = 0
                self.mass_flow = 0
                self.thrust = 0
        
        elif self.engine_type=="noctua":
            self.thust = 0
        
    def update_properties(self, rocket, atmosphere, delta_time):
        
        if self.engine_type=="paintball":
            if self.engine_on:
                water = rocket.propellants[0]
                air = rocket.propellants[1]
                
                water.mass -= self.mass_flow * delta_time
                if water.mass <= 0:
                    self.engine_on = False
                    water.mass = 0
                
                else:
                    if water.pressure <= self.max_pressure:
                        pressure_difference = air.pressure - water.pressure
                        pressure_difference_new = pressure_difference * self.pressure_rate ** delta_time
                        air.pressure = pressure_difference_new + water.pressure
                        air.density = PropsSI("D","T", air.temperature, "P", air.pressure, air.name)
                        air.mass = air.density * air.tank_volume
                        
                    air_mass_water_tank = air.initial_mass - air.mass
                    air_density_water_tank = air_mass_water_tank / (water.tank_volume - water.volume())
                    water.pressure = PropsSI("P", "T", air.temperature, "D", air_density_water_tank, air.name)
                    
                    
                    # if air.pressure > self.max_pressure:
                    #     air_mass_flow = self.volume_flow * delta_time * PropsSI("D","T",air.temperature,"P",self.max_pressure, air.name)
                    #     air.mass -= air_mass_flow
                    #     air.density = air.mass / air.tank_volume
                    #     air.pressure = PropsSI("P","T",air.temperature,"D",air.density, air.name)
                    # else:
                    #     #air_mass_flow = self.volume_flow * delta_time * PropsSI("D","T",air.temperature,"P",air.pressure, air.name)
                    #     total_air_volume = water.tank_volume + air.tank_volume - water.volume()
                    #     total_air_density = air.initial_mass / total_air_volume
                    #     total_air_pressure = PropsSI("P","T",air.temperature,"D",total_air_density,air.name)
                        
                    #     air.density = total_air_density
                    #     air.mass = air.density * air.tank_volume
                    #     air.pressure = total_air_pressure
                        
                    #     water.pressure = air.pressure
                    
                    # water.density = PropsSI("D","T",water.temperature,"P",water.pressure, water.name)
                    
    def prepressurize(self, rocket):
        
        if self.engine_type=="paintball":
            water = rocket.propellants[0]
            air = rocket.propellants[1]
            
            air_volume = water.tank_volume - water.volume()
            air_density_water_tank = PropsSI("D", "T", air.temperature, "P", self.max_pressure, air.name)
            air_mass_water_tank = air_density_water_tank * air_volume
            
            if air_mass_water_tank < air.mass:
                air.mass -= air_mass_water_tank
                air.density = air.mass / air.tank_volume
                air.pressure = PropsSI("P", "T", air.temperature, "D", air.density, air.name)
                
                
            
                
            