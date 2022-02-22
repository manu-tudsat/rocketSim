# -*- coding: utf-8 -*-
"""
Created on Sat Feb 12 2022

@author: Manu
"""

#%% Imports

import math
from CoolProp.CoolProp import PropsSI

class Rocket():
    #%% Init
    def __init__(self, vehicle_properties, propellants, engine):
        self.altitude = 0
        self.velocity = 0
        self.acceleration = 0
        self.drag = 0
        
        self.mass_empty = vehicle_properties["mass_empty"]
        self.diameter = vehicle_properties["diameter"]
        self.height = vehicle_properties["height"]
        self.fin_surface = vehicle_properties["fin_surface"]
        
        self.surface_parallel = self.fin_surface + self.height * self.diameter * math.pi
        self.surface_perpendicular = (self.diameter / 2) ** 2 * math.pi
        
        self.cw_perpendicular = vehicle_properties["drag_coefficient_perpendicular"]
        self.cw_parallel = vehicle_properties["drag_coefficient_parallel"]
        
        self.effective_area = self.surface_parallel * self.cw_parallel + self.surface_perpendicular * self.cw_perpendicular
        
        self.has_parachute = vehicle_properties["has_parachute"]
        
        if self.has_parachute:
            self.parachute_diameter = vehicle_properties["parachute_diameter"]
            self.parachute_area = self.parachute_diameter ** 2 / 4 * math.pi
            self.cw_parachute = vehicle_properties["drag_coefficient_parachute"]
            self.effective_area_parachute = self.cw_parachute * self.parachute_area
            self.deploy_delay = vehicle_properties["deploy_delay"]
            self.deploy_time = vehicle_properties["deploy_time"]
            self.deploy_point = -1
        
        self.engine = engine
        
        
        
        self.propellants = []
        
        for prop in propellants:
            self.propellants.append(self.Propellant(prop))
    
        
    
            
        
    #%% Mass
    def mass(self):
        '''Returns current mass of vehicle'''
        mass = self.mass_empty
        
        for prop in self.propellants:
            mass += prop.mass
            
        return mass
    
    #%% Inner class propellant tank
    class Propellant():
        
        def __init__(self, properties):
            self.tank_volume = properties["tank_volume"]
            
            self.name = properties["propellant_name"]
            self.state = properties["propellant_state"]
            
            self.pressure = properties["propellant_pressure"]
            self.temperature = properties["propellant_temperature"]
            
            self.density = PropsSI("D","T",self.temperature,"P",self.pressure, self.name)
            
            if self.state == "gas":
                self.initial_mass = self.tank_volume * self.density
                
            elif self.state == "liquid":
                self.initial_mass = properties["propellant_volume"] * self.density
                
            self.mass = self.initial_mass
                
            
            
            
        def volume(self):
            volume = 0
            
            if self.state == "gas":
                volume = self.tank_volume
                
            elif self.state == "liquid":
                volume = self.mass / self.density
                
            return volume