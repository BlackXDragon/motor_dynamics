# Functions to simulate the dynamics of a multicopter motor

import numpy as np

class Motor:
    
    def __init__(self, KV = 880., RPMSlugFactor = 1, resistance = 0.125, prop_dia = 0.245, kP = 1.13, kT = 1, prop_mass = 0.0125, voltage = 16.8, batt_resistance = 0.00034):
        self.KV = KV
        self.RPMSlugFactor = RPMSlugFactor
        self.resistance = resistance
        self.prop_dia = prop_dia
        self.kP = kP
        self.kT = kT
        self.prop_mass = prop_mass
        self.voltage = voltage
        self.batt_resistance = batt_resistance
        
        self.KT = 60. / (2. * np.pi * self.KV)
        
        self.air_density = 1.225
        
        self.prop_inertia = prop_mass * prop_dia**2 / 12.
    
        self.throttle = 0.
        self.eff_voltage = 0.
        self.current = 0.
        self.torque = 0.
        self.drag_torque = 0.
        self.prev_rpm = 0.
        self.rpm = 0.
        self.thrust = 0.
        
    def step(self, pwm, time_step = 0.02):
        self.throttle = (pwm - 1000.) / 1000.
        self.throttle = np.clip(self.throttle, 0., 1.)
        
        self.eff_voltage = self.throttle * (self.voltage - self.batt_resistance * self.current)
        
        self.current = (self.KV * self.eff_voltage - self.rpm) / (self.resistance * self.KV)
        
        self.torque = self.KT * self.current
        
        self.drag_torque = self.kP * self.air_density * (self.rpm / 60.)**2 * self.prop_dia**5
        
        self.w = self.rpm * 2. * np.pi / 60.
        self.delta_w = (self.torque - self.drag_torque) / self.prop_inertia * time_step
        self.w_hat = self.w + self.delta_w
        
        self.rps = self.w_hat / (2. * np.pi)
        
        self.rps = max(self.rps, 0.)
        
        self.rpm = self.rps * 60.
        
        self.rpm = self.prev_rpm + self.RPMSlugFactor * (self.rpm - self.prev_rpm)
        self.prev_rpm = self.rpm
        
        self.thrust = 2.2 * self.kT * self.rps**2 * self.prop_dia**4
        
    def reset(self):
        self.throttle = 0.
        self.eff_voltage = 0.
        self.current = 0.
        self.torque = 0.
        self.drag_torque = 0.
        self.prev_rpm = 0.
        self.rpm = 0.
        self.thrust = 0.
        
class TimeStepInvariantMotor:
    
    def __init__(self, KV = 880., RPMSlugFactor = 1, resistance = 0.125, prop_dia = 0.245, kP = 1.13, kT = 1, prop_mass = 0.0125, voltage = 16.8, batt_resistance = 0.00034):
        self.KV = KV
        self.RPMSlugFactor = RPMSlugFactor
        self.resistance = resistance
        self.prop_dia = prop_dia
        self.kP = kP
        self.kT = kT
        self.prop_mass = prop_mass
        self.voltage = voltage
        self.batt_resistance = batt_resistance
        
        self.KT = 60. / (2. * np.pi * self.KV)
        
        self.air_density = 1.225
        
        self.prop_inertia = prop_mass * prop_dia**2 / 12.
    
        self.throttle = 0.
        self.eff_voltage = 0.
        self.current = 0.
        self.torque = 0.
        self.drag_torque = 0.
        self.prev_rpm = 0.
        self.rpm = 0.
        self.thrust = 0.
        
    def step(self, pwm, time_step = 0.02):
        self.throttle = (pwm - 1000.) / 1000.
        self.throttle = np.clip(self.throttle, 0., 1.)
        
        self.eff_voltage = self.throttle * (self.voltage - self.batt_resistance * self.current)
        
        self.current = (self.KV * self.eff_voltage - self.rpm) / (self.resistance * self.KV)
        
        self.torque = self.KT * self.current
        
        self.drag_torque = self.kP * self.air_density * (self.rpm / 60.)**2 * self.prop_dia**5
        
        self.w = self.rpm * 2. * np.pi / 60.
        self.delta_w = (self.torque - self.drag_torque) / self.prop_inertia * time_step
        self.w_hat = self.w + self.delta_w
        
        self.rpm = 60. * self.w_hat / (2. * np.pi)
        
        self.rpm = max(self.rpm, 0.)
        
        self.rpm = self.prev_rpm + self.RPMSlugFactor * (self.rpm - self.prev_rpm)
        
        # RPM fix for overshoots
        if self.torque != self.drag_torque:
            new_eff_voltage = self.throttle * (self.voltage - self.batt_resistance * self.current)
            new_current = (self.KV * new_eff_voltage - self.rpm) / (self.resistance * self.KV)
            new_torque = self.KT * new_current
            new_drag_torque = self.kP * self.air_density * (self.rpm / 60.)**2 * self.prop_dia**5
            
            # If overshoot, calculate the RPM where the torque and drag torque are equal
            if (new_torque > new_drag_torque and self.torque < self.drag_torque) or (new_torque < new_drag_torque and self.torque > self.drag_torque):
                a = self.kP * self.air_density * self.prop_dia ** 5 / 3600.
                b = self.KT / (self.resistance * self.KV)
                c = -self.KT * new_eff_voltage / self.resistance
                
                # Check discriminant
                d = b**2 - 4 * a * c
                
                if d < 0:
                    raise ValueError("Discriminant is negative")
                
                root1 = (-b + np.sqrt(d)) / (2 * a)
                root2 = (-b - np.sqrt(d)) / (2 * a)
                
                if root2 > 0:
                    self.rpm = root2
                else:
                    self.rpm = root1
        
        self.rpm = max(self.rpm, 0.)
        
        self.prev_rpm = self.rpm
        
        self.thrust = 2.2 * self.kT * (self.rpm / 60.)**2 * self.prop_dia**4
        
    def reset(self):
        self.throttle = 0.
        self.eff_voltage = 0.
        self.current = 0.
        self.torque = 0.
        self.drag_torque = 0.
        self.prev_rpm = 0.
        self.rpm = 0.
        self.thrust = 0.