import math
import numpy as np
import sys

    
class SensoryManager():

    
    def __init__(self):
        self.pressureParticles = []
        self.latest_pressure = {}


    def applyPressure(self, id, pressure, c302NrnSimulation):
        # load list of neurons/sections which should receive additional input here?
        #if id not in pressureParticles:
         pressureParticles.append(id)
         c302NrnSimulation.ns.addStimulus("a_AVM", pressure)

    def resetParticleList():
        self.pressureParticles = []

    # TODO
    def calcSectionPressure(self, section):
        max_pressure = -1
        for p in section.particles:
            if p.pressure > max_pressure:
                max_pressure = p.pressure
        return max_pressure

    # TODO
    def updatePressure(self):
        # for section in sections:
        #     current_pressure = self.calcSectionPressure(section)
        #     latest_pressure = latest_pressure[section.name]
        #     if latest_pressure and current_pressure > latest_pressure:
        #         pass
        #     latest_pressure[section.name] = current_pressure
              
        pass

    def addStimulus(self, sectionName, pressure, h):
        if not h:
            return # MuscleSimulation
        section = getattr(h, sectionName)
        i = h.IClamp(sectionName(0.5))
        i.delay = 0    # in ms
        i.dur = 0.02   # in ms - set to dt
        i.amp = 20     # in ?

