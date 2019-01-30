import math
import numpy as np
import sys

    
class SensoryManager():

    
    def __init__(self):
        pass


    def applyPressure(self, id, pressure, c302NrnSimulation):
        # load list of neurons/sections which should receive additional input here?
        self.addStimulus('a_AVM[0]', pressure, c302NrnSimulation.ns.h)
        pass


    def addStimulus(self, sectionName, pressure, h):
        if not h:
            return # MuscleSimulation
        section = h.sectionName
        i = h.IClamp(sectionName(0.5))
        i.delay = 0    # in ms
        i.dur = 0.02  # (dt) in ms
        i.amp = 20     # in ?

