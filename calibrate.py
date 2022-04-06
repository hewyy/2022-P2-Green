import numpy as np
import math
from joy.plans import Plan
from joy.decl import progress
import os
import json

import tinyik

class Calibrate(Plan):
    def __init__(self, app):
        Plan.__init__(self,app)
        self.app = app
        self.calibCoords = np.array([[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [1, 0.5, 0], [1, 1, 0], [0.5, 1, 0], [0, 1, 0], [0, 0.5, 0]]) #these are placeholders
        self.calibCoords = self.calibCoords*2.5
        self.calibCoords = self.calibCoords + np.array([[8.5/2, 8, 0]])

        self.calibCoordsWorld = []
        for rc in self.calibCoords:
            self.calibCoordsWorld.append(self.app.paperToWorld(rc))
        self.calibCoordsWorld = np.array(self.calibCoordsWorld)

        self.armik = tinyik.Actuator(['z',[1.,0.,0.],'y',[5.,0.,0.],'y',[5.,0.,0.]])
        self.coordIdx = 0 #which of these coords its currently at

    def behavior(self):
        #when the button is pushed, record the current set of angles as corresponding with the 
        #current calibration position (unless this is the first press), then move to an estimate of the 
        #next position (unless this is the last time)
        if self.coordIdx != -1:
            self.armik.ee = self.calibCoordsWorld[self.coordIdx]
            angles = self.armik.angles
            progress("Recorded calibration position: " + str(self.coordIdx))
            progress("Calculated angles: " + str(angles))
            self.app.arm[0].set_pos(np.round(np.rad2deg(self.armik.angles[0])))
            self.app.arm[1].set_pos(np.round(np.rad2deg(self.armik.angles[1])))
            self.app.arm[2].set_pos(np.round(np.rad2deg(self.armik.angles[2])))
        