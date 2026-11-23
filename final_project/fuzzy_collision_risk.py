"""
UTC-CECS
ENEE4330: Fuzzy Logic Control

Final Project

	Davey Dickhut
	Austin Prichard
	Started: 2026-11-13
	Due: 2026-12-??

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This program is adapted from example code provided by Dr. Moses Acquah.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Fuzzy Collision Risk program expands on previous work with the 
Sunfounder PicarX.
Previous iterations used sharp decision boundaries to control steering 
angle and propulsion direction. This iteration uses fuzzy logic to 
create smooth, gradient decision boundaries


"""

import argparse
import csv
import json
import os
from time import sleep
from time import monotonic

import numpy as np
import config_params
import fuzzy_functions as fz

# This error handling code should allow the same code to run on either 
# the simulated picar or the real picar
try:
    from picarx import Picarx
    HARDWARE_AVAILABLE = True
except ImportError:
    Picarx = None
    HARDWARE_AVAILABLE = False

class Chassis:
    def __init__(self):
        if HARDWARE_AVAILABLE:
            self.px = Picarx()
        else:
            self.px = None
            print("[SIM] Running in simulation mode (no picarx hardware)")

    def drive(self, speed: int, angle: float) -> None:
        if self.px:
            self.px.set_dir_servo_angle(angle)
            self.px.forward(speed)
        else:
            # Feed your simulator state or log output here
            pass
