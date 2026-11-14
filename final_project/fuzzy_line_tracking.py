"""
UTC-CECS
ENEE4330: Fuzzy Logic Control

Final Project: Fuzzy Line Following 

	Davey Dickhut
	Austin Prichard
	Started: 2026-11-13
	Due: 2026-12-??
	
This program expands on previous work with the Sunfounder PicarX.
Previous iterations used naive guess-and-check algorithms for 
line following. While these approaches were effective, they were 
inelegant and inefficient. 
This approach uses the net grayscale reading from the sensors, as
well as the overall left-right balance to more smoothly correct 
the car's trajectory, and react in a more focused way to sharp 
turns.

~~~~~~~~~~~~~~~~~~~~~~~~
This is the main program
~~~~~~~~~~~~~~~~~~~~~~~~
"""
from picarx import Picarx
from time import sleep, monotonic

# Import fuzzy functions
import fuzzy_functions as fuzzy

# Set Alias for picarx specific functions
px = Picarx()
