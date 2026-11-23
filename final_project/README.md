# Final Project

This will be where work on the project actually takes place

## Project Structure

I propose we structure this project like an embedded C project, with the following file structure.

- final_project/
	- hardware_simulation.py	# Move line-following simulation out of main line-following script
	- fuzzy_line_following.py	# Main control loop for line following
	- fuzzy_collision_risk.py	# Main control loop for obstacle avoidance
	- fuzzy_line_collision.py	# Main control loop for unknown third program
	- config_params.py			# Dictionaries and hardcoded constants for all programs
	- fuzzy_functions.py		# Functions specific to fuzzy logic
	- logging_functions.py		# Functions for datalogging

Using this structure, each main file would begin with:

```
import argparse
import csv
import json
import os
import time

import numpy as np
import config_params
import fuzzy_functions

from picarx import Picarx
px = Picarx()
```

This way, we won't have to worry about redefining stuff for our final project, errors can be isolated to specific "header files", and the simulation can be independent of the main robot logic
