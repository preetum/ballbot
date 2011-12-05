"""
Contains parameters that define court, car and lattice properties
"""

SEARCHALGORITHM ="MT-AdaptiveA*" # allowed: "MT-AdaptiveA*","A*","LPA*","straightline","figureofeight"

# Robot properties
ROBOT_LENGTH = 49         # 49 cm
ROBOT_WIDTH  = 28         # 28 cm
ROBOT_RADIUS_MIN = 70.0   # 70 cm
ROBOT_RADIUS_2   = 119.5  # 119.5 cm
ROBOT_RADIUS_3   = 84.497 # 84.497 cm
ROBOT_RADIUS_4   = 74.246 # 74.246 cm
ROBOT_RADIUS_5   = 76.6845 # 76.6845 cm
ROBOT_SPEED_MAX = 100.0   #100 cm/s 

# Environement Properties
COURT_LENGTH = 3657.6    #cm
COURT_WIDTH = 1828.8     #cm

#LATTICE PROPERTIES
CELL_SIZE = ROBOT_RADIUS_MIN/4 # 17.5 cm
