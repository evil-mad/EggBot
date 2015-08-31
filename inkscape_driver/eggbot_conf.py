# eggbot-conf.py
# Part of the EggBot driver for Inkscape
# Version 2.6.0, dated August 31, 2015.
#
# https://github.com/evil-mad/EggBot/
#
# "Change numbers here, not there." :)



# Page size values typically do not need to be changed. They primarily affect viewpoint and centering.

N_PAGE_HEIGHT = 800       # Default Inkscape page height (each unit equiv. to one pixel/step)
N_PAGE_WIDTH = 3200       # Default Inkscape page width (each unit equiv. to one pixel/step) 


'''
Motor resolution: The "standard" setup for brand-name EggBot machines (at least through 2015) has been 3200 steps per revolution. 

Early (clear-chassis) models had 400 step/rev motors with 8X microstepping drivers, and more recent versions ("white-chassis" EggBot 2.0, Deluxe EggBot, Ostrich EggBot, and EggBot Pro) have all used 200 step/rev motors with 16X microstepping drivers (EBB 2.0).

The STEP SCALING FACTOR below can be used to work with motor-driver combinations that give a different overall number of steps per revolution-- for example 1600 (200 step/rev motors with 8X drivers) or 6400 (400 step/rev motors with 16X drivers) for "corner cases" and unofficial builds.


For 1600 steps/rev, use:
	STEP_SCALE = 4

For NORMAL USE, with overall 3200 steps/rev, use the default value:
	STEP_SCALE = 2

For 6400 steps/rev, use:
	STEP_SCALE = 1
	
Other _integer_ scaling values can be used as well, with similar scaling.

'''

STEP_SCALE = 2
