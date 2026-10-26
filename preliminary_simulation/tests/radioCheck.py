"""Radio Check Test"""

import os
import sys
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"            # Hide pygame support prompt
os.environ["SDL_jOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"  # Allow background joystick events

import pygame

refreshRate = 20
barWidth = 31
movedThreshold = 0.1 # an axis is considered moved if its value is above this threshold


def axisBar(value, width=barWidth):
    """Draw a value in -1..1 as a bar of width characters"""
    # Implementation for drawing axis bar
    pass


def chooseJoystick():
    """List all connected joystick and open the one given on the command line or the first one if none is given"""
    # Implementation for choosing joystick
    pass


def printSummary(axisMin, axisMax):
    """After stopping show the range each axis covered"""
    # Implementation for printing summary
    pass


def main():
    """Main function to run the radio check test"""
    # Implementation for main function
    pass


if __name__ == "__main__":
    main()

    