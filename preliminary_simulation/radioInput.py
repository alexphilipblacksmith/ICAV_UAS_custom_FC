"""Reads the radio (Radiomaster Pocket, USB joystick) and scales the sticks with the calibration from setup/radioCheck.py"""

import csv
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame

movedThreshold = 0.1 # an axis whose calibrated range is smaller than this was not moved during the radio check


class RadioDisconnected(ConnectionError):
    """Raised when the joystick is unplugged while flying"""


@dataclass
class RadioCommand:
    """Scaled stick positions"""

    roll: float      # -1..1, + = stick right
    pitch: float     # -1..1, + = stick forward
    yaw: float       # -1..1, + = stick right
    throttle: float  # 0..1,  0 = stick fully down


def loadCalibration(path):
    """Read the calibration csv written by setup/radioCheck.py.

    Returns:
        minVals, centreVals, maxVals: one value per joystick axis
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Calibration file {path} not found. Run setup/radioCheck.py from the setup folder first.")

    minVals, centreVals, maxVals = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if "Centre" not in reader.fieldnames:
            raise ValueError(f"{path} has no Centre column. Run setup/radioCheck.py again to record the stick centres.")
        for row in reader:
            minVals.append(float(row["Min"]))
            centreVals.append(float(row["Centre"]))
            maxVals.append(float(row["Max"]))

    return np.array(minVals), np.array(centreVals), np.array(maxVals)


def scaleStick(raw, minVal, centreVal, maxVal):
    """Map a raw spring-centred axis to -1..1: centre -> 0, max -> +1, min -> -1.
    Each side is scaled separately because the centre is rarely exactly in the middle of the range."""
    if raw >= centreVal:
        value = (raw - centreVal) / (maxVal - centreVal)
    else:
        value = (raw - centreVal) / (centreVal - minVal)
    return float(np.clip(value, -1.0, 1.0))


def scaleThrottle(raw, minVal, maxVal):
    """Map the raw throttle axis to 0..1. The throttle does not spring back so its centre is not used."""
    return float(np.clip((raw - minVal) / (maxVal - minVal), 0.0, 1.0))


def applyDeadband(value, deadband):
    """Ignore small movements around the centre. The rest is rescaled so the output still reaches +-1
    and has no jump at the edge of the deadband."""
    if abs(value) < deadband:
        return 0.0
    return float(np.sign(value) * (abs(value) - deadband) / (1.0 - deadband))


@dataclass(eq=False)
class RadioInput:
    """Opens the joystick and returns scaled stick positions.

    Args:
        calibrationFile: csv written by setup/radioCheck.py
        axes: joystick axis number of each stick, keys roll, pitch, throttle, yaw
        signs: +1 or -1 per stick so that roll right, pitch forward, yaw right and throttle up are positive
        deadband: stick deadband around the centre (roll, pitch, yaw only)
        joystickIndex: which joystick to open if several are connected
    """

    # Settings (constructor arguments)
    calibrationFile: Path
    axes: dict
    signs: dict
    deadband: float = 0.05
    joystickIndex: int = 0

    # Created in __post_init__ (not constructor arguments)
    joystick: object = field(init=False, repr=False)
    calMin: np.ndarray = field(init=False, repr=False)
    calCentre: np.ndarray = field(init=False, repr=False)
    calMax: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        """Load the calibration and open the joystick."""
        self.calMin, self.calCentre, self.calMax = loadCalibration(self.calibrationFile)

        for name, axis in self.axes.items():
            if axis >= len(self.calMin):
                raise ValueError(f"{name} uses axis {axis} but the calibration file only has {len(self.calMin)} axes.")
            if self.calMax[axis] - self.calMin[axis] < movedThreshold:
                raise ValueError(f"{name} (axis {axis}) was not moved during the radio check. Run setup/radioCheck.py again.")
            if name != "throttle" and (self.calMax[axis] <= self.calCentre[axis] or self.calCentre[axis] <= self.calMin[axis]):
                raise ValueError(f"{name} (axis {axis}) centre is not between its min and max. Run setup/radioCheck.py again.")

        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            pygame.quit()
            raise RadioDisconnected("No joystick found. Plug in the radio and select USB joystick mode.")
        if not 0 <= self.joystickIndex < count:
            pygame.quit()
            raise ValueError(f"Joystick index {self.joystickIndex} not available ({count} found).")

        self.joystick = pygame.joystick.Joystick(self.joystickIndex)
        self.joystick.init()
        if self.joystick.get_numaxes() != len(self.calMin):
            print(f"Warning: joystick has {self.joystick.get_numaxes()} axes but the calibration has {len(self.calMin)}. "
                  "Was the calibration made with this radio?")
        print(f"Using joystick: {self.joystick.get_name()}")

    def read(self):
        """Read and scale the sticks.

        Returns:
            RadioCommand
        """
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                raise RadioDisconnected("Joystick disconnected")

        def stick(name):
            axis = self.axes[name]
            value = scaleStick(self.joystick.get_axis(axis), self.calMin[axis], self.calCentre[axis], self.calMax[axis])
            return applyDeadband(self.signs[name] * value, self.deadband)

        throttleAxis = self.axes["throttle"]
        throttle = scaleThrottle(self.joystick.get_axis(throttleAxis), self.calMin[throttleAxis], self.calMax[throttleAxis])
        if self.signs["throttle"] < 0:
            throttle = 1.0 - throttle

        return RadioCommand(roll=stick("roll"), pitch=stick("pitch"), yaw=stick("yaw"), throttle=throttle)

    def close(self):
        """Release the joystick."""
        pygame.quit()
