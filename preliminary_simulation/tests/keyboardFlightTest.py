"""Fly the manual (level) mode simulation with the keyboard instead of the radio.

Run from anywhere:  python tests/keyboardFlightTest.py
Click on the plot window first so it receives the key presses.

    W / S         throttle up / down (stays where you leave it, starts at 0)
    A / D         yaw left / right
    Up / Down     pitch forward / back
    Left / Right  roll left / right
    Space         throttle to hover (mid-stick)
"""

import sys
import time as clock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent)) # so the simulation modules can be imported

import matplotlib.pyplot as plt

import main
from radioInput import RadioCommand

throttleRate = 0.4   # throttle change per second while W or S is held [1/s]
stickRate = 4.0      # how fast the virtual sticks move to / spring back from full deflection [1/s]
controlKeys = ["w", "a", "s", "d", "up", "down", "left", "right", " "]


class KeyboardInput:
    """Stands in for RadioInput: same read() and close(), but the sticks come from the keyboard."""

    def __init__(self, *args, **kwargs):
        self.pressedKeys = set()
        self.throttle = 0.0
        self.sticks = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.lastTime = None
        self.connected = False

        # Stop matplotlib's own shortcuts (s = save, left/right = back/forward, ...) from using the control keys
        for name in plt.rcParams:
            if name.startswith("keymap."):
                plt.rcParams[name] = [key for key in plt.rcParams[name] if key.lower() not in controlKeys]

        print(__doc__)

    def connectToWindow(self):
        """Listen to the key presses of the dashboard window (it is created after this object in main.py)."""
        if self.connected or not plt.get_fignums():
            return
        figure = plt.figure(plt.get_fignums()[-1])
        figure.canvas.mpl_connect("key_press_event", self.keyPressed)
        figure.canvas.mpl_connect("key_release_event", self.keyReleased)
        self.connected = True

    def keyPressed(self, event):
        if event.key is not None:
            self.pressedKeys.add(event.key.lower())

    def keyReleased(self, event):
        if event.key is not None:
            self.pressedKeys.discard(event.key.lower())

    def read(self):
        """Move the virtual sticks according to the held keys.

        Returns:
            RadioCommand
        """
        self.connectToWindow()

        now = clock.perf_counter()
        elapsed = 0.0 if self.lastTime is None else now - self.lastTime
        self.lastTime = now

        keys = self.pressedKeys

        # Throttle stays where it is left, like a real throttle stick
        if "w" in keys:
            self.throttle += throttleRate * elapsed
        if "s" in keys:
            self.throttle -= throttleRate * elapsed
        if " " in keys:
            self.throttle = 0.5
        self.throttle = min(max(self.throttle, 0.0), 1.0)

        # The other sticks move towards full deflection while a key is held and spring back to the centre after
        targets = {
            "roll": ("right" in keys) - ("left" in keys),
            "pitch": ("up" in keys) - ("down" in keys),
            "yaw": ("d" in keys) - ("a" in keys),
        }
        for name, target in targets.items():
            step = stickRate * elapsed
            difference = target - self.sticks[name]
            self.sticks[name] += max(-step, min(step, difference))

        return RadioCommand(roll=self.sticks["roll"], pitch=self.sticks["pitch"],
                            yaw=self.sticks["yaw"], throttle=self.throttle)

    def close(self):
        """Nothing to release."""


if __name__ == "__main__":
    main.simType = "manual"
    main.RadioInput = KeyboardInput # main.py builds its "radio" from this, so the keyboard replaces the radio
    main.main()
