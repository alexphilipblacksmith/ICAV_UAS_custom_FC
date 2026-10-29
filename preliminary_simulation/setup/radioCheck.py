"""Radio Check Test"""

import os
import sys
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"            # Hide pygame support prompt
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"  # Allow background joystick events

import pygame

refreshRate = 20
barWidth = 31
movedThreshold = 0.1 # an axis is considered moved if its value is above this threshold


def axisBar(value, width=barWidth):
    """Draw a value in -1..1 as a bar of width characters"""
    position = int((value + 1) / 2 * (width-1))
    position = min(max(position, 0), width-1) # clamp to [0, width-1]
    bar = ['-'] * width
    bar[width//2] = '|'
    bar[position] = '*'
    return "[" + "".join(bar) + "]"


def chooseJoystick():
    """List all connected joystick and open the one given on the command line or the first one if none is given"""
    count  = pygame.joystick.get_count()
    if count == 0:
        print("No joystick found")
        sys.exit(1)

    print("Joysticks found:")
    for index in range(count):
        print(f"  {index}: {pygame.joystick.Joystick(index).get_name()}")

    index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    if not 0 <= index < count:
        print(f"Invalid joystick index {index}")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return joystick


def printSummary(axisMin, axisMax):
    """After stopping show the range each axis covered"""
    print("\nAxis summary:")
    for i, (minVal, maxVal) in enumerate(zip(axisMin, axisMax)):
        if minVal is None:
            continue
        note = "moved" if maxVal - minVal > movedThreshold else "not moved"
        print(f"  Axis {i}: min={minVal:.3f}, max={maxVal:.3f} ({note})")


def main():
    """Main function to run the radio check test and give a csv file with the min and 
    max values of each axis. The user is prompted to move the sticks to their extremes 
    and the min and max values are recorded. The calibration is saved to a file for later use."""
    if os.name == "nt":
        os.system("")

    pygame.init()
    pygame.joystick.init()
    joystick = chooseJoystick()

    numAxes = joystick.get_numaxes()
    numButtons = joystick.get_numbuttons()
    numHats = joystick.get_numhats()

    print(f"Using joystick: {joystick.get_name()} ({numAxes} axes, {numButtons} buttons, {numHats} hats)")
    print("Press Ctrl+C to stop the test and show the axis summary.\n")

    axisMin = [None] * numAxes
    axisMax = [None] * numAxes
    linesPrinted = 0

    try:
        while True:
            # Process pygame events to update joystick state
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEREMOVED:
                    print("Joystick disconnected")
                    return

            # Read the current state of the joystick
            axes = [joystick.get_axis(i) for i in range(numAxes)]
            buttons = [joystick.get_button(i) for i in range(numButtons)]
            hats = [joystick.get_hat(i) for i in range(numHats)]

            # Track the range of each axis
            for i, value in enumerate(axes):
                axisMin[i] = value if axisMin[i] is None else min(axisMin[i], value)
                axisMax[i] = value if axisMax[i] is None else max(axisMax[i], value)

            # Build the output lines for axes, buttons, and hats
            lines = [f"Axes{i}: {value:+.3f} {axisBar(value)} min={axisMin[i]:+.3f}, max={axisMax[i]:+.3f}" for i, value in enumerate(axes)]
            pressed = [str(i) for i, value in enumerate(buttons) if value]
            lines.append(f"Buttons pressed: {', '.join(pressed) if pressed else 'None'}")

            for i, hat in enumerate(hats):
                lines.append(f"Hat {i}: {hat}")

            # Move the cursor up to overwrite previous lines
            if linesPrinted:
                sys.stdout.write(f"\033[{linesPrinted}A")  # Move cursor up
            sys.stdout.write("\n".join(line + "\033[K" for line in lines) + "\n")
            sys.stdout.flush()
            linesPrinted = len(lines)

            time.sleep(1 / refreshRate)

    except KeyboardInterrupt:
        pass
    finally:
        # Show the summary of axis ranges and quit pygame
        printSummary(axisMin, axisMax)
        pygame.quit()

        # Save the calibration values to a file in a CSV format
        with open("radio_check.csv", "w") as f:
            f.write("Axis,Min,Max\n")
            for i in range(numAxes):
                f.write(f"{i},{axisMin[i]},{axisMax[i]}\n")

if __name__ == "__main__":
    main()

    