from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Button, Side, Port, Direction
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
import utility


# STATE 1 Initialize global variables
Robot = PrimeHub()

utility.MoveStraight()


# STATE 2 Start monitor loop
# Check for button presses, increment selection number, start selection program