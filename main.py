from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Button, Side, Port, Direction
from pybricks.tools import wait
from pybricks.geometry import Axis
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
import utility

# When a new solution is made, make a solution file, such as solution_1.py, solution_2.py. 
# Add all solutions to the list here so that the solution function can be called, such as solution_1()
import solution_1
import solution_2


# STATE 1 Initialize global variables
Robot = PrimeHub()
leftmotor = Motor(Port.C)
rightmotor = Motor(Port.D)

topmotor = Motor(Port.B)
bottommotor = Motor(Port.A)

# Define number of solution slots
NumberOfSolutions = 15

# Selection number to run solution
solution = 0

# Set bluetooth button to stop button. Check out beta.pybricks.com for some examples.

# Define functions to be used globally

# Monitor function definition, this is the "boot loader" - can you spot and fix the bugs below?
# HINT: What is "hub." ?
# Can you change the monitor function to always run first as main?
def monitor():
  
  while True:
    pressed = []
    while not any(pressed):
      pressed = hub.buttons.pressed()
      wait(10)
    
    # Wait for all buttons to be released.
    while any(hub.buttons.pressed()):
      wait(10)
    
    # Increment selection number based on button pressed.
    if Button.LEFT in pressed: # check if the left button is pressed
        solution += -1 # If the left button was pressed, decrement the solution number by 1
        if solution < 0 # if the solution number is less than 0, wrap the number around to the max number. This way only numbers 0 to NumberOfSolutions (initially 15) is used.
          solution = NumberOfSolutions # Set solution number to the max number
        hub.display.number(solution) # Display the solution number on the PrimeHub
        pressed.clear() # clear the array storing the button press to keep everything tidy.
    elif Button.RIGHT in pressed:
        solution += 1
        if solution > NumberOfSolutions
          solution = 0
        hub.display.number(solution)
        pressed.clear()
    elif Button.CENTER in pressed:
    # If center button is pressed, run the selected mission
      # Python switch statement syntax
      if solution == 0: # run solution 0
        solution_0() # is there a solution 0?
      elif solution == 1: # run solution 1
        solution_1()
      elif solution == 2: # run solution 2
        solution_3() #does this seem right to you?
        # Add the rest of the solutions
      else :
        # default solution, what to do if a solution number is not implemented above?

# STATE 2 Start monitor loop
# Check for button presses, increment selection number, start solution program

# Remember that a function must always be "defined" before it can be used. So monitor() must be called after def monitor():
# Start monitor function
monitor()