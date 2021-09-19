# Temp file for notes and snippets of code.
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor,
from pybricks.parameters import Port, Direction, Color, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

left = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)


wheel_diameter = 56
axle_track = 126

robot = DriveBase(left, right, wheel_diameter, axle_track)
'''
def mission_one:
    robot.drive(500, 0)
    wait(1000)
    robot.drive(300, -45)
    wait(1000)
    robot.drive(-300, 45)
    wait(1000)
    robot.drive(-500, 0)
    wait(1000)
   
'''


def line_follow_until_color(line_sensor_right, line_sensor_left, drive_speed, proportional_gain, stop_color):
    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 10
    WHITE = 95
    threshold = (BLACK + WHITE) / 2

    while line_sensor_right.color() != stop_color:
        print(line_sensor_right.color())
        # Calculate the deviation from the threshold.
        deviation = line_sensor_left.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = proportional_gain * deviation

        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)


line_follow_until_color(
    line_sensor_right = ColorSensor(Port.E), 
    line_sensor_left = ColorSensor(Port.F), 
    drive_speed = 150,
    proportional_gain = 1.2,
    stop_color=Color.NONE)

robot.turn(20)
robot.drive(400,0)
wait(500)
robot.drive(-500,0)
wait(1200)
robot.turn(-90)
robot.turn(-120)
robot.drive(500,0)
wait(1250)

robot.stop()
#line_follow_until_distance(
#    line_sensor_right = ColorSensor(Port.F), 
#    drive_speed = 150,
#    proportional_gain = 1.2,
#    distance=1500)
