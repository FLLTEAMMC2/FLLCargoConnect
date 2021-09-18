# This file contains all the movement and control instructions.
# By putting the commands here, everyone can use the same command and have the same behavior on the robot.

# Some basic ideas are listed below
# enter code for move straight
def MoveStraight():

# enter code to rotate the robot using both wheels
def RotateByDegrees():
  
def RotateFirstAccessory():
  
def FollowLineForDistanceWithRightSensor():
  
  
def line_follow_until_color(line_sensor_right, line_sensor_left, drive_speed, proportional_gain, stop_color):
    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 10
    WHITE = 95
    threshold = (BLACK + WHITE) / 2

    while line_sensor_right.color() != stop_color:
        #print(line_sensor.reflection(), line_sensor.color(), line_sensor.hsv())
        # Calculate the deviation from the threshold.
        deviation = line_sensor_left.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = proportional_gain * deviation

        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)

def line_follow_until_distance(line_sensor, drive_speed, proportional_gain, distance):
    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 10
    WHITE = 95
    threshold = (BLACK + WHITE) / 2

    while robot.distance() < distance:
        #print(line_sensor.reflection(), line_sensor.color(), line_sensor.hsv())
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = proportional_gain * deviation

        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)

# Sample Function Calls        
# line_follow_until_distance(
#    line_sensor_right = ColorSensor(Port.F), 
#    drive_speed = 150,
#    proportional_gain = 1.2,
#   distance=1500)
#        
# line_follow_until_color(
#    line_sensor_right = ColorSensor(Port.F), 
#    line_sensor_left = ColorSensor(Port.E), 
#    drive_speed = 150,
#    proportional_gain = 1.2,
#    stop_color=Color.RED)
        
  
