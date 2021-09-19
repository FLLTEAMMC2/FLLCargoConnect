# This file contains the instructions for solution 1

def solution_1():
  # Do awesome stuff!
  line_follow_until_color(
    line_sensor_right = ColorSensor(Port.E), 
    line_sensor_left = ColorSensor(Port.F), 
    drive_speed = 150,
    proportional_gain = 1.2,
    stop_color=Color.NONE)

  robot.turn(20)
  robot.drive(400,0)
  wait(500)
  # Now the robot is in front of the generator.  
  robot.drive(-500,0)
  wait(1200)
  robot.turn(-90)
  # Now the robot is in front of the back of the plane.
  robot.turn(-120)
  robot.drive(500,0)
  wait(1250)
  # Now the robot is in home
