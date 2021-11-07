# 10/28/2021 edit by eric mccague - cleaned old test code and added comments
#    
# Call libraries 
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# define robot defualts
wheel_diameter = 56
axle_track = 126

# Mission Module deff
# run "truck" side arms gear and motor define
bottom_motor_mission_1 = Motor(
        Port.A,
        positive_direction=Direction.CLOCKWISE, 
        gears=[[12,20],[20,12]], 
        reset_angle=True)

# run "truck" forklift gear and motor define
top_motor_mission_1 = Motor(
        Port.B,
        positive_direction=Direction.CLOCKWISE, 
        gears=[[12,20]], 
        reset_angle=True)

# Initialize the hub.
hub = PrimeHub()

#defines what button leave the code
#hub.system.set_start_button(Button.CENTER)
hub.system.set_stop_button(Button.BLUETOOTH)

# Define Robot defaults
# define basic motor defaults
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D)
bottom_motor = Motor(Port.A)
top_motor = Motor(Port.B)
line_sensor_right = ColorSensor(Port.F)
line_sensor_left = ColorSensor(Port.E)

# Use your measurements to override the default colors, or add new colors:
Color.WHITE = Color(h=0, s=0, v=100)  # pure wite line
Color.BLACK = Color(h=216, s=20, v=0)   # pure black line
Color.YELLOW = Color(h=56, s=46, v=83)    # finsh line
Color.BLUE = Color(h=208, s=71, v=87)     # blue by cargo area
Color.GRAY = Color(h=225, s=9, v=96)  # grey 
Color.RED = Color(h=350, s=90, v=95)  # red
Color.GREEN = Color(h=105, s=64, v=72)   # Green
Color.ORANGE = Color(h=5, s=75, v=96)  # orange
Color.MAGENTA = Color(h=328, s=71, v=60) # magenta 

# Put your colors in a list or tuple.
my_colors = (Color.WHITE, Color.BLACK, Color.YELLOW, Color.BLUE, Color.GRAY, Color.RED, Color.ORANGE, Color.GREEN, Color.MAGENTA, Color.NONE)

# Save your colors.
line_sensor_right.detectable_colors(my_colors)
line_sensor_left.detectable_colors(my_colors)

# defines
bottom_motor_speed = 100
top_motor_speed = 10
 
# set inital value for DriveBase class
straight_speed = 1000
straight_acceleration = 540
turn_rate = 390
turn_acceleration =  250
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
# this function moves robot to push blue box out of mat and back up to home

# Display a line on the robot screen to show that the program started 
#hub.display.text('A')
hub.display.pixel(0, 0, brightness=20)
hub.display.pixel(0, 1, brightness=40)
hub.display.pixel(0, 2, brightness=60)
hub.display.pixel(0, 3, brightness=80)
hub.display.pixel(0, 4, brightness=100)
wait(1000)

# bootloader function
# This function lets the user select which run number to command
# arrors rigth are up and left are down
# center button runs program
def monitor():
  solution = 1
  NumberOfSolutions = 6  # how many numbers you want to show in the boot loader
  hub.display.number(solution)  # show the number on the screen
  hub.speaker.beep(frequency=200, duration=50) # beep to say unit is ready
  hub.speaker.beep(frequency=400, duration=50)
  hub.speaker.beep(frequency=600, duration=50)
  hub.speaker.beep(frequency=800, duration=50)
  hub.speaker.beep(frequency=1000, duration=50)

  while True:
    pressed = [] # nothing is presses to the program waits
    while not any(pressed):
      pressed = hub.buttons.pressed()
      wait(10)
    
    # Wait for all buttons to be released.
    while any(hub.buttons.pressed()):
      wait(10)
    
    # Increment selection number based on button pressed.
    if Button.LEFT in pressed: # check if the left button is pressed
        hub.speaker.beep(frequency = 100, duration= 100)
        solution += -1 # If the left button was pressed, decrement the solution number by 1

        if solution < 1: # if the solution number is less than 0, wrap the number around to the max number. This way only numbers 0 to NumberOfSolutions (initially 15) is used.
            solution = NumberOfSolutions # Set solution number to the max number
        hub.display.number(solution) # Display the solution number on the PrimeHub
            #pressed.clear() # clear the array storing the button press to keep everything tidy.
    elif Button.RIGHT in pressed:
        hub.speaker.beep(frequency = 300, duration= 100)
        solution += 1
        if solution > NumberOfSolutions:
          solution = 1
        hub.display.number(solution)
        #Button.pressed.clear()
    elif Button.CENTER in pressed:
    # If center button is pressed, run the selected mission
      # Python switch statement syntax
      if solution == 1: # run 1
        hub.speaker.beep(frequency=300, duration=100)
        wait(500)
        hub.light.on(Color.RED)
        #  Run 1  # airplane missions
        #mission_4()
        airplane()
        #solution_2() #corresponded with 3 instead of 2
        #mission_2()  # we don't know what this is yet....
        robot.stop()
        hub.speaker.beep(frequency = 1000, duration= 100)
        hub.speaker.beep(frequency = 500, duration= 100)
        hub.speaker.beep(frequency = 100, duration= 100)
        hub.light.on(Color.BLUE)
        solution += 1
        hub.display.number(solution)
      elif solution == 2: # run solution 2
        hub.speaker.beep(frequency=300, duration=100)
        wait(500)
        hub.light.on(Color.RED)
        #  Run 2
        
        # solves Chicken, large cargo and Home delivery
        Chicken_run()
        
        robot.stop()
        hub.speaker.beep(frequency = 1000, duration= 100)
        hub.speaker.beep(frequency = 500, duration= 100)
        hub.speaker.beep(frequency = 100, duration= 100)
        hub.light.on(Color.BLUE)
        solution += 1
        hub.display.number(solution)
      elif solution == 3:

        hub.speaker.beep(frequency=300, duration=100)
        wait(500)
        hub.light.on(Color.RED)
        #  Run 3
        mission_truck()  # this run starts with the trucks but is long and does many other missions
        mission_train()   # this contunes that run - broken appart for easy debug
        robot.stop()
        hub.speaker.beep(frequency = 1000, duration= 100)
        hub.speaker.beep(frequency = 500, duration= 100)
        hub.speaker.beep(frequency = 100, duration= 100)
        hub.light.on(Color.BLUE)
        solution += 1
        hub.display.number(solution)
      elif solution == 4:
       # solution_4()
        hub.speaker.beep(frequency=900, duration=100)
        hub.light.blink(Color.CYAN, [500, 500])
        #wait(2000)
        hub.light.on(Color.BLUE)
        mission_4()
      elif solution == 5:
        hub.speaker.beep(frequency=300, duration=100)
        wait(500)
        hub.light.on(Color.RED)
        #  Run 2 module reset after run 
        top_motor_mission_3 = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=[[12,24],[1,24]], reset_angle=True)
        top_motor_mission_3.reset_angle(0)
        top_motor_mission_3.run_target(1500, 190)
        run_test() 
        robot.stop()
        hub.speaker.beep(frequency = 1000, duration= 100)
        hub.speaker.beep(frequency = 500, duration= 100)
        hub.speaker.beep(frequency = 100, duration= 100)
        hub.light.on(Color.BLUE)
         
      elif solution == 6:

        hub.speaker.beep(frequency=300, duration=100)
        wait(500)
        hub.light.on(Color.RED)
        #  Read color scensor 
        read_colors()
        robot.stop()
        hub.speaker.beep(frequency = 1000, duration= 100)
        hub.speaker.beep(frequency = 500, duration= 100)
        hub.speaker.beep(frequency = 100, duration= 100)
        hub.light.on(Color.BLUE)
              
      else :
        print("lost!")
        # default solution, what to do if a solution number is not implemented above?


# Function to follow line with color sensor and stop when the other color senor
# sees a target color
def pid_line_follow(sensor_to_track, side_of_line, drive_speed, critical_gain, critical_period, stop_color):
    # sensor_to_track - need to be left or right'
    # side_of_line - need to be 'left or right'

    # Calculate the light threshold. Choose values based on your measurements.
    _black = 10
    _white = 95
    target = (_black + _white) / 2
    # PID contol for line follwing
    # Tune rate is in degrees / second

    # Ziegler-nicholes Method Gail Vaules
    # Finding the gains Kp, Ki, and Kd - turning
    # Step 1: Set the Gains Ki and Kd to zero (making it only proportional)
    # Step 2: Start w/ Kp as a SMALL value and increase Kp until the output the robot
    #           starts to oscillate. - this is Kc
    # Step 3: Measure the time period of the oscillation at that Kp value - this is Pc
    # critical_gain = 2   # Kc
    # critical_period = .5   # Pc
    Previous_Error = 0
    Integral = 0
    DeltaT = 1/100
    '''
    # use this for calibration
    PROPORTIONAL_GAIN = 3.5
    INTEGRAL_GAIN = 0
    Derivative_GAIN = 0
    '''
    PROPORTIONAL_GAIN = 0.6 * critical_gain  # Kp
    INTEGRAL_GAIN = 2 * PROPORTIONAL_GAIN / critical_period # Ki
    Derivative_GAIN = PROPORTIONAL_GAIN * critical_period / 8  #Kd
    
    if sensor_to_track == "right":
        print("sensor to track is right ")
        watch = line_sensor_left
        track = line_sensor_right
    else :
        print("sensor to track is left ")
        watch = line_sensor_right
        track = line_sensor_left      

    if side_of_line == "right":
        print("side of line is right")
        direction = -1
    else :
        print("side of line is left")
        direction = 1
    count = 0

    print(stop_color)
    while watch.color() != stop_color:

        print(watch.color())
        Error = track.reflection() - target
        Integral = Integral + (Error * DeltaT)
        Derivative = (Error - Previous_Error) /DeltaT
        Previous_Error = Error

        # Calculate the turn rate.
        p = PROPORTIONAL_GAIN * Error
        i = INTEGRAL_GAIN * Error
        d = Derivative_GAIN * Error

        turn_rate = (p + i + d) * direction
        print(turn_rate)
        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)
        count = count +1
        #print(count, turn_rate, Error, line_sensor_right.reflection())

# Function has robot follow the line for a target distance (in mm)
def pid_line_follow_dist(sensor_to_track, side_of_line, drive_speed, critical_gain, critical_period, stop_distance):
    # sensor_to_track - need to be left or right'
    # side_of_line - need to be 'left or right'

    # Calculate the light threshold. Choose values based on your measurements.
    _black = 10
    _white = 95
    target = (_black + _white) / 2
    # PID contol for line follwing
    # Tune rate is in degrees / second

    # Ziegler-nicholes Method Gail Vaules
    # Finding the gains Kp, Ki, and Kd - turning
    # Step 1: Set the Gains Ki and Kd to zero (making it only proportional)
    # Step 2: Start w/ Kp as a SMALL value and increase Kp until the output the robot
    #           starts to oscillate. - this is Kc
    # Step 3: Measure the time period of the oscillation at that Kp value - this is Pc
    # critical_gain = 2   # Kc
    # critical_period = .5   # Pc
    Previous_Error = 0
    Integral = 0
    DeltaT = 1/100
    '''
    # use this for calibration
    PROPORTIONAL_GAIN = 3.5
    INTEGRAL_GAIN = 0
    Derivative_GAIN = 0
    '''
    PROPORTIONAL_GAIN = 0.6 * critical_gain  # Kp
    INTEGRAL_GAIN = 2 * PROPORTIONAL_GAIN / critical_period # Ki
    Derivative_GAIN = PROPORTIONAL_GAIN * critical_period / 8  #Kd
    stop_distance = abs(stop_distance)


    if sensor_to_track == "right":
        print("sensor to track is right ")
        watch = line_sensor_left
        track = line_sensor_right
    else :
        print("sensor to track is left ")
        watch = line_sensor_right
        track = line_sensor_left      

    if side_of_line == "right":
        print("side of line is right")
        direction = -1
    else :
        print("side of line is left")
        direction = 1

    count = 0
    robot.reset()
    while robot.distance() <= stop_distance:

        #print(line_sensor_right.reflection(), line_sensor_right.color(), line_sensor_right.hsv())
        Error = track.reflection() - target
        Integral = Integral + (Error * DeltaT)
        Derivative = (Error - Previous_Error) /DeltaT
        Previous_Error = Error

        # Calculate the turn rate.
        p = PROPORTIONAL_GAIN * Error
        i = INTEGRAL_GAIN * Error
        d = Derivative_GAIN * Error

        turn_rate = (p + i + d) * direction

        # Set the drive base speed and turn rate.
        robot.drive(drive_speed, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)
        count = count +1
        #print(count, turn_rate, Error, line_sensor_right.reflection())



def module1_home():
    bottom_motor_speed = -500
    bottom_motor_mission_1.run_until_stalled(bottom_motor_speed, then=Stop.COAST, duty_limit=None)
    print("home finished")

def straight_until_color(sensor_to_track, turn_rate, drive_speed, stop_color):
    if sensor_to_track == "right":
        print("sensor to track is right ")
        watch = line_sensor_right
    else :
        print("sensor to track is left ")
        watch = line_sensor_left
    while watch.color() != stop_color:
        robot.drive(drive_speed, turn_rate)
        wait(10)
    robot.stop()

def straight_until_color_time(sensor_to_track, turn_rate, drive_speed, stop_color, timer):
    if sensor_to_track == "right":
        print("sensor to track is right ")
        watch = line_sensor_right
    else :
        print("sensor to track is left ")
        watch = line_sensor_left
    elapsed_time = 0
    while (watch.color() != stop_color):
        robot.drive(drive_speed, turn_rate)
        elapsed_time += 10
        wait(10)
        if (elapsed_time > timer):
          break
        print(elapsed_time < timer)
        print(watch.color())
        print(elapsed_time)
    robot.stop()
    
def mission_truck():
    robot.reset()
    top_motor_mission_1 = Motor(
        Port.B,
        positive_direction=Direction.CLOCKWISE, 
        gears=[[12,20]], 
        reset_angle=True)    
    top_motor_mission_1.reset_angle(0)
    bottom_motor_mission_1.reset_angle(0)  # sets side arms to zero (should be down on truck)
    bottom_motor_mission_1.run_target(200,-20) # was 80, -10  this wasn't working so low
    #straight_speed = 900
    #robot.reset()
    robot.straight(165+20) #drives out of home  was 165 added more to get onto the line
    #wait_until_button() #debug command


    pid_line_follow(      # this takes you to first black line
        sensor_to_track = "right", 
        side_of_line = "left",
       drive_speed = 150, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button() 

    pid_line_follow_dist(     # This takes until trucks both latch
      sensor_to_track = "right", 
      side_of_line = "left",
      drive_speed = 150, 
      critical_gain = 1, 
      critical_period = 10, 
      stop_distance = 450-20-240+180)
    
    robot.stop()
    #wait_until_button() #debug command

    #robot.straight(180)  # was 160 # moved this into line follow above
    robot.straight(-10) #  back off a sec to you can remove arm from truck
    
    #this lifts the side arms up
    bottom_motor_mission_1.run_target(300,90) #Was (80, 40) but was hitting trucks
    #wait_until_button()
    #knock-down first part of bridge
    #robot.straight(140)
    #pid_line_follow(sensor_to_track = "right", side_of_line = "left", drive_speed = 100, critical_gain = 1, 
    #    critical_period = 10, stop_color = Color.BLACK)

    pid_line_follow_dist(
      sensor_to_track = "right", 
      side_of_line = "left",
      drive_speed = 150, 
      critical_gain = 1, 
      critical_period = 10, 
      stop_distance = 260+20-120)
    robot.stop()
    #wait_until_button() # debug wait - check possition
    robot.straight(130) #drives normal to pass black line in table (was causing error)
    bottom_motor_mission_1.run_target(80,45) #Goes down to hit first bridge (but not trucks)
    #wait_until_button()

    #robot.straight(200) # was 400
    #below command tasks you to the second bridge
    pid_line_follow(      # this takes you to the midel of the briges
        sensor_to_track = "right", 
        side_of_line = "left",
        drive_speed = 100, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.straight(80)
    '''
    pid_line_follow_dist(
      sensor_to_track = "right", 
      side_of_line = "left",
      drive_speed = 150, 
      critical_gain = 1, 
      critical_period = 10, 
      stop_distance = 200-30)
      '''
    robot.stop()
     #debug command

    bottom_motor_mission_1.run_target(80,90) #raises arm to clear drawbridge
    #wait_until_button()

    robot.straight(130-10) #passes bridge
    #wait_until_button()

    bottom_motor_mission_1.run_target(80,45) #lowers arm to hit bridge
    #wait_until_button()
    #robot.straight(-80)
    #wait_until_button()

    robot.straight(-40)
    #wait_until_button()

    robot.turn(60)# turns into cargo connect circle
    top_motor_mission_1.reset_angle(0)
    top_motor_mission_1.run_target(80,90)

    robot.turn(-60)
    bottom_motor_mission_1.run_target(80,90)
    robot.straight(430)
    straight_until_color(
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 75, 
      stop_color = Color.BLACK)

    #backup slightly so the sensor is on the line
    #robot.straight(-10)
    robot.turn(90)

    

    '''
    #bottom_motor_mission_1.run_target(80,90)
    robot.straight(-50) #clearing circle #100
    #wait_until_button()
    bottom_motor_mission_1.run_target(80,0)

    robot.reset()

    robot.turn(20) #was 155
    #wait_until_button()
    robot.drive(-200,0)#crash in to cargo dock mission to wall square
    wait(1000)
    robot.straight(20-10) #was going to far away from the cargo dock
    #wait_until_button()
    robot.turn(80) # solves mission 7 lift cargo on dock
    
    robot.straight(-350)
    robot.turn(-80)
    robot.drive(-300,0) #crash in to cargo dock mission to wall square
    wait(1500)
    robot.stop()
    #top_motor_mission_1.reset_angle(0)
    #top_motor_mission_1.run_target(100,45)
    # stop here - continues in next function for debuging
    '''
def mission_train():
    '''
    robot.straight(60) #  Get closer to the line 
    bottom_motor_mission_1.run_target(100,75)  # move side arms up to clear train
    straight_until_color( # looks for white line
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 150, 
      #stop_color = Color.WHITE)
      stop_color = Color(h=0, s=0, v=100))

    #wait_until_button()
    straight_until_color(  #then looks for black line 
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.BLACK)
    #wait_until_button()
    robot.turn(-30) #turns to face target line
    robot.straight(100)  # drives over the odd colors on the table
    #wait_until_button()

    #robot.straight(-150)
    #robot.turn(-30)

    #bottom_motor_mission_1.reset_angle(0)
  
    
    straight_until_color(  # now look for white (this is the line follow)
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.WHITE)
    robot.stop()

    #wait_until_button()
    straight_until_color(  # drive to black line
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()
    
    #wait_until_button()
    straight_until_color(  # drive to black line
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 100, 
      stop_color = Color.WHITE)
    #robot.straight(40) # over shoot the black to make sure you are on correct side
    #wait_until_button()
    robot.turn(20+10) # turn into the line more to help PID 
    #wait_until_button()

    ### May want to remove this ###
    #top_motor_mission_1.run_target(200,90)   # moves the arm up out of the way.
    #top_motor_mission_1.run_target(100,0)  # lower fork lift if you want to grab cargo , later...
    ### // ###
    '''
    pid_line_follow_dist(  # line follow for alignment
      sensor_to_track = "left", 
      side_of_line = "left",
      drive_speed = 150, 
      critical_gain = 1, 
      critical_period = 10, 
      stop_distance = 200)
    

    pid_line_follow(      # stop on white (so you don't crash into the mission)
        sensor_to_track = "left", 
        side_of_line = "left",
        drive_speed = 100, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.WHITE)
    robot.stop()
    #wait_until_button()
    ################################################################################
    #  # go really slow to see black ()
    # if you get stuck it times out.
    straight_until_color_time( 
      sensor_to_track = "right", 
      turn_rate = 0,
      drive_speed = 75, 
      stop_color = Color.BLACK,
      timer = 1000)
    robot.stop()
    #wait_until_button()
    ### May want to remove this ### '''
    #top_motor_mission_1.run_target(200,90)  # picks up cargo , not going to try this now. 
    #robot.straight(5)
    #wait_until_button()
    robot.straight(-200)   # backs up to exact spot to lower arm for fix train track
    #wait_until_button()
    bottom_motor_mission_1.run_target(100,0)  # lower arms to hit track
    #wait_until_button()
    robot.straight(75)  # push into the track to knock it down
    #wait_until_button()
    robot.straight(-50)  # get out of the way so it can fall
    #wait_until_button()
    bottom_motor_mission_1.run_target(300,65)   # get arms back up and away from train
    #wait_until_button()
    robot.straight(-320+30)   #  back up to push front of train      was 20!!!
    #wait_until_button()
    robot.drive(-25,0) #crash into helicopter 
    wait(500)
    robot.turn(-5)
    #wait_until_button()
    bottom_motor_mission_1.run_target(100,0) # slow arm to push train
    #wait_until_button()

    pid_line_follow_dist(  # line follow to push train until latched
      sensor_to_track = "left", 
      side_of_line = "left",
      drive_speed = 150, 
      critical_gain = 1, 
      critical_period = 10, 
      stop_distance = 430)
    robot.turn(-45)
    robot.straight(-370) # back away from cargo area
    robot.turn(135)  # face line
    
    #wait_until_button()  This is looking for the line 
    straight_until_color(
      sensor_to_track = "right", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()
    straight_until_color(
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 75, 
      stop_color = Color.WHITE)
    robot.stop()
    #wait_until_button()
    straight_until_color(
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 75, 
      stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()
    '''
    straight_until_color(
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 75, 
      stop_color = Color.WHITE)
    robot.stop
    '''
    #wait_until_button()
    pid_line_follow(      
        sensor_to_track = "left", 
        side_of_line = "left",
        drive_speed = 100, 
        critical_gain = 1.5, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()  #We are back in the middle of the bridges 

    bottom_motor_mission_1.run_target(300,90)
    robot.straight(-30-80+20)
    #wait_until_button()
    robot.turn(45+5)
    #wait_until_button()
    robot.straight(200)
    #wait_until_button()
    straight_until_color(
      sensor_to_track = "right", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.WHITE)
    robot.stop()
    #wait_until_button()
    straight_until_color(
      sensor_to_track = "right", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()
    robot.turn(-60)
    #wait_until_button()
    pid_line_follow(      # finding cross 
        sensor_to_track = "right", 
        side_of_line = "left",
        drive_speed = 100, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()  #Ready to back into the crane
    bottom_motor_mission_1.run_target(300,0) 
    robot.straight(-240)
    #wait_until_button()
    robot.straight(100)
    #wait_until_button()
    pid_line_follow(      # Line square after crane
        sensor_to_track = "right", 
        side_of_line = "left",
        drive_speed = 100, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.stop()
    #wait_until_button()

    robot.turn(45)
    #wait_until_button()
    
    bottom_motor_mission_1.run_target(500,90)
    #wait_until_button()

    robot.straight(200)
    #wait_until_button()

    straight_until_color(
      sensor_to_track = "left", 
      turn_rate = 0,
      drive_speed = 150, 
      stop_color = Color.WHITE)
    robot.stop()
    #wait_until_button()

    robot.turn(-53+5) #  turn tords axident avoidence
    #wait_until_button()

    straight_until_color(
      sensor_to_track = "right", 
      turn_rate = 0,
      drive_speed = 50, 
      stop_color = Color.BLACK)
    #wait_until_button()
    straight_speed = 25
    robot.settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
    robot.straight(55)
    robot.stop() # QED

    #wait_until_button()
    # puts this back to defalt 
    straight_speed = 1000 
    robot.settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
    robot.stop()
    #drives up to blue lines carefully, searching for correct colors
 

def airplane(): #  this solves the airplain run 
  top_motor_airplane = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=[[12,24],[1,24]], reset_angle=True)

  top_motor_airplane.reset_angle(0)
  robot.reset()
  
  robot.straight(80+25+140)     #was 280
  robot.turn(-45)
  robot.turn(45)
  pid_line_follow(
        sensor_to_track = "right", 
        side_of_line = "right",
        drive_speed = 150, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
  robot.stop()
  robot.straight(40)
  top_motor_airplane.run_target(200, 65.00000000000000000000000000000001, then=Stop.HOLD, wait=True) # was 800
  robot.straight(-203+30)
  robot.turn(-85.00001)
  robot.straight(-65)
  top_motor_airplane.run_target(200, 0, then=Stop.HOLD, wait=True)   # was 1000
  robot.straight(50)
  top_motor_airplane.run_target(200, -10, then=Stop.HOLD, wait=True)   # was 1000
  robot.straight(-130)
  robot.turn(30)
  top_motor_airplane.run_target(200, 5, then=Stop.HOLD, wait=True)  # was 800
  robot.straight(75)
  robot.turn(-30)
  robot.straight(-110)
  robot.turn(30)
  robot.straight(-110)
  robot.turn(-90)
  robot.straight(500+100)

def Chicken_run():  # this looks like correct run #2 code full. 
    #Robot starts at fixture with cargo and positioned at 45 degrees (450)
    robot.reset() 
    robot.straight(150)
    robot.turn(45)
    robot.straight(540)
    robot.turn(-25)
    #wait_until_button()
    pid_line_follow(
        sensor_to_track = "right", 
        side_of_line = "left",
        drive_speed = 125, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK) 
    robot.stop()

    robot.turn(-70)
    #wait_until_button()
    pid_line_follow_dist(
        sensor_to_track = "right", 
        side_of_line = "right",
        drive_speed = 150, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_distance = 300)
    pid_line_follow(
        sensor_to_track = "right", 
        side_of_line = "right",
        drive_speed = 150, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_color = Color.BLACK)
    robot.stop()
    robot.turn(40)
    robot.straight(60)
    robot.turn(-40)
    robot.straight(85)
    # drops package
    robot.straight(-85)
    robot.turn(40)
    robot.straight(-60)
    robot.turn(-40)
    robot.straight(-30)
    robot.turn(-150)
    pid_line_follow_dist(
        sensor_to_track = "left", 
        side_of_line = "right",
        drive_speed = 100, 
        critical_gain = 1, 
        critical_period = 10, 
        stop_distance = 150)
    robot.stop()
    robot.turn(-85)
    robot.stop()

    top_motor_chicken = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=[[12,24],[1,24]], reset_angle=True)
    top_motor_chicken.reset_angle(0)
    top_motor_chicken.run_target(1000, -80)
    robot.straight(-70)
    robot.turn(90)
    robot.straight(1500)


def wait_until_button():
    pressed = []
    robot.stop()
    while not any(pressed):
      pressed = hub.buttons.pressed()
      wait(10)

def read_colors():
    pressed = []
    robot.stop()
    while not any(pressed):
      pressed = hub.buttons.pressed()
      #print("right " & line_sensor_right.color() & " " & line_sensor_right.hsv() )
      print("right ")
      print(line_sensor_right.color())
      print(line_sensor_right.hsv() )
      wait(500)

monitor()
