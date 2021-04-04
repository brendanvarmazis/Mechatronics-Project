#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
leftwheel = Motor(Port.A)
rightwheel = Motor(Port.D)
gyro = GyroSensor(Port.S1)
ultrasonic = UltrasonicSensor(Port.S4)
robot = DriveBase(leftwheel, rightwheel,wheel_diameter= 55.5 , axle_track= 104)
colorsensor=ColorSensor(Port.S3)

last_Turn = 'left'#robot will turn right when it first reaches the boundary
left_Side='clear'
right_Side='clear'
front = 'clear'

def turn_Right():
    current_Angle = gyro.angle()
    while gyro.angle()<current_Angle + 90:
        position = (current_Angle + 90 - gyro.angle())*0.5 
        robot.turn(position)

def turn_Left():
    current_Angle = gyro.angle()
    while gyro.angle()> current_Angle -90:
        position = (current_Angle -90 - gyro.angle())*0.5
        robot.turn(position)    


        

def go_Straight():
    robot.drive(75,0)

    while gyro.angle()>0 and gyro.angle()<4 or gyro.angle()>-90 and gyro.angle()<-86:
        robot.drive(75,-10)
        if gyro.angle()==0:
            gyro.reset_angle(0)

    while gyro.angle()<0 and gyro.angle()>-4 or gyro.angle() < 90 and gyro.angle()>86:
        robot.drive(75,10)
        if gyro.angle()==0:
            gyro.reset_angle(0)


    if ultrasonic.distance() < 200:
        robot.stop()
        leftwheel.brake()
        rightwheel.brake() #robot stops 
        count = 0 #sets count equal to zero
        left_Side = 'clear' #left side is marked as clear
        right_Side = 'clear' # right side is marked as clear
        front = 'clear' #front is marked as clear


        turn_Left() #checking to see if the left side is clear
        while count < 17:
            robot.straight(10)
            count = count + 1
            if colorsensor.color()==Color.GREEN or ultrasonic.distance()<10: #has reached the border or path is blockd by an object
             robot.straight(-count*10) #moves back to where it was
             turn_Right
             left_Side = 'blocked'
             count=21

        if left_Side == 'clear' and front == 'clear':
            count = 0
            turn_Right()
            while count < (23 + 25):
                robot.straight(10)
                count = count + 1
                if colorsensor.color()==Color.GREEN or ultrasonic.distance()<100: #makes sure front is clear
                    robot.straight(-count*10)
                    turn_Left()
                    robot.straight(-170)
                    turn_Right()
                    front = "'blocked"
                    count = 1000
            turn_Right()
            if ultrasonic.distance() < 150:
                turn_Left()
                robot.straight(200)
                turn_Right()
                robot.straight(170)
                turn_Left
            turn_Left() 
        
        if left_Side == 'blocked':
            robot.stop()
            leftwheel.brake()
            rightwheel.brake() #robot stops              
            
        
            turn_Right()
            count = 0
            while count < 17:
                robot.straight(10)
                count = count + 1
                if colorsensor.color()==Color.GREEN or ultrasonic.distance()<100: #has reached the border or path is blockd by an object
                    robot.straight(-count*10) #moves back to where it was
                    right_Side = 'blocked'
                    count = 19

            if right_Side == 'clear' and front == 'clear':
                count = 0
                turn_Left()
                while count < (23 + 25):
                    robot.straight(10)
                    count = count + 1
                    if colorsensor.color()==Color.GREEN or ultrasonic.distance()<100: #makes sure front is clear
                        robot.straight(-count*10)
                        turn_Right()
                        robot.straight(-170)
                        turn_Left()
                        front = "'blocked"
                        count = 1000
                turn_Left()
                if ultrasonic.distance() < 150:
                    turn_Right()
                    robot.straight(200)
                    turn_Left()
                    robot.straight(170)
                    turn_Right()

        #if left_Side == 'blocked' and right_Side = 'blocked'


while True:
  

    gyro.reset_angle(0)
    
    go_Straight() 

    
    if colorsensor.color()==Color.WHITE: 
        robot.stop() #robot stops
        leftwheel.brake()
        rightwheel.brake()
        if last_Turn == 'left' and colorsensor.color()==Color.WHITE :
            robot.straight(-50) #backs up 50mm
            turn_Right() #turns right
            robot.straight(60)#moves forward 20 mm
            turn_Right() #turns right
            last_Turn = 'right'
        if last_Turn=='right' and colorsensor.color()==Color.WHITE:
            robot.straight(-50) #backs up 50mm
            turn_Left() #turns left
            robot.straight(60)#moves forward 20 mm
            turn_Left() #turns left
            last_Turn = 'left' 
       







