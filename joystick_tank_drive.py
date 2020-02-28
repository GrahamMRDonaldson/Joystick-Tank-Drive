import os
import pprint
import pygame

#for the pin input/output
from gpiozero import PWMLED
from signal import pause
from time import sleep
from gpiozero import Servo

#class for controlling pitch servos (TURRET)
class ServoGPIO:
    def __init__(self, PIN):    
        self.servo = Servo(PIN) #defining servo pin for GPIO output on PI
        self.degree = 0         #vertical degree (0 to 90)
        self.value = -1         #value of servo (-1 to 1)
   
    def set_degrees(self, degree):  #FUNCTION: for setting the servo to a certain degree (0 to 90)
        self.degree = degree        #sets the value of classes degree to inputed degree
        self.value = degrees / 90   #gets a value from 0 - 90 and turns it to 0 - 1
        self.value *= 2             #sets the degree ratio to a range from 0 to 2
        self.value -= 1             #sets the degree ratio to a range from -1 to 1
        self.servo.value = value    #setting the servo value to the value
        
    def set_value(self, value):     #FUNCTION: for setting the servo to a certain value (-1 to 1)
        self.value = value          #sets the value of classes value to inputed value
        self.degree = value + 1     #sets degree to a value from 0 to 2
        self.degree /= 2            #sets degree to a value from 0 to 1
        self.degree *= 90           #sets degree to a value from 0 to 90
        self.servo.value = value    #setting the servo value to the value
        
    #JUST FOR DEGREES, NOT FOR VALUE
    def __add__(self, other):           #FUNCTION: for iterating the servos degree
        self.degree += other            #sets the value of classes value to itself + inputed value
        self.value = self.degrees / 90  #gets a value from 0 - 90 and turns it to 0 - 1
        self.value *= 2                 #sets the degree ratio to a range from 0 to 2
        self.value -= 1                 #sets the degree ratio to a range from -1 to 1
        self.servo.value = value        #setting the servo value to the value
    
    def __sub__(self, value):           #FUNCTION: for iterating the servos degree
        self.degree -= other            #sets the value of classes value to itself + inputed value
        self.value = self.degrees / 90  #gets a value from 0 - 90 and turns it to 0 - 1
        self.value *= 2                 #sets the degree ratio to a range from 0 to 2
        self.value -= 1                 #sets the degree ratio to a range from -1 to 1
        self.servo.value = value        #setting the servo value to the value
        
    def debug(self):                              #FUNCTION: for printing out MotorDrivetrain values for debugging perposes
        print("self.horizontal:",self.horizontal) #prints out the horizontal value
        print("self.vertical:",self.vertical)     #prints out the vertical value
        
#class for controlling the left/right. AKA drivetrain
class MotorDrivetrain:
    def __init__(self):
        self.horizontal = 0   #horizontal joystick value (between -1 and 1) 
        self.vertical = 0     #vertical joystick value (between -1 and 1) 
    
    def set_horizontal(self, val):  #FUNCTION: for updating the horizontal value with the left joystick
        self.horizontal = val #sets the value of horizontal to the horizontal joystick value
    
    def set_vertical(self, val): #FUNCTION: for updating the vertical value with the left joystick
        self.vertical = val   #sets the value of vertical to the vertical joystick value
    
    def calculate(self): #FUNCTION: for calculating the left and right values of the drive train
        #http://home.kendra.com/mauser/joystick.html - used this algorithm for joystick control
        v = (1-abs(self.horizontal)) * (self.vertical/1) + self.vertical
        w = (1-abs(self.vertical)) * (self.horizontal/1) + self.horizontal
        l = (v-w)/-2
        r = (v+w)/-2
        return l,r
    
    def debug(self):                              #FUNCTION: for printing out MotorDrivetrain values for debugging perposes
        print("self.horizontal:",self.horizontal) #prints out the horizontal value
        print("self.vertical:",self.vertical)     #prints out the vertical value
        
#class for handling a single motor
class MotorGPIO:
    def __init__(self, forwardPin, backwardPin):
        self.forwardPin = PWMLED(forwardPin)   #defining forward direction pin for GPIO output on PI
        self.backwardPin = PWMLED(backwardPin) #defining backward direction pin for GPIO output on PI
        self.forwardPin.value = 0              #setting the motors initial speed value to 0 (not spinning)
        self.backwardPin.value = 0             #setting the motors initial speed value to 0 (not spinning)
    
    def move(self, value):                         #FUNCTION: for changing the values of the motors ("moving"/turning them)
        if(value >= 0):                            #if the motor is supposed to turn forward
            if(self.backwardPin.value != 0):       #if the motor was turning backward
                self.backwardPin.value = 0         #set the pin for turning backward to 0
            self.forwardPin.value = value          #set the pin for turning forward to given value
        else:                                      #if the motor isn't supposed to turn forward (ie turning backward)
            if(self.forwardPin.value != 0):        #if the motor was turning forward
                self.forwardPin.value = 0          #set the pin for turning forward to 0
            self.backwardPin.value = abs(value)    #set the pin for turning backward to given value
            
    def debug(self):                                       #FUNCTION: for printing out motor values for debugging perposes
        print("forwardPin.value:",self.forwardPin.value)   #prints out the forward pin value
        print("backwardPin.value:",self.backwardPin.value) #prints out the backward pin value

#defining motor controller pins
motor_FL = MotorGPIO('GPIO25','GPIO8') #front left motor
motor_FR = MotorGPIO('GPIO7','GPIO1') #front right motor
motor_BL = MotorGPIO('GPIO12','GPIO16') #back left motor
motor_BR = MotorGPIO('GPIO20','GPIO21') #back right motor

#drive train
DT = MotorDrivetrain()

#defining the servo controller pins
servo_L = ServoGPIO('GPIO24')
servo_R = ServoGPIO('GPIO23')

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""
    controller = None
    axis_data = None
    button_data = None
    hat_data = None
    

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                for key, val in self.axis_data.items():
                    if(key==0): #LEFT AND RIGHT -LEFT JOYSTICK
                        DT.set_horizontal(val)
                    if(key==1): #UP AND DOWN -LEFT JOYSTICK
                        DT.set_vertical(val)
                        l,r = DT.calculate()
                        
                        motor_FL.move(l)
                        motor_BL.move(l)
                        motor_FR.move(r)
                        motor_BR.move(r)
 
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()
