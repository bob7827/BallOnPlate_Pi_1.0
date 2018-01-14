#!/usr/bin/python

# These three packages are standard in Python
import smbus, time
from math import sin, cos
import os

# This doesn't work on Pi
# I had hoped it would clear the shell
os.system('clr')

count_debug = 0
expn0_debug = 90.0
expn1_debug = 90.0

# This counter is used to change the patterns.
# Let's start at zero.
# In previous code, I set this to some higher number to test
# one specific pattern
p_count = 0

# These are some globals I used to estimate center of plate,
# do some debugging and do a statistical measurement of
# error variance.
N=0
X0i=2000.0
X1i=2000.0
SumX0=0.0
SumX1=0.0
U0=2000.0
U1=2000.0
SumSqX0 = 0.0
SumSqX1 = 0.0


# The touch screen controller samples the resitive touchscreen
# and relays to the Pi the coordinates.
# In theory, with 12 bit resolution, the screen is 4096 x 4096
# The controller uses I2C to communicate with Pi.  This is common.
# Many devices can be hooked up to a single I2C connection.
# They are differentiated by their address.  The address of the
# touchscreen controller is 0x41 (hex, obviously).
# The smbus package has the functions to communicate through I2C
# The I2C is on bus 1 of the Pi (in this case).
# All of this code came from the bus reads and writes that are
# listed in the chip document for the touchscreen controller.
#
# I2C bus set up
time.sleep(0.5)
bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
time.sleep(0.5)
#############################################
# TOUCHSCREEN CONTROLLER
#############################################
addr = 0x41           # I2C address of the PWM chip.
LASTX=0.0
LASTY=0.0

def initTouchScreen():
    time.sleep(0.5)
    bus.write_byte_data(addr, 0x03, 0x02)     # enable the chip
    time.sleep(0.1)
    print("screen up")

    for i in range(65):
        bus.read_byte_data(addr,i)
    
    bus.write_byte_data(addr, 0x04, 0x00)  # turn on clocks
    bus.write_byte_data(addr, 0x40, 0x00|0x01)  # XYZ enable
    bus.write_byte_data(addr, 0x40, 0x0A|0x01)  # touch detect?
    bus.write_byte_data(addr, 0x20, 0x00|(0x6<4))  # 96 clock/conversion
    bus.write_byte_data(addr, 0x21, 0x02)  # ? CTLR2_6_5MHZ
    
    #bus.write_byte_data(addr, 0x41, 0x00|0x20|0x04)  # ? 4 sample|delay 1ms|settle 5ms
    
    bus.write_byte_data(addr, 0x41, 0b10101011)  # ? 4 sample|delay 1ms|settle 5ms
    
    bus.write_byte_data(addr, 0x56, 0x06)  # ?
    bus.write_byte_data(addr, 0x4A, 1)  # ? FIFO
    bus.write_byte_data(addr, 0x4B, 0x01)  # ? FIFO reset
    bus.write_byte_data(addr, 0x4B, 0)  # ? FIFO unreset

    # this controls the max current through the touchscreen
    # I found that 50ma, the max, works best
    bus.write_byte_data(addr, 0x58, 0x01)  # ? drive 50ma
    bus.write_byte_data(addr, 0x0B, 0xFF)  # ? reset all interrupts
    bus.write_byte_data(addr, 0x09, 0x04|0x01)  # ? interrupt pol high enable



def touched():
    return (bus.read_byte_data(addr,0x40) & 0x80)


def bufferEmpty():
    return (bus.read_byte_data(addr,0x4B) & 0x20)


def bufferSize():
    return (bus.read_byte_data(addr,0x4C))


def readData():
    global LASTX, LASTY
    t0 = bus.read_byte_data(addr,0xD7)
    t1 = bus.read_byte_data(addr,0xD7)
    t2 = bus.read_byte_data(addr,0xD7)
    t3 = bus.read_byte_data(addr,0xD7)

    #print(t0, t1, t2, t3)

    # 4 bytes are read
    # however the x and y values are 12-bit
    # therefore, it was necessary to split a byte between
    # x and y
    t=bytearray([t0, t1, t2, t3])


    # 12 bits
    x=bytearray([0,0])
    x=t0
    x<<=4  # peal off 4 bits
    x|=(t[1]>>4)
    x = 1.00*x + 0.00*LASTX
    LASTX = x

    # 12 bits
    y=bytearray([0,0])
    y=((t1 & 0x0F)<<8)
    y|=t2
    y = 1.00*y + 0.00*LASTY
    LASTY = y
    
    # z is 8 bits
    # what is z, you're asking?
    # it's basically pressure on the surface
    z=t3

    
    ##if (bufferEmpty()!=0):
        ##bus.read_byte_data(addr,0xD7)
    
    
    #readX = (bus.read_word_data(addr,0x4D))
    #readY = ((bus.read_word_data(addr,0x4F)<<4)>>4)
    #readZ = bus.read_byte_data(addr,0x51)


    if bufferEmpty():
        bus.write_byte_data(addr, 0x0B, 0xFF)

    # returning a tuple of 3 floats
    # Tuples are great-- I think
    return(x, y, z)



def getPoint():
    ts_point=readData()
    return ts_point



#############################################
# SERVO CONTROLLER
#############################################
# servo controller is also an I2C device, but its
# address is 0x40  

NUMSERVOS = 1   # There are really 2 servos, but numbered beginning at 0
                # I picked a bad variable name, probably MAXSERVO = 1
                # would have been better.  Sorry
            
addr2 = 0x40           # I2C address of the PWM chip.

def initServos():
    bus.write_byte_data(addr2, 0, 0x20)     # enable the chip
    bus.write_byte_data(addr2, 0xfe, 0x1e)  # configure the chip for multi-byte write

    # code to initialize the servos on the controller
    for i in (0,NUMSERVOS):
        bus.write_word_data(addr, 6+(i*4), 0)
        bus.write_word_data(addr, 8+(i*4), 1250)


# servos are pulse width modulated on a 5ms frame.
# pass the servo number (0 or 1) and the servo arm angle.
# I'm using 90 degrees for an arm parallel with horizon
def turnTo(servo, angle):  # 0 to 180
    pulse = int(833+(4.6*angle))
    
    # based on Hitec Servos
    # I tested the Hitec servos
    # I'm using Hitec HS-7955TG servos, a little pricey
    if pulse<800: pulse = 800
    if pulse>2200: pulse =800
    bus.write_word_data(addr2, 8+servo*4, pulse)

def setInitServos():  #leftover stuff
    # this is some old test code...  repeat of initServos()
    for i in (0,NUMSERVOS):
        bus.write_word_data(addr2, 6+(i*4), 0)
        bus.write_word_data(addr2, 8+(i*4), 1250)
        
  
#############################################
# PID CONTROLLER
#############################################
# I hacked this together from some C code, and then
# simplified the math on the integral part of the PID
# controller.
# Lots of globals

AUTOMATIC = True  # old stuff, probably take this out
MANUAL = False    # old stuff
DIRECT = False    # old stuff
REVERSE = True    # sometimes the correction is backwards, depending on setup
P_ON_M = False    # mutually exclusive to P_ON_E
P_ON_E = True     # proportional sum a function of error (actual-setpoint)

# PID computed command for servo
output0 = 0.0
output1 = 0.0

# setpoint = desired position of ball on plate
setpoint0 = 0.0
setpoint1 = 0.0

# actual position of ball on plate
myInput0 = 0.0
myInput1 = 0.0

inAuto = False  # old stuff

# these are the PID controller factors
Kp0 = 0.0  # proportional -- actual, real time 
Ki0 = 0.0  # integral -- think past movement or accumulated error
Kd0 = 0.0  # derivative -- think future movement
pOn0 = 0
pOnE0 = P_ON_E  # adjust proportional based solely on actual error


lastTime0 = time.time()
#print(lastTime0)

# the time interval is 50ms (really it's slightly more due to the
# overhead of the OS and code).
sampleTime0 = 0.050
mySetPoint = 0.0
lastInput0 = 0.0
myOutput0 = 0.0
outMax0 = 180.0
outMin0 = 0.0

Kp1 = 0.0
Ki1 = 0.0
Kd1 = 0.0
pOn1 = 0
pOnE1 = P_ON_E
lastTime1 = time.time()
sampleTime1 = 50
lastInput1 = 0.0
myOutput1 = 0.0
outMax1 = 170.0
outMin1 = 10.0


# 0 PID axis
# around the y axis
# this is the PID for the servo that pivots around y axis
def setUp0():
    global Kp0, Ki0, Kd0, pOn0, pOnE0, lastTime0, sampleTime0
    global mySetPoint0, lastInput0, myOutput0, outMax0, outMin0
    Kp0 = 0.047500  #-0.0475, originally reversed
    Ki0 = 0.000175  #0.0002
    Kd0 = 0.30000  #0.2200
    pOn0 = 0
    pOnE0 = P_ON_E
    lastTime0 = time.time()
    sampleTime0 = 0.040  #0.03750
    mySetPoint0 = 1930.0
    lastInput0 = 0.0
    myOutput0 = 0.0
    outMax0 = 195.0
    outMin0 = 0.0
    
    
def compute0():
    global Kp0, Ki0, Kd0, pOn0, pOnE0, lastTime0, sampleTime0
    global mySetPoint0, lastInput0, myOutput0, outMax0, outMin0
 
    now = time.time()
    #print(lastTime0)
    # just keep looping in main until at least 50ms has passed
    timeChange = (now-lastTime0)
    if (timeChange>sampleTime0):
        input = myInput0
        error = mySetPoint0 - input
        dInput = (input-lastInput0)
        outputSum = 115.0
        
        # integral
        # rectangular integration over one time interval
        # yes, that's crude, but it's simple
        # In many respects, the integral part is not that important.
        # It would be more important in a feedback loop for an autopilot
        # in the altitude hold mode.  Over time the error (like 40 feet off)
        # accumulates and forces a correction.  Apparently, the A300
        # doesn't do much on this part.  Ha
        # If you increase too much Ki0, it'll overshoot and wobble
        # wildly, like the A300 sometimes.
        outputSum += (Ki0 * error)*timeChange/sampleTime0
        
        # proportional on measurement
        # simple-- move the servo proportionally to the error
        if (pOnE0==False):
            outputSum -= Kp0 * dInput

        # yes the follow two if statement should be merged into an
        # if then else
        # I was new to Python and didn't have time to figure out the
        # syntax.
        # Just making sure the PID adjustment don't exceed the limits of
        # the servos.
        if (outputSum>outMax0):
            outputSum = outMax0
            
        if (outputSum<outMin0):
            outputSum = outMin0
            
        # proportional on Error
        output0 = 0.0
        if (pOnE0==True):
            output0 += Kp0 * error
        else:
            output0 = 0.0
            
        # compute remaining PID value
        output0 += outputSum - Kd0 * dInput
        
        if (output0>outMax0):
            output0 = outMax0
            
        if (output0<outMin0):
            output0 = outMin0
        
        myOutput0 = output0
        
        lastInput0 = input
        lastTime0 =now
        
        return True
    else:
        # returns False when the actual time interval is too short
        return False
            
        
        
        
# 0 PID axis
# aroubnd the x axis
def setUp1():
    global Kp1, Ki1, Kd1, pOn1, pOnE1, lastTime1, sampleTime1
    global mySetPoint1, lastInput1, myOutput1, outMax1, outMin1
    Kp1 = 0.04600
    Ki1 = 0.00025
    Kd1 = 0.3200  #0.3000
    pOn1 = 0
    pOnE1 = P_ON_E
    lastTime1 = time.time()
    sampleTime1 = 0.0400  #0.03750
    mySetPoint1 = 2108.0
    lastInput1 = 0.0
    myOutput1 = 0.0
    outMax1 = 195.0
    outMin1 = 0.0
    
    
def compute1():
    global Kp1, Ki1, Kd1, pOn1, pOnE1, lastTime1, sampleTime1
    global mySetPoint1, lastInput1, myOutput1, outMax1, outMin1
 
    now = time.time()
    #print(lastTime1)
    timeChange = (now-lastTime1)
    if (timeChange>sampleTime1):
        input = myInput1
        error = mySetPoint1 - input
        dInput = (input-lastInput1)
        outputSum = 107.0
        # integral
        outputSum += (Ki1 * error)*timeChange/sampleTime1
        
        # proportional on measurement
        if (pOnE1==False):
            outputSum -= Kp1 * dInput
            
        if (outputSum>outMax1):
            outputSum = outMax1
            
        if (outputSum<outMin1):
            outputSum = outMin1
            
        # proportional on Error
        output1 = 0.0
        if (pOnE1==True):
            output1 = Kp1 * error
        else:
            output1 = 0.0
            
        # compute remaining PID value
        output1 += outputSum - Kd1 * dInput
        
        if (output1>outMax1):
            output1 = outMax1
            
        if (output1<outMin1):
            output1 = outMin1
        
        myOutput1 = output1
        
        lastInput1 = input
        lastTime1 =now
        
        return True
    else:
        return False


# center ball 2000,2000 is roughly the center
def updateCenter():
    global mySetPoint0, mySetPoint1
    mySetPoint0 = 2000.0
    mySetPoint1 = 2000.0

# another function I used to translate the center of the plate
# to (0,0).  This made some calculations cleaner.
def updatePosition(x,y):
    global mySetPoint0, mySetPoint1
    mySetPoint0 = x+2000.0
    mySetPoint1 = y+2000.0
    

# ellipse pattern that also reverses
ellipse_angle=0.0
ellipse_direction=1
def updateEllipse():
    global mySetPoint0, mySetPoint1, ellipse_angle, ellipse_direction
    ellipse_angle += 0.042500*ellipse_direction
    x = 1000.0 * cos(ellipse_angle) + 2000.0
    y = 1000.0 * sin(ellipse_angle) + 2000.0
    mySetPoint0 = x
    mySetPoint1 = y
    if ((ellipse_angle>(6.0*2.0*3.1415926535))or(ellipse_angle<0.0)):
##        ellipse_angle = 0.0
        ellipse_direction=-ellipse_direction


# hypotrochoid pattern
# another parametric equation
# It's a heart shape.  Made it for Sherri
hypo_angle=0.0
def updateHypotrochoid():
    global mySetPoint0, mySetPoint1, hypo_angle
    R = 1000.0
    r = 600.0
    d = 900.0
    hypo_angle += 0.042500
    x = (R-r)*cos(hypo_angle)+d*cos((R-r)/r*hypo_angle) + 2000.0
    y = (R-r)*sin(hypo_angle)-d*sin((R-r)/r*hypo_angle) + 2000.0
    mySetPoint0 = x
    mySetPoint1 = y
    #if (hypo_angle>(2.0*3.1415926535)):
        #hypo_angle=0.0
          
#*************************************************
# Main Code
# Ball on Plate PID Controller
# B. Walker, Dec. 2017
#*************************************************
initTouchScreen()
initServos()

setUp0()
setUp1()

while 1:  # like old C code, should have used True, instead of 1

    # it's important to stop the calculations when the ball is off
    # the plate, otherwise the servos go full tilt.
    # I have the servos freeze in their last position when the ball
    # is off.
    if touched():
        while(bufferEmpty()==0):
            #print(bufferSize())
            
            pt=getPoint()
            #print(pt)
            myInput0 = pt[0]-0.0
            myInput1 = pt[1]-0.0
            
            compute0()
            turnTo(0, myOutput0)
            
            compute1()
            turnTo(1, myOutput1)


            p_count+=1
            
            if ((p_count>=0)and(p_count<3000)):
                updateCenter()
                
            if ((p_count>=3000)and(p_count<6000)):
                updateEllipse()
            
            if ((p_count>=6000)and(p_count<9000)):
                updateCenter()
                
            if ((p_count>=9000)and(p_count<12000)):
                updateHypotrochoid()

            # I
            if ((p_count>=12000)and(p_count<12200)):
                updatePosition(800.0, 1000.0)

            # II
            if ((p_count>=12200)and(p_count<12400)):
                updatePosition(800.0, -1000.0)

            # III
            if ((p_count>=12400)and(p_count<12600)):
                updatePosition(-800.0, -1000.0)

            # IV
            if ((p_count>=12600)and(p_count<12800)):
                updatePosition(-800.0, 1000.0)
            # I
            if ((p_count>=12800)and(p_count<13000)):
                updatePosition(800.0, 1000.0)

            # II
            if ((p_count>=13000)and(p_count<13200)):
                updatePosition(800.0, -1000.0)

            # III
            if ((p_count>=13200)and(p_count<13400)):
                updatePosition(-800.0, -1000.0)

            # IV
            if ((p_count>=13400)and(p_count<13600)):
                updatePosition(-800.0, 1000.0)

            # I
            if ((p_count>=13600)and(p_count<13800)):
                updatePosition(800.0, 1000.0)

            if ((p_count>=13800)and(p_count<15000)):
                updateCenter()
                
            if (p_count>=15000):
                p_count=0
            
        bus.write_byte_data(addr, 0x0B, 0xFF)
    
    #time.sleep(0.005)   # slow down loop?

# There you have it.  Hacker code.
# I can't wait to see what you guys do with the new
# and improved ball on plate
#  Bob



