from machine import Pin, PWM

# Left Motor Pins (Motor A)
in1 = Pin(2, Pin.OUT)
in2 = Pin(3, Pin.OUT)
ena = PWM(Pin(8))
ena.freq(1000)

# Right Motor Pins (Motor B)
in3 = Pin(4, Pin.OUT)
in4 = Pin(5, Pin.OUT)
enb = PWM(Pin(7))
enb.freq(1000)

#Basic motor movements(forward, backward, stop)
#these are specific to each motor and are used in combinations that make the roomba go right, left, forward, backward
#input1 is forward direction movement, input2 is backward direction movement
def motorA_forward(speed):
    in1.high()
    in2.low()
    ena.duty_u16(speed)

def motorA_backward(speed):
    in1.low()
    in2.high()
    ena.duty_u16(speed)

def motorA_stop():
    in1.low()
    in2.low()
    ena.duty_u16(0)

def motorB_forward(speed):
    in3.high()
    in4.low()
    enb.duty_u16(speed)

def motorB_backward(speed):
    in3.low()
    in4.high()
    enb.duty_u16(speed)

def motorB_stop():
    in3.low()
    in4.low()
    enb.duty_u16(0)

DEFAULT_SPEED = 30000 ## duty_u16 ranges 0–65535, 30000 is a calm, sustainable pace of about 45% power

def forward(speed=DEFAULT_SPEED):
    motorA_forward(speed)
    motorB_forward(speed)
        #Forward movement is created through both motors moving at the same speed in the forward direction

def backward(speed=DEFAULT_SPEED): 
    motorA_backward(speed)
    motorB_backward(speed)
        #Backward movement is created through both motors moving at the same speed in the backward direction 

def stop():
    motorA_stop()
    motorB_stop()

def left(speed=DEFAULT_SPEED):
    motorA_backward(speed)
    motorB_forward(speed)
        #To turn left, the left motor goes backward and the right motor goes forward

def right(speed=DEFAULT_SPEED):
  
    motorA_forward(speed)
    motorB_backward(speed)
        #To turn right, the left motor goes forward and the right motor goes backward