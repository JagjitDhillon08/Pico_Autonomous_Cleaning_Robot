
import utime #used for timing
from time import sleep #used for sleep function to have small pauses
from machine import Pin, PWM, time_pulse_us 
from MPU6050 import MPU6050 #imports the gyroscope library
from motors import * #grabs all motor functions or movements 
print("start") 
trigger_Front = Pin(16, Pin.OUT) #Front sensors trigger, releases the ultrasonic sensor
echo_Front    = Pin(17, Pin.IN) #Front sensors echo, which captures the ultrasonic sensor 
print("mpu6050 trying to connect") 
mpu = MPU6050() #connects to gyroscope
print("connected")
c_yaw    = 0.0 #current direction in degrees
t_yaw    = 0.0 #direction it wants to face
deadband = 0.8 #minimum gyro reading that is considered, anything under this is ignored
loopcounter    = 0 #counts number of loops in the main code
looptarget     = -1 #target loop, when roomba reaches this it will turn 90 degrees
turnside      = 1 #which way to spin 1 is right -1 is left, should flip after each turn 
turnsidetwo = 1 #creates a copy of turnside so the second matches the first 
buzzer = PWM(Pin(27)) #setup for the buzzer
buzzer.duty_u16(0) #buzzer duty of 0 indicates is silent 
l_time = utime.ticks_ms() #records the current time
print("calibrating gyro")
mpu.set_gyro_range(0x10) 
x_offset = 0.0 #the offset applied in gyro calculation data
for _ in range(200):
    x_offset += mpu.read_gyro_data()["x"]
    utime.sleep_ms(2)
    #adds all the x readings together for 200 times, taking 2 millisecond breaks in between each reading
x_offset /= 200 #average the 200 to find the existing drift
print("X-offset:", round(x_offset, 2)) #show the rounded drift value
print("STARTING LOOP") 
def read_distance(trig, ech): #code for the HC-SR04 that determines distance in cm
    try:
        timeout = utime.ticks_us()
        while ech.value() == 1: 
            if utime.ticks_diff(utime.ticks_us(), timeout) > 30000:
                return 999.0
            #sometimes echo can get stuck on high, if the timeout exceeds 30 milliseconds, return 999 to indicate a sensor error
        trig.low() 
        utime.sleep_us(5)
            #brief 5 microsecond pause before setting releasing trigger
        trig.high()
        utime.sleep_us(10)
            #sends trigger 
        trig.low() #end the trigger pulse
        utime.sleep_us(50) 
        duration = time_pulse_us(ech, 1, 30000) #measure the travel time
        if duration <= 0:
            return 999.0
                #if echo never came back, return 999 to indicate error
        return (duration * 0.0343) / 2 #calculates microseconds to cm using the duration factored by the speed of sound divided 2 times for the one way trip
    except Exception as e: # on error, return 999 
        print("Sensor error:", e)
        return 999.0
try:
    while True: #Main loop
        loopcounter += 1 #increase loop counter
        try:
            c_time = utime.ticks_ms() #update to current time
            dt = utime.ticks_diff(c_time, l_time) / 1000.0 #difference in time from last loop
            l_time = c_time #last time = current time for the next loop
            dt = max(dt, 0.001) #avoid dividing by 0 on the first loop
            dist = read_distance(trigger_Front, echo_Front) #calculate distance
            utime.sleep_ms(15)
            try:
                gyro = mpu.read_gyro_data() #read the gyro
                gZ = gyro["x"] - x_offset # x axis rate minus drift gives the true rotation
                if abs(gZ) > deadband: # ignores tiny readings
                    c_yaw += gZ * dt #true heading = degrees/second times seconds
            except Exception as e:
                print("Gyro read failed:", e) #if gyro fails, no rotation
                gZ = 0.0
            c_yaw = c_yaw % 360.0
            t_yaw = t_yaw % 360.0
            print("facing:", round(c_yaw, 1),
                  "| goal:", round(t_yaw, 1),
                  "| f:", round(dist, 1))
                    #outputs current yaw, target yaw and distance to object or wall
            if looptarget == loopcounter: 
                print("looptarget reached")
                t_yaw = (t_yaw + 90 * turnsidetwo) % 360.0
                    #if target is reached, add 90 degrees to the same direction as the first spin
            if 2 < dist < 20: 
                try:
                    buzzer.freq(500)
                    buzzer.duty_u16(32768)
                        #if something is within 2 to 20 cm, the roomba is too close and the buzzer will go off to make an alert(performing at 50% duty)
                except Exception: pass
                stop()
                utime.sleep_ms(500)
                    #if buzzer encounters an error, ignore it and stop all motors 
                try:
                    buzzer.duty_u16(0) #silence the buzzer
                except Exception: pass
                print("180 degree turn time")
                target_heading = (t_yaw + 90.0 * turnside) % 360.0
                        #ignore the buzzer, set the turn target 
                looptarget = loopcounter + 10 #10 loops from now, turn 90 degrees again
                turnsidetwo = turnside #remember which way the turn went
                turnside = -turnside #next time turn the other way
                l_time = utime.ticks_ms() #reset time value
                while True:
                    c_time_spin = utime.ticks_ms() #current time
                    dt_spin = max(utime.ticks_diff(c_time_spin, l_time) / 1000.0, 0.001) #Seconds from last spin
                    l_time = c_time_spin #set last time to current time
                    try:
                        gZ = mpu.read_gyro_data()["x"] - x_offset #read gyro and remove the drift
                        if abs(gZ) > deadband: #filter out small readings 
                            c_yaw = (c_yaw + (gZ * dt_spin)) % 360.0 #update degrees heading after turn 
                    except Exception: pass
                    d_yaw_spin = (c_yaw - target_heading + 180.0) % 360.0 - 180.0 #degrees remaining
                    abs_err = abs(d_yaw_spin) #Degrees left to turn
                    if abs_err < 15: #very close to turn, go slower
                        turn_speed = 14000
                    elif abs_err < 45:
                        turn_speed = 19000 #moderately close to turn, moderate speed
                    else:
                        turn_speed = 25000 #far from turn, full speed
                    if abs_err < 2.0: #close enough, consider turn finished
                        stop() #stop motors
 
                        coast_end = utime.ticks_add(utime.ticks_ms(), 350) # For 350 milliseconds, keep reading
                        while utime.ticks_diff(coast_end, utime.ticks_ms()) > 0: #until 350 milliseconds is over
                            c_time_coast = utime.ticks_ms() #current coast time
                            dt_coast = max(utime.ticks_diff(c_time_coast, l_time) / 1000.0, 0.001)
                            l_time = c_time_coast #save time for next loop
                            try:
                                gZ = mpu.read_gyro_data()["x"] - x_offset #read gyro with offset
                                if abs(gZ) > deadband:
                                    c_yaw = (c_yaw + (gZ * dt_coast)) % 360.0 #capture coast rotation
                            except Exception: pass
                            utime.sleep_ms(10) #10 millisecond wait
                        break
                    if d_yaw_spin > 0: #heading above target, turn to bring back down 
                        right(turn_speed)
                    else:
                        left(turn_speed) #heading below target, spin to bring back up 
                    utime.sleep_ms(10)
                t_yaw = target_heading #set new target after turning
                utime.sleep_ms(200)
                continue
            d_yaw = (c_yaw - t_yaw + 180.0) % 360.0 - 180.0 #degrees between current and target heading
            if abs(d_yaw) > 30.0:
                if d_yaw > 0:
                    right(25000)
                        #if more than 30 degrees off, turn right
                else:
                    left(25000) # negative, so spin left
            elif abs(d_yaw) > 2.0: #move forward, while turning 
                if d_yaw > 0: #positive, curve right 
                    motorA_forward(25000) #motor A curve  faster to curve back
                    motorB_forward(17000) #motor B curve slower to curve back 
                else:
                    motorA_forward(17000) #motor A curve slower to curve back
                    motorB_forward(25000) #motor B curve faster to curve back
            else:
                if dist >= 15: 
                    forward(30000)
                    utime.sleep_ms(20)
                            #if all clear ahead drive straight
        except Exception as e:
            print("LOOP ERROR:", e) 
            stop()
            utime.sleep_ms(500)
                #if any random crash happens in the main loop, stop the motors
except KeyboardInterrupt:
    print("stop button pressed") #hitting keyboard stops code from running
finally:
    print("shutting down") 
    stop() #stops motors
    try:
        buzzer.duty_u16(0) #stops buzzer 
    except:
        pass
    print("shutdown complete")
 