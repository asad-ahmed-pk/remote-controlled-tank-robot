#!/usr/bin/env python3

# motor_control.py
# Motor control using GPIO

import RPi.GPIO as GPIO
import rospy
       
from geometry_msgs.msg import Twist
from time import sleep

# Motor pins
in1 = 24
in2 = 23
in3 = 17
in4 = 27

# Enable pins
enb = 22
en = 25

def setup_gpio():
    in_pins = [in1, in2, in3, in4]

    GPIO.setmode(GPIO.BCM)

    # setup in pins
    for pin in in_pins:
        GPIO.setup(pin, GPIO.OUT)
        
    # setup enable pins
    GPIO.setup(en, GPIO.OUT)
    GPIO.setup(enb, GPIO.OUT)

    # all motors stopped
    for pin in in_pins:
        GPIO.output(pin, GPIO.LOW)

    # enable pins PWM    
    p = GPIO.PWM(en, 1000)
    p2 = GPIO.PWM(enb, 1000)

    p.start(25)
    p2.start(25)
    
# Motor control with x (left/right), z(forward/back)
def drive_motor(x, z):
    if z == 1:
        # forward
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
    elif z == -1:
        # backward
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
    elif x == 1:
        # right
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.LOW)
    elif x == -1:
        # left
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
    else:
        # stopped
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.LOW)
        

# ROS subscriber callback
def process_twist(twist):
    x = twist.linear.x
    z = twist.linear.z
    print(f"Input: {x}, {z}")
    drive_motor(x, z)
    
# ROS main node init
def run_node():
    setup_gpio()
    rospy.init_node('motor_control')
    rospy.Subscriber('/cmd_vel', Twist, process_twist)
    rospy.spin()
    

if __name__ == '__main__':
    run_node()
