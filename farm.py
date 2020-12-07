# This code was written by Andy Court

# Include libraries

import RPi.GPIO as GPIO
import time
import pygame

# Pin setup

speaker     = 3 # Relay chanel 2
pump        = 5 # Relay chanel 3, the solenoid is also conected here
strobes     = 7 # Relay chanel 5
buzzer      = 11 # Relay chanel 6
topPanel    = 13 # Relay chanel 7
bottomPanel = 15 # relay chanel 8 

bigB        = 19
resetB      = 21
squirtB     = 23

# Flags

progState = 2 # Start with 2 shich is everytinh off and reset
panelState = 1
doOnce = 1 

# Counters

lightCounter = 0
endCounter = 0

# Constants 

lightOnTime = 115
endValue = 115


# General GPIO setup 

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin bering

GPIO.setup(speaker, GPIO.OUT)
GPIO.setup(pump, GPIO.OUT)
GPIO.setup(strobes, GPIO.OUT)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setup(topPanel, GPIO.OUT)
GPIO.setup(bottomPanel, GPIO.OUT)

# Interrupt setup, handlers before actual setup 

def bigRedButton(channel):
    global progState
    progState = 0
    global doOnce
    doOnce = 1

GPIO.setup(bigB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(bigB,GPIO.RISING,callback=bigRedButton) # Setup event on pin 10 rising edge

def resetButton(channel):
    global progState
    progState = 2

GPIO.setup(resetB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.add_event_detect(resetB,GPIO.RISING,callback=resetButton) 

def squirtButton(channel):
    global progState
    progState = 1

GPIO.setup(squirtB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.add_event_detect(squirtB,GPIO.RISING,callback=squirtButton)

# Initiations

pygame.mixer.init()

# Working functions

def lightsOn():
    GPIO.output(strobes, 1)
    GPIO.output(topPanel, 1)
    GPIO.output(speaker, 1)
    #print("lights on functions")
    
    
def lightsOff():
    GPIO.output(buzzer, 0)
    GPIO.output(strobes, 0)
    GPIO.output(bottomPanel, 0)
    GPIO.output(pump, 0)
    GPIO.output(speaker, 0)
    #GPIO.output(topPanel, 0)
    #print("lights off function")
    
def squirt():
    GPIO.output(strobes, 1)
    GPIO.output(topPanel, 1)
    time.sleep(1)
    GPIO.output(pump, 1)
    GPIO.output(buzzer, 1)
    time.sleep(0.5)
    GPIO.output(pump,0)
    GPIO.output(buzzer,0)
    print("squirt function")
    
def flashPannel():
    GPIO.output(bottomPanel, 1)
    time.sleep(0.25)
    GPIO.output(bottomPanel, 0)
    time.sleep(0.25)
    #print("flash function")
    
    # Need some work
    
def reset():
    lightsOff()
    pygame.mixer.music.stop()
    pygame.mixer.music.rewind()
    pygame.mixer.music.stop()
    doOnce = 1
    global lightCounter
    lightCounter = 0
    #print("reset function")
    
       
    
# Main functions. Use global progState to switch between them


def zero(): # Play music, main button
    global doOnce
    global lightCounter
    global panelState
    if doOnce == 1:
        lightsOn()
        GPIO.output(speaker,1)
        pygame.mixer.music.load("mix.mp3") 
        pygame.mixer.music.play()
        doOnce = 0
    
    if lightCounter >= lightOnTime:
        global progState
        progState = 3
        reset()
        global doOnce
        doOnce = 1
        
    
    flashPannel()
    
    lightCounter = lightCounter + 1
    
    print("state 0")
    
    

def one(): # Squirt, squirt button 
    reset()
    squirt()
    global progState
    reset()
    progState = 3
    
    print("state 1")
    

def two(): # Reset button
    reset()
    global progState
    progState = 3 
    
    print("state 2")

def three(): # Leave top panel on
    if endCounter > endValue:
        GPIO.output(topPanel, 0)
    
    global endCounter
    endCounter = endCounter + 1 
    
    print("state 3")    
    

def switch(argument):
    switcher = {
        0: zero,
        1: one,
        2: two,
        3: three,
    }
    # Get the function from switcher dictionary
    func = switcher.get(argument, lambda: "nothing")
    # Execute the function
    return func()

while True: # Infinate loop 
    switch(progState)
 
    
    



    
    
    
    
