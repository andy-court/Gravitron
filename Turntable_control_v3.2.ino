/*
  Code for level shifting from a pot to a Flyskye GT3C transmitter. 
  This code takes a voltage level from 0 to 4.91 and shifts it so that the pot produces a liner change
  in speed and a specified no rotation zone. 
  A multi turn pot is used to get precise voltage readings and to give a smooth turning feeling. 
  Code used from the folowing site:
  https://learn.sparkfun.com/tutorials/mcp4725-digital-to-analog-converter-hookup-guide/all
  Code used for smoothing - https://www.arduino.cc/en/tutorial/smoothing. 
  
  Attempt to fix the issues around the 0.5 mark.  
*/

// Includes
#include<Wire.h>

// Defines
#define MCP4725_ADDR 0x60 

// Pin Setup
int inputPin = A0;
int powerLED = 5;
const byte interruptPin = 2;

// Global variables 
volatile float inputVoltage = 0;
volatile float outputVoltage = 0;
volatile byte state = LOW; 

// Smoothing variables 
const int numReadings = 25;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

//Remove jitter at low speed of 0.5 input
volatile float lastOutput = 2.5; 

//Setup loop ------------------------------------------------------------------------------

void setup() {
  //Serial.begin(115200); //Start serial coms for trouble shooting
  Wire.begin(); //Begins the I2C communication
  pinMode(A2, OUTPUT); //Set to output and used to power and ground the breakout board
  pinMode(A3, OUTPUT);
  pinMode(powerLED, OUTPUT); //Set pin 5 as output to be used for the LED on the power button.
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), pausePlay, FALLING);
  digitalWrite(A2, LOW); //GND
  digitalWrite(A3, HIGH); //Vcc  
  digitalWrite(powerLED, HIGH); //PowerLED pin

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  startSequence();
  dac();
}

//Main loop --------------------------------------------------------------------------------

void loop() {
  inputHandler();
  outputHandler();
  dac();
  //delay(100); //Remove later and implement interrupts
}

//Interrupt handlers -----------------------------------------------------------------------

void pausePlay(){
  static unsigned long last_interrupt_time = 0; //Some useful debounce tricks from - https://forum.arduino.cc/index.php?topic=45000.0
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 400){
     state = !state;
     Serial.println("interrupt recieved");
  }
  last_interrupt_time = interrupt_time;   
}

//Functions --------------------------------------------------------------------------------

void inputHandler() {
  total = total - readings[readIndex]; // Subtract the last reading:
  readings[readIndex] = analogRead(inputPin); // Read the input on analog pin 0:
  total = total + readings[readIndex]; // Add the reading to the total:
  readIndex = readIndex + 1; // Advance to the next position in the array:
  
  if (readIndex >= numReadings) { // Check for end of array
    readIndex = 0; // Wrap around to the beginning:
  }

  average = total / numReadings; // Calculate the average:
  Serial.print("Average: ");
  Serial.println(average); // Send it to the computer as ASCII digits
  delay(1);        // delay in between reads for stability

  inputVoltage = average * (4.91 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 1.39V):
  Serial.print("Input Voltage: "); // Print out the value you read:
  Serial.println(inputVoltage);
  
}



void outputHandler() { //Gives correct output voltage based on scaling and offset
  Serial.print("state: ");
  Serial.println(state);
  if (state == HIGH){ 
    
    // Lower input voltage, reverse speed
    if (inputVoltage < 0.63) {
      outputVoltage = inputVoltage * 1.64179 + 0.90; 
      Serial.println("Dir: CW");   
    }
   // Dead zone, motor stopped
    else if ( (inputVoltage >= 0.63) && (inputVoltage <= 0.72)) {
      outputVoltage = 2.5;
      Serial.println("Stationary");
    }
   // Upper voltage, motor forwards
    else{
     outputVoltage = (inputVoltage - 0.72) * 1.2121 + 2.8;
    /*if (abs(lastOutput - outputVoltage) == 0.31){
      outputVoltage = lastOutput; 
    }*/
     Serial.println("Dir: CCW");
    }
    Serial.print("Output Voltage: ");
    Serial.println(outputVoltage);
  }

  else if (state == LOW){
    Serial.println("Turntable is paused");
    outputVoltage = 2.5;
  }
  
  else{
    outputVoltage = 2.5;
    Serial.print("Output Voltage: ");
    Serial.println(outputVoltage); 
    Serial.println("Input voltage out of bounds");
  }

  lastOutput = outputVoltage; 
}



void dac(){ //Prepares data to be sent to the DAC 
  unsigned int dacOut = (outputVoltage / 4.91) * 4096; //Go from analoug value to 12 bit digital value 
  Serial.print("DAC out: ");
  Serial.println(dacOut);
  Serial.println(""); 
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64); //Command to update the DAC
  Wire.write(dacOut >> 4); //The 8 most significant bits
  Wire.write((dacOut & 15) << 4); //The 4 least significant bits. Think binary, bitwise and (&) dacOut with 15 removes all the bits from 5 to 12. << shifts those bits left. More info: https://www.programiz.com/c-programming/bitwise-operators#and  
  Wire.endTransmission(); //Ends the transmission 
}



void startSequence(){
  outputVoltage = 2.5;
  delay(1500);
  Serial.println("Start sequence run");
}
 
