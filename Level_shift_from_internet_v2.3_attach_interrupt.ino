/*
  Code for level shifting from a pot to a Flyskye GT3C transmitter. 
  This code takes a voltage level from 0 to 4.91 and shifts it so that the pot produce liner change in speed and a specified no rotation zone. 
  Code used from the folowing site https://learn.sparkfun.com/tutorials/mcp4725-digital-to-analog-converter-hookup-guide/all
*/

// Includes
#include<Wire.h>

// Defines
#define MCP4725_ADDR 0x60 

// Pin Setup
int inputPin = A0;
int outputPin = 5;
const byte interruptPin = 2;

// Global variables
volatile float inputVoltage = 0;
volatile float outputVoltage = 0;
volatile byte state = LOW; 


void setup() {
  Serial.begin(115200); //Start serial coms for trouble shooting
  Wire.begin(); //Begins the I2C communication
  pinMode(A2, OUTPUT); //Set to output and used to power and ground the breakout board
  pinMode(A3, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), pausePlay, FALLING);
  digitalWrite(A2, LOW); //GND
  digitalWrite(A3, HIGH); //Vcc  

  startSequence();
  Serial.println("Start sequence run");
  dac();
}

void loop() {
    inputHandler();
    outputHandler();
    dac();

  //delay(100); //Remove later and implement interrupts
}

void pausePlay(){
  static unsigned long last_interrupt_time = 0; //Some useful debounce tricks from - https://forum.arduino.cc/index.php?topic=45000.0
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200){
     state = !state;
     Serial.println("interrupt recieved");
  }
  last_interrupt_time = interrupt_time; 
  
}

void inputHandler() {
  int sensorValue = analogRead(inputPin); // Read the input on analog pin 0:
  inputVoltage = sensorValue * (4.91 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.print("Input Voltage: "); // Print out the value you read:
  Serial.println(inputVoltage);
}

void outputHandler() { //Gives correct output voltage based on scaling and offset
  Serial.print("state: ");
  Serial.println(state);
  if (state == HIGH){
    
    // Lower input voltage, reverse speed
    if (inputVoltage < 2.21) {
      outputVoltage = inputVoltage * 0.747 + 0.85; //Originally + 0.85
      Serial.println("Reverse zone");
    }
   // Dead zone, motor stopped
    else if ( (inputVoltage >= 2.21) && (inputVoltage <= 2.70)) {
      outputVoltage = 2.5;
      Serial.println("Dead zone");
    }
   // Upper voltage, motor forwards
    else{
     outputVoltage = (inputVoltage - 2.7) * 0.320 + 2.9; //Originally  * 0.339 + 2.9
     Serial.println("Forwards zone");
    }
    Serial.print("Output Voltage: ");
    Serial.println(outputVoltage);
  }

  else{
    outputVoltage = 2.5;
    Serial.print("Output Voltage: ");
    Serial.println(outputVoltage);
    Serial.println("Dead zone");
  }
}

void dac(){ //Prepares data to be sent to the DAC 
  unsigned int dacOut = (outputVoltage / 4.91) * 4096; //Go from analoug value to 12 bit digital value 
  Serial.print("DAC out: ");
  Serial.println(dacOut);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64); //Command to update the DAC
  Wire.write(dacOut >> 4); //The 8 most significant bits
  Wire.write((dacOut & 15) << 4); //The 4 least significant bits. Think binary, bitwise and (&) dacOut with 15 removes all the bits from 5 to 12. << shifts those bits left. More info: https://www.programiz.com/c-programming/bitwise-operators#and  
  Wire.endTransmission(); //Ends the transmission 
}

void startSequence(){
  outputVoltage = 2.5;
  delay(1500);
}
