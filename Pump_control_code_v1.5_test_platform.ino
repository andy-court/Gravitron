//Description----------------------------------------------------------------------------------------------------

/*
   This code us used to controll the temprature of the fluid in the smoke machine. A PI controller is used.

   Code version: 1.1

   Reference:
    https://create.arduino.cc/projecthub/Oniichan_is_ded/lcd-i2c-tutorial-664e5a - LCD
    https://learn.adafruit.com/ad8495-thermocouple-amplifier/arduino - Temp sensor
    https://playground.arduino.cc/Code/PIDLibaryBasicExample/ - PID library
    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/ - library breakdown here
    https://forum.arduino.cc/index.php?topic=45000.0 - debounch tricks 
    https://controlguru.com/ - controls info 
    http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/ - PID auto tune library blog post for Arduino 

    Changes:
    02/09/2020 -  This code uses the Arduino PID library. 
    03/09/2020 -  This code uses 1 pot and a pushbutton to set the three PID constants.
                  Direction pins for the motor change from 2,3 to 5,6, pot input on A1.
                  Setpoint can now vary from 50.0 deg C to 350.0 deg C. 
                  Add code to check average temp, will be used for shut off of gas.  
                  Add code to check how much gas is lowing. 
*/

//Defines and Includes-------------------------------------------------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <Time.h>
#include <TimeAlarms.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

#define TC_PIN A3           // set to ADC pin used
#define AREF 5.0            // set to AREF, typically board voltage like 3.3 or 5.0
#define ADC_RESOLUTION 10   // set to ADC bit resolution, 10 is default

#define SET_POINT_PIN A2    // set pin A4 to read the set point temp from the pot 

#define PUMP_EA 10          // pump PWM pin
#define PUMP_DIR_I2 5       // pump direction pin I2
#define PUMP_DIR_I1 6       // pump direction pin I1

#define PID_CONST_SET A1    // pin of pot which changes the PID constants
#define INTERRUPT_PIN 2     // push button interrupt pin 
#define TEMP_ARRAY_SIZE 60
#define RELAY_CONTROL 4     // digital pin for the relay controlling the gas shutoff 

#define GAS_POT A1           // pot on the gas knob, ereplacing the pid tune knob at A1  

// note: A4 (SDA), A5 (SCL)- for LDC

//Global Variabes------------------------------------------------------------------------------------------------

double setPoint, input, output;// PID variables
int Kp =2, Ki = 1, Kd = 0, COBias = 150; 
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, REVERSE);// PID setup
volatile int PIDParamIndex = 0; 
float tempArray[TEMP_ARRAY_SIZE];
volatile int arrayIndex = 0; 


//Main Function--------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  lcd.init();// initialize the lcd
  lcd.backlight();
  myPID.SetMode(AUTOMATIC);// turn on PID
  setupPins();
  myPID.SetOutputLimits(0,255);
  lcd.setCursor(1,0);
  lcd.print("Setup Run!");
  delay(1000);
}

void loop(){// main loop
  setPoint = readSetPoint();
  input = claculateProbeTemp(TC_PIN); 
  //updateTunings();
  //myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute();
  outputInterpreter(output);// output from pid will be in range from 0 to 255
  lcd.setCursor(1,0);
  lcd.print((String)"SP:" + round(setPoint) + " In:" + round(input) + "   ");// the extra spaces at the end are for overwriting the number :) 
  lcd.setCursor(1,1);
  lcd.print((String)"Kp:" + Kp + " Ki:" + Ki + " Kd:" + Kd + "   ");
}  

//Setups--------------------------------------------------------------------------------------------------------

void setupPins(void) {
  pinMode(PUMP_DIR_I2, OUTPUT);
  pinMode(PUMP_DIR_I1, OUTPUT);
  pinMode(PUMP_EA, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), changePIDConst, FALLING);
  Alarm.timerRepeat(2, Repeats); //Timer used to minotor temprature
  pinMode(RELAY_CONTROL, OUTPUT);  
}

//Operation Function-------------------------------------------------------------------------

float getVoltage(int raw_adc){// function converts the digital value from the ADC to a voltage
  return raw_adc * (AREF / (pow(2, ADC_RESOLUTION) - 1));
}

float getProbeTemperature(float voltage){// function uses the converted voltage with temp sensor data sheet equations to get the temperature
  return (voltage - 1.25) / 0.005;
}

float claculateProbeTemp(int pinToRead){ // function stors the probe temp in a global variable and
  int tempProbeReading = analogRead(pinToRead);
  float tempProbeVoltage = getVoltage(tempProbeReading);// used for printing graphs, remove later
  Serial.print(getProbeTemperature(tempProbeVoltage));
  Serial.print("   ");
  return getProbeTemperature(tempProbeVoltage);
}

float readSetPoint(void){ // function gets the pot voltage and mpas it to a temprature range in degrees celcius
  float potReading = analogRead(SET_POINT_PIN);
  int gasSetting = analogRead(GAS_POT);
  if (getVoltage(gasSetting) > 2.5){// standard setpoint, gas high
    Serial.print(map(potReading, 0.0, (pow(2, ADC_RESOLUTION) - 1), 30.0, 350.0));// used for printing graphs, remove later
  }
  else{// lower setpoint, gas low 
    Serial.print(map(potReading, 0.0, (pow(2, ADC_RESOLUTION) - 1), 30.0, 350.0) -20);// used for printing graphs, remove later
  }
    Serial.print("   ");
    
  if (getVoltage(gasSetting) > 2.5){// standard setpoint, gas high
    return map(potReading, 0.0, (pow(2, ADC_RESOLUTION) - 1), 30.0, 350.0); // map pot reading from 50 degrees C to 200 degrees C
  }
  else{// lower setpoint, gas low 
    return map(potReading, 0.0, (pow(2, ADC_RESOLUTION) - 1), 30.0, 350.0) - 20; // map pot reading from 50 degrees C to 200 degrees C
  }
  
}

void outputInterpreter(float controlSignal){ 
  digitalWrite(PUMP_DIR_I2, HIGH);
  digitalWrite(PUMP_DIR_I1, LOW);
  analogWrite(PUMP_EA, controlSignal);
  Serial.println(controlSignal);// used for printing graphs, remove later
}

int readTunings(int pinToRead){// function to modify the tuning parameters 
  int potValue = analogRead(pinToRead); // get ADC value from the pot 
  return map(potValue, 0, (pow(2, ADC_RESOLUTION) - 1), 0, 20); // gives a new tuning parameter from 0 to 20 
}

void updateTunings(void){
  int potVal = readTunings(PID_CONST_SET);
  if (PIDParamIndex == 0){
    Kp = potVal;
  }
  else if (PIDParamIndex == 1){
    Ki = potVal;
  }
  else if (PIDParamIndex == 2){
    Kd = potVal;
  }
}

//Interrupt handlers -----------------------------------------------------------------------

void changePIDConst(void){// function used to handle the push button input
  static unsigned long last_interrupt_time = 0; 
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 400){// add interrupt code in the if statement
    if (PIDParamIndex < 2){
      PIDParamIndex ++; 
    }
    else{
      PIDParamIndex = 0; 
    }
  }
  last_interrupt_time = interrupt_time;   
}

void Repeats(void){// executes every 2 seconds and isused for data capture 
  if (arrayIndex < TEMP_ARRAY_SIZE){// somwhere in the array
    tempArray[arrayIndex] = input;// input is appended to the array
  }
  else{// end of array, set index to zero, 2 minuts have elapsed
    arrayIndex = 0;
    float arrayTotal = 0; 
    for (int i = 0; i < TEMP_ARRAY_SIZE; i++){
      arrayTotal += tempArray[i];
    }
    float arrayAverage = arrayTotal/TEMP_ARRAY_SIZE;
    if (arrayAverage > 200.0){// average temprature is above 200 deg C for 2 mins, dangerous, gas shut off. 
      
      lcd.setCursor(1,0);
      lcd.print("OVER HEATING");
      lcd.setCursor(1,1);
      lcd.print("GAS SHUTDOWN");
    }
  }
  arrayIndex ++;
   
  
}
