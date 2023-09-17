/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Servo.h>
#include <Arduino.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3




float voltage = 0, current = 0;
int adc = 0;

// ******************************************
// Function declaration
// ******************************************
void dataReadyISR();
void SerialDataWrite();

int potValue;

//Define Variables we'll be connecting to
double Setpoint, Input = 0, Output = 0;

//Specify the links and initial tuning parameters
double Kp=0.3, Ki=0.15, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//define loadcell's component
#define LOADCELL_DT_PIN 2
#define LOADCELL_SCK_PIN 3
HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);
#define calibration_factor -1672 // Depends on the load cell
volatile boolean newDataReady;
float loadcell_data; // measurement from the loadcell in gr
// Serial configuration parameters
unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in 

Servo ESC;     // create servo object to control the ESC




// Adafruit_INA219 ina219;

// float current_mA = 0;
// float busvoltage = 0;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  ESC.attach(9,1000,2000); 
  ESC.write(0);


  // if (! ina219.begin()) {
  //   Serial.println("Failed to find INA219 chip");
  //   while (1) { delay(10); }
  // }

  //setup scale
  // scale.begin();
  // scale.start(2000, true);
  // scale.setCalFactor(calibration_factor);
  // Serial.read();
  // pinMode(LOADCELL_DT_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);

  
  //initialize the variables we're linked to
  Input = loadcell_data;

  //turn the PID on
  myPID.SetOutputLimits(0,80);
  myPID.SetSampleTime(50);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  
  if (newDataReady)
  {
    newDataReady = 0;
    loadcell_data = scale.getData();
  }

  Input = loadcell_data * (-1);

    myPID.Compute();
    ESC.write(Output);  

  
  curr_time = micros();
  if (curr_time - prev_time >= dt)
  {
    prev_time += dt;
    SerialDataWrite();
  }

}

void dataReadyISR()
{
  if (scale.update())
    newDataReady = 1;
}



void SerialDataWrite()
{
  Serial.print(loadcell_data); Serial.print(",");
  // Serial.print(Output); Serial.print(",");
  // Serial.print(potValue); Serial.print(",");
  // Serial.print(current_mA + 600); Serial.print(",");
  // Serial.print(busvoltage); Serial.print("");
  Serial.println("");
}
