// /********************************************************
//  * PID Basic Example
//  * Reading analog input 0 to control analog PWM output 3
//  ********************************************************/

// #include <PID_v1.h>
// #include <Servo.h>
// #include <Arduino.h>
// #include <HX711_ADC.h>
// #include <Wire.h>
// #include <Adafruit_INA219.h>

// #define PIN_INPUT 0
// #define PIN_OUTPUT 3

// float voltage = 0, current = 0;
// int adc = 0;

// // ******************************************
// // Function declaration
// // ******************************************
// void dataReadyISR();
// void SerialDataWrite();

// int potValue;



// //define loadcell's component
// #define LOADCELL_DT_PIN 2
// #define LOADCELL_SCK_PIN 3
// HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);
// #define calibration_factor -1710 // Depends on the load cell
// volatile boolean newDataReady;
// float loadcell_data; // measurement from the loadcell in gr
// // Serial configuration parameters
// unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in 

// Servo ESC;     // create servo object to control the ESC




// Adafruit_INA219 ina219;

// float current_mA = 0;
// float busvoltage = 0;
// float power = 0;

// void setup()
// {
//   // initialize serial communication at 9600 bits per second:
//   Serial.begin(9600);
//   ESC.attach(9,1000,2000); 
//   ESC.write(0);


//   if (! ina219.begin()) {
//     Serial.println("Failed to find INA219 chip");
//     while (1) { delay(10); }
//   }

//   //setup scale
//   scale.begin();
//   scale.start(2000, true);
//   scale.setCalFactor(calibration_factor);
//   Serial.read();
//   pinMode(LOADCELL_DT_PIN, INPUT);
//   attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);
// }

// void loop()
// {
//   potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
//   potValue = map(potValue, 0, 1023, 0, 180);

//   busvoltage = ina219.getBusVoltage_V();
//   current_mA = ina219.getCurrent_mA();
//   power = ina219.getPower_mW();
  
//   if (newDataReady)
//   {
//     newDataReady = 0;
//     loadcell_data = scale.getData();
//   }

//   ESC.write(potValue);  

  
//   curr_time = micros();
//   if (curr_time - prev_time >= dt)
//   {
//     prev_time += dt;
//     SerialDataWrite();
//   }

// }

// void dataReadyISR()
// {
//   if (scale.update())
//     newDataReady = 1;
// }



// void SerialDataWrite()
// {
//   Serial.print(loadcell_data); Serial.print(",");
//   Serial.print(potValue); Serial.print(",");
//   Serial.print(current_mA + 600); Serial.print(",");
//   Serial.print(busvoltage); Serial.print(",");
//   Serial.print(power); Serial.print("");
//   Serial.println("");
// }










#include <Servo.h>

Servo ESC1;   
Servo ESC2;



int potValue1 = 0;  // value from the analog pin
int potValue2 = 0;

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(10,1000,2000);
}

void loop() {
  potValue1 = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue1 = map(potValue1, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)

  potValue2 = analogRead(A1);
  potValue2 = map(potValue2, 0, 1023, 0, 180);

  ESC1.write(potValue1);    // Send the signal to the ESC
  ESC2.write(potValue2);

  Serial.println(potValue1 + "," + potValue2);
}