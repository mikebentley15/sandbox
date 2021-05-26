/*
 * https://circuits4you.com
 * 2016 November 25
 * Load Cell HX711 Module Interface with Arduino to measure weight in Kgs
 Arduino 
 pin 
 2 -> HX711 CLK
 3 -> DOUT
 5V -> VCC
 GND -> GND

 Most any pin on the Arduino Uno will be compatible with DOUT/CLK.
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 *
 * I installed the HX711 library through the Arduino "Tools > Manage
 * Libraries..." and searching for HX711, and choosing the one made by Bogdan
 * Necula and Andreas Motl.
 */

#include "HX711.h"  //You must have this library in your arduino library folder

#define LOADCELL_DOUT_PIN  11
#define LOADCELL_SCK_PIN   12

HX711 loadcell;

float calibration_factor = 418110.f; // found using this calibration technique

//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("HX711 Calibration");
  Serial.println("Remove all weight from loadcell");
  Serial.println("After readings begin, place known weight on loadcell");
  Serial.println("Press a,s,d,f to increase calibration factor by 1,10,100,1000 respectively");
  Serial.println("Press z,x,c,v to decrease calibration factor by 1,10,100,1000 respectively");
  Serial.println("Press t for tare");
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale();
  loadcell.tare(); //Reset the loadcell to 0

  long zero_factor = loadcell.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the loadcell. Useful in permanent loadcell projects.
  Serial.println(zero_factor);

  Serial.print("read: \t\t");
  Serial.println(loadcell.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(loadcell.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(loadcell.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(loadcell.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  loadcell.set_scale(calibration_factor);    // this value is obtained by calibrating the loadcell with known weights; see the README for details
  loadcell.tare();               // reset the loadcell to 0

  Serial.println("After setting up the loadcell:");

  Serial.print("read: \t\t");
  Serial.println(loadcell.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(loadcell.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(loadcell.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(loadcell.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale
}

//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {

  loadcell.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Reading: ");
  Serial.print(loadcell.get_units(5), 6);
  Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();

  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 1;
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 1;
    else if(temp == 's')
      calibration_factor += 10;  
    else if(temp == 'x')
      calibration_factor -= 10;  
    else if(temp == 'd')
      calibration_factor += 100;
    else if(temp == 'c')
      calibration_factor -= 100;
    else if(temp == 'f')
      calibration_factor += 1000;  
    else if(temp == 'v')
      calibration_factor -= 1000;  
    else if(temp == 't')
      loadcell.tare();  //Reset the loadcell to zero
  }
  delay(1); // sleep just for a little
}
//=============================================================================================
