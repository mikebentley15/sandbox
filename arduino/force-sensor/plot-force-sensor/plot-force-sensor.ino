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

 * Michael's comments:
 *
 *  This program simply outputs the sensor readings according to the given calibration factor.
 *  It will do this as fast as it can, then the serial plotter can be used to see the 
 *  measurements.
 *
 *  Observations:
 *  - A single measurement can be made every 85-86 milliseconds
 *  - Asking for a measurement before the last one will block until it can complete
 *  - Doing other work during those 86 milliseconds will make the call be super quick
 *  - Could do other things and limit myself to measuring the force sensor at 10 Hz.
 *  - If we do it every 86 ms, the function call takes 1-2 ms and results in about 11.6 Hz.
 */

#include "HX711.h"  //You must have this library in your arduino library folder

#define LOADCELL_DOUT_PIN  11
#define LOADCELL_SCK_PIN   12

HX711 loadcell;

float calibration_factor = 418110.f; // found using this calibration technique
unsigned long prev_millis = 0;

//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  Serial.begin(9600);
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(calibration_factor);
  loadcell.tare(); //Reset the loadcell to 0
  delay(1000);
  Serial.println("Mass(g),Milliseconds,Millis(call)");
  prev_millis = millis();
}

//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  unsigned long curr_millis = millis();
  unsigned long diff_millis = curr_millis - prev_millis;
  prev_millis = curr_millis;
  delay(90);
  curr_millis = millis();
  Serial.print(loadcell.get_units()*1000, 6);
  Serial.print("\t");
  Serial.print(diff_millis);
  Serial.print("\t");
  diff_millis = millis() - curr_millis;
  Serial.print(diff_millis);
  Serial.println();
}
//=============================================================================================
