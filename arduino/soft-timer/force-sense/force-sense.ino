/*
 * Plays with https://github.com/yehnan/SoftTimer with force sensor as an example
 *
 * Note: you must install this library manually.  I did it with
 *   mkdir -p ~/Arduino/libraries
 *   pushd ~/Arduino/libraries
 *   git clone https://github.com/yehnan/SoftTimer
 *   popd
 */

#include "HX711.h"
#include "SoftTimer.h"

#define LOADCELL_DOUT_PIN  11
#define LOADCELL_SCK_PIN   12

HX711 loadcell;
SoftTimer timer;

float calibration_factor = 418110.f; // found using this calibration technique
unsigned long prev_millis = 0;

void setup() {
  Serial.begin(9600);
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(calibration_factor);
  loadcell.tare(); //Reset the loadcell to 0
  delay(1000);
  Serial.println("Mass(g),Milliseconds,Millis(call)");
  prev_millis = millis();

  timer.every(90, print_force);
  timer.every(20, fake_work);
}

void loop() {
  timer.update(); // start the event loop
}

bool print_force(EventBase* event) {
  unsigned long curr_millis = millis();
  unsigned long diff_millis = curr_millis - prev_millis;
  prev_millis = curr_millis;
  curr_millis = millis();
  Serial.print(loadcell.get_units()*1000, 6);
  Serial.print("\t");
  Serial.print(diff_millis);
  Serial.print("\t");
  diff_millis = millis() - curr_millis;
  Serial.print(diff_millis);
  Serial.println();

  return false; // remember to return false, means you want the event to continue
}

bool fake_work(EventBase* event) {
  delay(5);
  Serial.print(" ");
  return false;
}
