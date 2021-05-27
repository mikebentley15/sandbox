/*     Simple Stepper Motor Control Exaple Code
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
// defines pins numbers
const int linear_step_pin = 5;
const int linear_dir_pin  = 6;
const int rotary_step_pin = 5;
const int rotary_dir_pin  = 6;

void setup() {
  // Sets the two pins as Outputs
  pinMode(linear_step_pin, OUTPUT);
  pinMode(linear_dir_pin,  OUTPUT);
  //pinMode(rotary_step_pin, OUTPUT);
  //pinMode(rotary_dir_pin,  OUTPUT);
}
void loop() {
  digitalWrite(linear_dir_pin, HIGH); // Enables the motor to move in a particular direction
  //digitalWrite(rotary_dir_pin, HIGH); // Enables the motor to move in a particular direction

  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(linear_step_pin, HIGH);
    //digitalWrite(rotary_step_pin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(linear_step_pin, LOW);
    //digitalWrite(rotary_step_pin, LOW);
    delayMicroseconds(2000);
  }

  // Makes 400 pulses for making two full cycle rotation
  digitalWrite(linear_dir_pin, LOW); // Enables the motor to move in a particular direction
  //digitalWrite(rotary_dir_pin, LOW); // Enables the motor to move in a particular direction
  for(int x = 0; x < 400; x++) {
    digitalWrite(linear_step_pin, HIGH);
    //digitalWrite(rotary_step_pin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(linear_step_pin, LOW);
    //digitalWrite(rotary_step_pin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
}
