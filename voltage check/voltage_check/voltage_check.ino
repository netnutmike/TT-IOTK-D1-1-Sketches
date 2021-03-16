void setup() {
  Serial.begin(115200);             // initialize serial at 115,200 baud
  // the builtin LED on the D1 Mini is connected to pin 2
  //pinMode(LED_BUILTIN, OUTPUT);     // Set the LED pin to OUTPUT mode
} // setup()

// the loop function runs continuously until power is removed
// read the ADC and blink the onboard LED
void loop() {
  // read the ADC value. it is an integer between 0 and 1023
  int val = analogRead(A0);         // this is an Arduino function
  // convert the ADC value to a floating point decimal from 0 to 5 volts
  // do not remove the .0 in the next line. it is necessary to
  // make the result a float
  float volts = 5.0 * val / 1023.0;
  // print the voltage to the Serial Monitor
  Serial.print("ADC Voltage reads = ");
  Serial.println(volts);

    // turn the LED on and off
  //digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on
  delay(100);                       // Wait 0.1 second (100 milliseconds)
  //digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off
  delay(900);                       // Wait 0.9 second (900 milliseconds)
} // loop()
