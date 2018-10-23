#include <Servo.h>

Servo esc_signal1;
Servo esc_signal2;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  esc_signal1.attach(12);
  esc_signal2.attach(9);
  delay(1000);
  esc_signal1.write(30);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  esc_signal2.write(30);
  delay(3000);             //ESC initialization delay.
}

void loop() {
  // put your main code here, to run repeatedly:
 esc_signal1.write(45);
 esc_signal2.write(45);
}
