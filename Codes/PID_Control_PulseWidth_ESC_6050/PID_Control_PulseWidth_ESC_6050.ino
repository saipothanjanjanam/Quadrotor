/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
Servo esc_signal1; 
Servo esc_signal2;

MPU6050 mpu;

float e_total = 0, e_pre = 0, e_now, Time  ;

void setup() 
{
  Serial.begin(115200);
  esc_signal1.attach(12);  //Specify here the pin number on which the signal pin of ESC is connected.
  esc_signal2.attach(9); 
  Time = millis();
  esc_signal1.writeMicroseconds(1000);
  esc_signal2.writeMicroseconds(1000);
  delay(3000);             //ESC initialization delay.
  // Set up the interrupt pin, its set as active high, push-pull
  
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

void loop()
{
    float Time_Prev = Time ;
    Time = millis();
    float Time_Elapse = (Time - Time_Prev)/1000;
 
  // Read normalized values  
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  float pitch = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  //float roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  // Output
  //Serial.print(" Pitch = ");
  //Serial.print(pitch);
  //Serial.print(" Roll = ");
  //Serial.print(roll);
  
  //Serial.println();

         Serial.println(pitch-4.5) ;
         float kp = 10, kd = 15, ki = 0, desired = 0;   
         float error = desired - (pitch);
         e_now = error;
         //Serial.print(error);
         float pid, pid1, pid2, throttle = 1200;
         e_total = e_total + e_now;
         pid = kp*e_now + kd*(e_now - e_pre) + ki*e_total; 
        // Serial.println(pid);
//       float p = kp*e_now, d =  kd*(e_now - e_pre), i = ki*e_total; 

         if (pid > -100 && pid < 100)
         {
                esc_signal1.writeMicroseconds(throttle+pid);          
                esc_signal2.writeMicroseconds(throttle -pid-10);
//              Serial.print(throttle + pid);
//              Serial.print("\t");
//              Serial.println(throttle - pid);                  
         }
         else if (pid > 100)
         {
               esc_signal1.writeMicroseconds(throttle + pid);
               pid1 = 100;
               esc_signal2.writeMicroseconds(throttle - pid1-10);
//             Serial.print(throttle + pid);
//             pid1 = 200;
//             Serial.print("\t");
//             Serial.println(throttle - pid1);
         }
         else if (pid < -100)
         {
               esc_signal2.writeMicroseconds(throttle - pid-10);
               pid2 = -100;
               esc_signal1.writeMicroseconds(throttle + pid2);
//             Serial.print(throttle - pid);
//             pid2 = -200;
//             Serial.print("\t");
//             Serial.println(throttle + pid2);
         }
         e_pre = e_now ;
}


