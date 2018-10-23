#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Servo.h>
#define AHRS true               // Set to false for basic data read
#define SerialDebug true        // Set to true to get Serial output for debugging

Servo motor1;                   //Creating motor1 object using servo class 
Servo motor2;                   //Creating motor2 object using servo class

float e_total = 0, e_pre = 0, e_now, Time  ;
int intPin = 11;                // These can be changed, 2 and 3 are the Arduinos ext int pins

MPU9250 myIMU;                  //Creating myIMU object of MPU9250 class

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);
  motor1.attach(12);            //Specifing 12th pin number on which the signal pin of ESC of motor 1 is connected.
  motor2.attach(9);             //Specifing 9th pin number on which the signal pin of ESC of motor 2 is connected.
  Time = millis();              //Counting the time for time elapse
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  delay(3000);                  //ESC initialization delay.
  //pinMode(intPin, INPUT);       // Set up the interrupt pin, its set as active high, push-pull
  //digitalWrite(intPin, LOW);
  //byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read the WHO_AM_I register, this is a good test of communication
}

void loop()
{
    float Time_Prev = Time ;
    Time = millis();
    float Time_Elapse = (Time - Time_Prev)/1000;  //Time elapsed for each single loop

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
 
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;
      
      if(SerialDebug)
      {  
         float output = myIMU.pitch - 1;                    //Output angle of pid loop i.e .., given by MPU - 9250
         float desired = 0;                                 //Desired angle
         float kp = 7, kd = 0, ki = 0;                      //Proportional,Derivative and Integral constants
         float error = desired - output;                    //Calculating error 
         float pid, pid1, pid2;        
         float throttle = 1200;                             //Throttle at zero degrees is declaring 
         e_now = error;                                     //e_now is the present error a.k.a error                             
         e_total = e_total + e_now;                         //Summing up the previous errors
         
         pid = kp*e_now + kd*(e_now - e_pre)/Time_Elapse + ki*e_total;  //PID Algorithm 
         
//       float p = kp*e_now, d =  kd*(e_now - e_pre), i = ki*e_total; 
//       9 - sig1 - motor1, 12 - signal2 - motor2
         
         if (pid > -100 && pid < 100)
         {
                motor1.writeMicroseconds(throttle + pid);          
                motor2.writeMicroseconds(throttle - pid);  
         }
         if (pid > 100)
         {
               motor1.writeMicroseconds(throttle + pid);
               pid1 = 100;
               motor2.writeMicroseconds(throttle - pid1);
         }
         if (pid < -100)
         {
               motor2.writeMicroseconds(throttle - pid);
               pid2 = -100;           
               motor1.writeMicroseconds(throttle + pid2);
         }
         
         e_pre = e_now ;      //Making present error to past error        
       }
}
