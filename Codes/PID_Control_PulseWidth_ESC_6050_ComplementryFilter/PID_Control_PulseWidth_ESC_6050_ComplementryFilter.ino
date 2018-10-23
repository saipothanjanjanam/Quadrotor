#include <Wire.h>
#include <Servo.h>


Servo motor1;
Servo motor2;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float e_total = 0, e_pre = 0, e_now;
float elapsedTime, Time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  motor1.attach(3); //attatch the right motor to pin 3
  motor2.attach(5);  //attatch the left motor to pin 5

  Time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs  won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  motor1.writeMicroseconds(1000); 
  motor2.writeMicroseconds(1000);
  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = Time;  // the previous time is stored before the actual time read
    Time = millis();  // actual time read
    elapsedTime = (Time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Read the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

         float output = Total_angle[0] - 1;                    //Output angle of pid loop i.e .., given by MPU - 9250
         float desired = 0;                                 //Desired angle
         float kp = 5.8, kd = 0.8, ki = 0.0008;             //Proportional,Derivative and Integral constants
         float error = desired - output;                    //Calculating error 
         float pid, pid1, pid2;        
         float throttle = 1200;                             //Throttle at zero degrees is declaring 
         e_now = error;                                     //e_now is the present error a.k.a error                             
         e_total = e_total + e_now;                         //Summing up the previous errors
         
         pid = kp*e_now + kd*(e_now - e_pre)/elapsedTime + ki*e_total;  //PID Algorithm 
         
//       float p = kp*e_now, d =  kd*(e_now - e_pre), i = ki*e_total; 
//       9 - sig1 - motor1, 12 - signal2 - motor2
         Serial.println(output);
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
         e_pre = e_now ;   
}//end of loop void

