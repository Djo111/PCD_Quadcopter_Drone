///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  RC Module Libraries
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SPI.h>             // Arduino Serial Peripheral Interface protocol library
#include "nRF24L01.h"
#include "RF24.h"            // RC transceiver module libraries
//#include "WifiSendData.h"
//#include "MPU9250.h"
#include "WifiReceivedata.h"

//  Defines
#define CE   5
#define CSN  4
#define SCK  18
#define MOSI 23
#define MISO 19
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BLDC Motor\ESC Librarie
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32_Servo.h>
//  Defines
#define ESC_0    33        // PWM pin to connect with corresponding ESC
#define ESC_1    12        // These pins control the rotation speed of the BLDC motors
#define ESC_2    13
#define ESC_3    32
// minimum and maximum PWM pulse width
#define MIN_THROTTLE   1000      // you can see it as minimum throttle that can be applied to motors    
#define MAX_THROTTLE    1900      // maximum throttle that can be applied to motors

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MPU9250 Module Libraries
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MPU9250_Youssef_Weslati.h"
#include <Wire.h>
float Roll, Pitch, Yaw;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID gain settings for pich , roll and yaw
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double PID_PITCH_kp =0.00;
double PID_PITCH_kd =0.00;
double PID_PITCH_ki =0.00;

double PID_ROLL_kp =PID_PITCH_kp;
double PID_ROLL_kd =PID_PITCH_kd;
double PID_ROLL_ki =PID_PITCH_ki;
// #define PID_PITCH_kp  kp
// #define PID_PITCH_ki  ki
// #define PID_PITCH_kd  kd

// #define PID_ROLL_kp   PID_PITCH_kp
// #define PID_ROLL_kd   PID_PITCH_kd
// #define PID_ROLL_ki   PID_PITCH_ki

#define PID_YAW_kp    0
#define PID_YAW_kd    0
#define PID_YAW_ki    0
//this are the defines of the additional filter in the PID function
 // Define the filter size
#define FILTER_SIZE 10

// Define the filter arrays
float roll_values[FILTER_SIZE];
float pitch_values[FILTER_SIZE];
float yaw_values[FILTER_SIZE];
int filter_index = 0;

float roll_error, pitch_error, yaw_error;
float roll_integral, pitch_integral, yaw_integral;
float roll_derivative, pitch_derivative, yaw_derivative;
float prev_roll_error, prev_pitch_error, prev_yaw_error;

int16_t PitchPIDOutput, RollPIDOutput, YawPIDOutput;

void pid_controller(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float roll, float pitch, float yaw);

void    inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Uncomment when needed
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG        // for adding and removing debug code 
//#define DEBUG_MPU
//#define DEBUG_FILTER_MPU    
//#define DEBUG_PID_gains
#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;

RF24 receiver(CE, CSN, SCK, MISO, MOSI);  //check link for class methods:https://maniacbug.github.io/RF24/classRF24.html

const byte add[6] = "00001";              //IMPORTANT: The same as in the transmitter

int16_t values_received;
volatile int16_t NRFdata;

#define led 2
#define calibLed 15
float elapsedTime, now, timePrev;

char str1[20];


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Define the queue handle
QueueHandle_t myQueue;


void nrfTask(void *parameter) {
  float n, p, e;
  int16_t NRFdata_send;
  BaseType_t status;
  // setting-up nRF module //
  receiver.begin();       // Begin operation of the chip.
  receiver.setChannel(2); // Which RF channel to communicate on, 0-127
  receiver.setPayloadSize(2);
  receiver.setDataRate(RF24_250KBPS);
  receiver.openReadingPipe(1, add); // Open the pipe number 1 (0-5) for reading .
  receiver.setPALevel(RF24_PA_MIN); // Set Power Amplifier (PA) level to one of four levels (RF24_PA_MIN, RF24_PA_LOW, RF24_PA_MED, RF24_PA_HIGH).
  receiver.startListening();        // Set the radio comunication to receiver mode

  n = millis();
  while (1)
  {
    p = n;
    n = millis();
    e = (n - p) * 1000; //in microseconds
    //Serial.println(e);

    
    if (receiver.available())
    {
      values_received = 0; // throtlle
    
      receiver.read(&values_received, sizeof(int16_t));

      NRFdata_send = values_received;

      // Send the data to the queue
      status = xQueueSend(myQueue, &NRFdata_send, portMAX_DELAY);

        // Check if the data was sent successfully
        if (status != pdPASS) {
            // Handle the error here
            Serial.println("data wasn't sent to the Queue!!");

        }
      
#ifdef DEBUG_NRF
        delay(1);
        Serial.print(" throttle : ");
        Serial.println(int16_t(values_received));
        // Serial.print(" kp : ");
        // Serial.print(values_received[1]);
        // Serial.print(" ki : ");
        // Serial.print(values_received[2]);
        // Serial.print(" kd : ");
        // Serial.print(int16_t(values_received[3]));
        // Serial.print("\n");
      
     #endif
     
}
else{
//secure landing
    #ifdef DEBUG_NRF
     Serial.print("no radio available \n");
    #endif
     NRFdata_send = 1000;
     // Send the data to the queue
      status = xQueueSend(myQueue, &NRFdata_send, portMAX_DELAY);

        // Check if the data was sent successfully
        if (status != pdPASS) {
            // Handle the error here
          Serial.println("can't add data to the Queue in secure landing!!!!");
        }
     
}
      
     vTaskDelay(pdMS_TO_TICKS(1));
  }
  vTaskDelete(NULL);
}




void flight_controll(void *parameter) {
  int16_t NRFdata_receive;
  BaseType_t status;
  float n2, p2, e2;
  n2 = millis();
while (1)
{
     p2 = n2;
     n2 = millis();
     e2 = (n2 - p2) * 1000;
     //Serial.println(e2);

     // Receive the data from the queue
     status = xQueueReceive(myQueue, &NRFdata_receive, portMAX_DELAY);

     // Check if the data was received successfully
    //  if (status == pdPASS)
    //  {
    //  // Process the received data here
    //  // Serial.printf("Received data: %d\n", NRFdata_receive);
    //  }


  IMU_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  
  Roll = getRoll()-0.90;
  Pitch = getPitch()+0.65;
  Yaw = RateYaw;
#ifdef DEBUG_MPU
  Serial.print("roll: ");
  Serial.print(Roll);
  Serial.print(" pitch: ");
  Serial.print(Pitch);
  Serial.print(" yaw: ");
  Serial.println(Yaw);
  #endif

  // get PID gains from the wifi
  mainwificalib(PID_PITCH_kp, PID_PITCH_ki, PID_PITCH_kd, Pitch, Roll);
  PID_ROLL_kp = PID_PITCH_kp;
  PID_ROLL_kd = PID_PITCH_kd;
  PID_ROLL_ki = PID_PITCH_ki;

  pid_controller(0, 0, 0, Roll, Pitch, Yaw);

  UpdateMotorsValues(NRFdata_receive, PitchPIDOutput, RollPIDOutput, 0);

  vTaskDelay(pdMS_TO_TICKS(1));
}
vTaskDelete(NULL);
}











void ESC_calibration(){
    delay(500);
  MOTOR_0.writeMicroseconds(MAX_THROTTLE);
  MOTOR_1.writeMicroseconds(MAX_THROTTLE);
  MOTOR_2.writeMicroseconds(MAX_THROTTLE);
  MOTOR_3.writeMicroseconds(MAX_THROTTLE);
  delay(5000);
  MOTOR_0.writeMicroseconds(MIN_THROTTLE);
  MOTOR_1.writeMicroseconds(MIN_THROTTLE);
  MOTOR_2.writeMicroseconds(MIN_THROTTLE);
  MOTOR_3.writeMicroseconds(MIN_THROTTLE);
  delay(3000);

}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void){
  pinMode(led, OUTPUT);
  pinMode(calibLed, OUTPUT);
#ifdef DEBUG
  Serial.begin(9600);
#endif
  digitalWrite(led, HIGH);
  // setting-up the motors //
  MOTOR_0.attach(ESC_0, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_1.attach(ESC_1, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_2.attach(ESC_2, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_3.attach(ESC_3, MIN_THROTTLE, MAX_THROTTLE);

  // calibrating the ESCs
  ESC_calibration();

  // MPU9250 settings
  setup_mpu9250();

  digitalWrite(calibLed, LOW);

  


  
  //setupWifi();
  setupwificalib();
  



  // Create the queue with a capacity of 10 elements, each element is of size int16_t
  myQueue = xQueueCreate(10, sizeof(int16_t));

  xTaskCreatePinnedToCore(nrfTask,   // Task function
                         "nrfTask",  // Task name (for debugging purposes)
                          8000,      // Stack size (in words)
                          NULL,      // Task parameters
                          1,         // Task priority
                          NULL,      // Task handle
                           0         // Core ID (0 or 1)
                           );

  xTaskCreatePinnedToCore(flight_controll, "flight_controll_Task", 8000, NULL, 1, NULL, 1);

digitalWrite(led, LOW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
}




// Define the PID controller function
void pid_controller(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float roll, float pitch, float yaw) {

//this is an additional filter that help to reduce some noise in the mpu values

  // Add the current values to the filter arrays
//   roll_values[filter_index] = roll;
//   pitch_values[filter_index] = pitch;
//   yaw_values[filter_index] = yaw;

//   // Increment the filter index
//   filter_index = (filter_index + 1) % FILTER_SIZE;

//   // Calculate the average values
//   float roll_sum = 0.0f;
//   float pitch_sum = 0.0f;
//   float yaw_sum = 0.0f;
//   for (int i = 0; i < FILTER_SIZE; i++) {
//     roll_sum += roll_values[i];
//     pitch_sum += pitch_values[i];
//     yaw_sum += yaw_values[i];
//   }
//   float roll_avg = roll_sum / (float)FILTER_SIZE;
//   float pitch_avg = pitch_sum / (float)FILTER_SIZE;
//   float yaw_avg = yaw_sum / (float)FILTER_SIZE;

// #ifdef DEBUG_FILTER_MPU
//         Serial.print("Roll: ");
//         Serial.print(roll_avg);
//         Serial.print(" Pitch: ");
//         Serial.print(pitch_avg);
//         Serial.print(" Yaw: ");
//         Serial.print(yaw_avg);
//         Serial.print("\n");
//   #endif



  // Calculate the errors
  roll_error = setpoint_roll - roll;      //change roll, pitch and yaw with roll_avg, pitch_avg and yaw_avg when using the additional filter!!!
  pitch_error = setpoint_pitch - pitch;
  yaw_error = setpoint_yaw - yaw;

  
  // if(roll_error<1 && roll_error>-1)
  //      roll_error = 0;
  // if(pitch_error<1 && pitch_error>-1)
  //      pitch_error = 0;
  // if(yaw_error<1 && yaw_error>-1)
  //      yaw_error = 0;


// Calculate the integrals
/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -6 and 6 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/

  if(roll_error<6 && roll_error>-6)
       roll_integral += PID_ROLL_ki*(roll_error-prev_roll_error)/2;
  if(pitch_error<6 && pitch_error>-6)
       pitch_integral += PID_PITCH_ki*(pitch_error-prev_pitch_error)/2;
  if(yaw_error<6 && yaw_error>-6)
      yaw_integral += PID_YAW_ki*(yaw_error-prev_yaw_error)/2;
  

  roll_integral = constrain(roll_integral, -300, 300);
  pitch_integral = constrain(pitch_integral, -300, 300);
  yaw_integral = constrain(yaw_integral, -300, 300);
  
  // Calculate the derivatives
  roll_derivative = PID_ROLL_kd*(roll_error - prev_roll_error);
  pitch_derivative = PID_PITCH_kd*(pitch_error - prev_pitch_error);
  yaw_derivative = PID_YAW_kd*(yaw_error - prev_yaw_error);

  // Calculate the PID output
  float roll_output = PID_ROLL_kp * roll_error + roll_integral +  roll_derivative;
  float pitch_output = PID_PITCH_kp * pitch_error + pitch_integral +  pitch_derivative;
  float yaw_output = PID_YAW_kp * yaw_error + yaw_integral +yaw_derivative;

  // Apply the PID output to the drone's control inputs
  RollPIDOutput = static_cast<int16_t>(roll_output);
  PitchPIDOutput = static_cast<int16_t>(pitch_output);
  YawPIDOutput = static_cast<int16_t>(yaw_output);

  RollPIDOutput = constrain(RollPIDOutput, -300, 300);
  PitchPIDOutput = constrain(PitchPIDOutput, -300, 300);
  YawPIDOutput = constrain(YawPIDOutput, -300, 300);

  // Update the previous error values
  prev_roll_error = roll_error;
  prev_pitch_error = pitch_error;
  prev_yaw_error = yaw_error;

  #ifdef DEBUG_PID_VALUES
// Serial.print("pitch PID output: ");
// Serial.print(PitchPIDOutput);
// Serial.print(" roll PID output: ");
// Serial.print(RollPIDOutput);
// Serial.print(" pitch error: ");
// Serial.print(pitch_error);
// Serial.print(" roll error: ");
// Serial.print(roll_error);
Serial.print(" pitch integral: ");
Serial.print(pitch_integral);
Serial.print(" roll integral: ");
Serial.print(roll_integral);
Serial.print("\n");
#endif
#ifdef DEBUG_PID_gains
Serial.print(" kp: ");
Serial.print(PID_PITCH_kp);
Serial.print(" ki: ");

Serial.print(PID_PITCH_ki);
dtostrf(PID_PITCH_kd, 1, 3, str1);
Serial.print(" kd: ");
Serial.print(str);

Serial.print("\n");
#endif
}




void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
                                const int16_t roll_pid_output, const int16_t yaw_pid_output) {
  
  //check if it works correclly for your mpu orientation 
  int16_t m0 = throttle + pitch_pid_output + roll_pid_output + yaw_pid_output;
  int16_t m1 = throttle - pitch_pid_output + roll_pid_output - yaw_pid_output;
  int16_t m2 = throttle + pitch_pid_output - roll_pid_output - yaw_pid_output;
  int16_t m3 = throttle - pitch_pid_output - roll_pid_output + yaw_pid_output;
  
  //motors values should not be more or less than the max and min throttle
  m0 = constrain(m0, MIN_THROTTLE, MAX_THROTTLE);
  m1 = constrain(m1, MIN_THROTTLE, MAX_THROTTLE);
  m2 = constrain(m2, MIN_THROTTLE, MAX_THROTTLE);
  m3 = constrain(m3, MIN_THROTTLE, MAX_THROTTLE);
  #ifdef DEBUG_MOTORS_SPEED
  Serial.print("  m0 :  ");
  Serial.print(m0);
  Serial.print("  m1 :  ");
  Serial.print(m1);
  Serial.print("  m2 :  ");
  Serial.print(m2);
  Serial.print("  m3 :  ");
  Serial.print(m3);
  Serial.print("\n");
  #endif
  MOTOR_0.writeMicroseconds(m0);
  MOTOR_1.writeMicroseconds(m1);
  MOTOR_2.writeMicroseconds(m2);
  MOTOR_3.writeMicroseconds(m3);
}