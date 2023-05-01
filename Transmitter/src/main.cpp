/*#include <Arduino.h>
#include <SPI.h>  
#include <RF24.h>       // RC transceiver module libraries
#include <nRF24L01.h> 

#define CE    4        // necessary for RC tranceiver
#define CSN   5 

RF24      transmitter(CE, CSN);         


const byte address[6] = "00001";
int16_t values_to_send;

void setup()
{
  Serial.begin(115200);
  transmitter.begin();                      
  transmitter.stopListening();              
  transmitter.openWritingPipe(address);
}

void loop() {
  

  transmitter.stopListening();
  int potValue = analogRead(34);
  values_to_send = map(potValue, 0, 1023, 0, 180);
  Serial.println(values_to_send);
  transmitter.write(&values_to_send, sizeof(int16_t));

  // put your main code here, to run repeatedly:
}*/

#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <LiquidCrystal_I2C.h>
/*#include <WiFi.h>
const char* ssid = "Mi 9 Lite";
const char* password = "12345678@";*/

LiquidCrystal_I2C lcd(0x27, 16, 2);

int16_t values_to_send=0;


RF24 radio(4, 5, 18, 19, 23);
const byte add[6] = "00001";
float n, e, p;

float map_with_step(float value, float low, float high, float tolow, float tohigh, float step) {
  // Calculate the range of the input values and output values
  float from_range = high - low;
  float to_range = tohigh - tolow;

  // Calculate the scaled value in the input range
  float scaled_value = (value - low) / (from_range / step);

  // Map the scaled value to the output range
  float mapped_value = tolow + (scaled_value * (to_range / step));

  return mapped_value;
}

void setup(void) {
  Serial.begin(9600);
   radio.begin();
  radio.setChannel(2);
  radio.setPayloadSize(2);
  //radio.setRetries(15, 15);
  //radio.setAutoAck(true);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(add);
   radio.setPALevel(RF24_PA_MIN);
  radio.stopListening(); 

  lcd.init(); //initialize the lcd
lcd.backlight();


/*WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.println("Connecting to WiFi...");
}
*/
//Serial.println("Connected to WiFi");
//Serial.print("IP address: ");
//Serial.println(WiFi.localIP());
n = millis();
}
/*int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
val = constrain(val, lower, upper);
if ( val < middle )
val = map(val, lower, middle, 0, 128);
else
val = map(val, middle, upper, 128, 255);
return ( reverse ? 255 - val : val );
}*/


void loop(void) {
p = n;
n = millis();
e = (n - p) / 1000;
//Serial.println(e * 1000000);
int throttle = analogRead(14);
int pitch = analogRead(25);
int kp = analogRead(33);
// int ki = analogRead(14);
int kd = analogRead(27);
values_to_send = map(kd, 0, 4095, 1600, 1000);



// if(1080 <= pitch && pitch <=3090){
//   values_to_send[4] = 0;
// }else if(1100 >= pitch){
//   values_to_send[4]=map_with_step(pitch, 0, 1080, 40, 0,1);
// }else{
//   values_to_send[4]=map_with_step(pitch, 3090, 4095 ,0, -40,1);
// }

// values_to_send[1] = map_with_step(kp, 0, 4095, 0.0, 5,0.02);
// values_to_send[2] = map_with_step(ki, 0, 4095, 0.0, 50, 1);
// values_to_send[3] = map_with_step(kd, 0, 4095, 0, 1, 0.01);

Serial.print(" throttle : ");
Serial.println(int16_t(values_to_send));


// Serial.print(" pitch : ");
// Serial.print(int16_t(values_to_send[4]));
// Serial.print(" kp : ");
// Serial.print(values_to_send[1]);
// Serial.print(" ki : ");
// Serial.print(values_to_send[2]);
// Serial.print(" kd : ");
// Serial.print(values_to_send[3]);
//Serial.print("\n");

// lcd.clear(); // clear display
// lcd.setCursor(0, 0);
// lcd.print("THROTTLE: ");
// lcd.print(values_to_send);



//   lcd.print("ki");
//   lcd.print(values_to_send[2]);
  
// lcd.setCursor(0, 1);
// lcd.print("kd");
//   lcd.print(values_to_send[3]);
  //lcd.print("throttle: "); // move cursor to   (0, 0)
  //lcd.print(int16_t(values_to_send[0]));
  

  
  /*Serial.print(" pitch : ");
  Serial.print(values_to_send3);
  Serial.print(" yaw : ");
  Serial.print(values_to_send4);
  Serial.print("\n");
  lcd.print(" roll : ");
  lcd.print(values_to_send2);*/
  radio.write(&values_to_send, sizeof(int16_t));
  //delay(0);
}