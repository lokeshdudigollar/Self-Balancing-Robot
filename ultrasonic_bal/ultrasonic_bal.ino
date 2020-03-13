/* Balancing robot
– uses 2 modified servos for balancing
– uses a potentiometer to adjust the perfect balance point
– uses one Ping))) ultrasonic sensor to read the distance to the surface
– uses a PID control algorithm to adjust the servo pulses so the Ping)))
sensor reading will match the value of the perfect balance point
*/

//Input/outputs
#define PingPin 7 // digital pin 7
#define PotPin 0 // analog pin 0
#define Lservo 4 // digital pin 4
#define Rservo 5 // digital pin 5
#define Led 13 //digital pin 13


//Variables
unsigned int Ptime = 0; // raw value from sensors
int Drive = 0; // PWM value sent to Servos
int Error[5]; // array of 5 Error elements
int P = 0; // proportional term
int I = 0; // integral term
int D = 0; // derivative term
int SetPoint = 307; // perfect balance point value
byte Count = 0; // counter
int Period = 0; // period for generating sounds
int Lwheel = 0; // left wheel variable
int Rwheel = 0; // right wheel variable
int Pot = 0; // potentiometer variable

//tests should be made to determine acurate Min, Mid and Max values for the servos
#define Midl 1460 // center for servos, they should be stoped
#define Midr 1460



//Ping PID constants
#define Kp 2
#define Ki 3
#define Kd 4


//Meaningful names for error array index:
#define Current 0
#define Sum 1
#define Last 2
#define SecondToLast 3
#define Delta 4



void setup() {
pinMode(PingPin, OUTPUT);
digitalWrite(PingPin, LOW);
pinMode(Led, OUTPUT);
digitalWrite(Led, LOW);
pinMode(Lwheel, OUTPUT);
digitalWrite(Lwheel, LOW);
pinMode(Rwheel, OUTPUT);
digitalWrite(Rwheel, LOW);


Serial.begin (19200);
Serial.println(“start”);


//delay(2000); //wait 2 seconds before start for debug purposes
}
There are 4 functions that are looped at about 20ms: Read_Pot_sensor, Read_Ping_Sensor, PID, Drive_Motors.

void loop(){
Read_Pot_Sensor();
Read_Ping_Sensor();
PID();
Drive_Motors();
delay(7);     //wait 7 miliseconds, adjust this value for a 18 to 20 ms loop
}

1. Read_Pot_Sensor.

This function was added to read the pot and set the balancing Setpoint.

int Read_Pot_Sensor() {
Pot = analogRead(PotPin);
//Serial.print (“Pot = “);                  // debug – remember to comment out
//Serial.println (Pot, DEC);              // debug – remember to comment out
return Pot;
}

2. Read_ping_sensor.

The Ping))) sensor has only one signal pin which acts as both trigger and echo. The microcontroller has to output a high signal for 5us (microseconds), then it switches to input mode, waits for the pin to go high, starts a counter and waits for the pin to go low again. At that moment the counter will return the time of sound flight from the sensor to the surface and back to the sensor. If distance is needed, this value has to be divided by 2 and multiplied by 29.034 to get the distance in centimeters or 11.3236 to get the distance in inches. For this application, the time of flight will be stored in the Ptime variable. Here is the code:

int Read_Ping_Sensor() {
//trigger the sensor
pinMode(PingPin, OUTPUT);
digitalWrite(PingPin, LOW);
delayMicroseconds(2);
digitalWrite(PingPin, HIGH);
delayMicroseconds(5);
digitalWrite(PingPin, LOW);
//receive the echo
pinMode(PingPin, INPUT);
digitalWrite(PingPin, HIGH);              //turn on pull up resistor
Ptime = pulseIn(PingPin, HIGH);
//print out the value for fine tuning of SetPoint constant
//Serial.print (“Ping time = “);            // debug – remember to comment out
//Serial.println (Ptime, DEC);              // debug – remember to comment out
return Ptime;
}
