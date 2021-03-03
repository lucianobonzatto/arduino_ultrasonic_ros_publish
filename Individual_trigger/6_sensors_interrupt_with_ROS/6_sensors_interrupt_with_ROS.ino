/****************************************************************
      ROS STATEMENTS
****************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h> // Type of message to be published

// ---- Declarations of ROS parameters ----

//create an object which represents the ROS node.
ros::NodeHandle nh;

//Create three instances for range messages.
sensor_msgs::Range range_sensor_1;
sensor_msgs::Range range_sensor_2;
sensor_msgs::Range range_sensor_3;
sensor_msgs::Range range_sensor_4;
sensor_msgs::Range range_sensor_5;
sensor_msgs::Range range_sensor_6;

//Create publisher objects for all sensors
ros::Publisher pub_range_sensor_1("/ultrasound_1", &range_sensor_1);
ros::Publisher pub_range_sensor_2("/ultrasound_2", &range_sensor_2);
ros::Publisher pub_range_sensor_3("/ultrasound_3", &range_sensor_3);
ros::Publisher pub_range_sensor_4("/ultrasound_4", &range_sensor_4);
ros::Publisher pub_range_sensor_5("/ultrasound_5", &range_sensor_5);
ros::Publisher pub_range_sensor_6("/ultrasound_6", &range_sensor_6);


/****************************************************************
      SENSORS STATEMENTS
****************************************************************/
#define sonarNum 6          // Total number of interrupts
#define pingInterval 40      // How many milliseconds between each measurement ; keep > 5ms
#define debugInterval 200    // How many milliseconds between each ROS publich ; keep > 200ms
#define soundSpeed 343.0  // Speed of sound in m/s

// Triggers pins
#define triggerPin1 4      // Pin number for the sensor_1 trigger
#define triggerPin2 5      // Pin number for the sensor_2 trigger
#define triggerPin3 6      // Pin number for the sensor_3 trigger
#define triggerPin4 7      // Pin number for the sensor_4 trigger
#define triggerPin5 8      // Pin number for the sensor_5 trigger
#define triggerPin6 9      // Pin number for the sensor_6 trigger
const int triggerPin[6] = {4, 5, 6, 7, 8, 9};

// Interrupts pins in Mega: 2, 3, 18, 19, 20, 21
#define sensorP1 2 // Sensor pin 1 in port 2 
#define sensorP2 3
#define sensorP3 18
#define sensorP4 19
#define sensorP5 20
#define sensorP6 21

// Interrupts number in Mega: INT.0, INT.1, INT.2, INT.3, INT.4, INT.5
#define INTSensor1 0
#define INTSensor2 1
#define INTSensor3 2
#define INTSensor4 3
#define INTSensor5 4
#define INTSensor6 5

volatile unsigned long travelTime[sonarNum];  // Place to store traveltime of the pusle
volatile unsigned long startTime[sonarNum];   // Place to store ping times (interrupt)
float distance[sonarNum];                     // Calculated distances in cm
unsigned long lastPollMillis;
unsigned long lastDebugMillis;
unsigned long pingTimer[sonarNum];



void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26; // ~ 14.89 degree
  range_name.min_range = 0.2;
  range_name.max_range = 6.0;
}



/****************************************************************
      SETUP
****************************************************************/
void setup()
{
  // Init ROS node
  nh.initNode();

  // Set the publishers
  nh.advertise(pub_range_sensor_1);
  nh.advertise(pub_range_sensor_2);
  nh.advertise(pub_range_sensor_3);
  nh.advertise(pub_range_sensor_4);
  nh.advertise(pub_range_sensor_5);
  nh.advertise(pub_range_sensor_6);

  // msg setup
  sensor_msg_init(range_sensor_1, "/ultrasound_1");
  sensor_msg_init(range_sensor_2, "/ultrasound_2");
  sensor_msg_init(range_sensor_3, "/ultrasound_3");
  sensor_msg_init(range_sensor_4, "/ultrasound_4");
  sensor_msg_init(range_sensor_5, "/ultrasound_5");
  sensor_msg_init(range_sensor_6, "/ultrasound_6");

  pinMode(triggerPin, OUTPUT);   // Set common triggerpin as output


  // Manage interrupt pins here
  pinMode(sensorP1, INPUT);     // Set interrupt pin 02 (INT0) as INPUT (sensor 1)
  pinMode(sensorP2, INPUT);     // Set interrupt pin 03 (INT1) as INPUT (sensor 2)
  pinMode(sensorP3, INPUT);    // Set interrupt pin 18 (INT2) as INPUT (sensor 3)
  pinMode(sensorP4, INPUT);    // Set interrupt pin 19 (INT3) as INPUT (sensor 4)
  pinMode(sensorP5, INPUT);    // Set interrupt pin 20 (INT4) as INPUT (sensor 5)
  pinMode(sensorP6, INPUT);    // Set interrupt pin 21 (INT5) as INPUT (sensor 6)


  attachInterrupt(digitalPinToInterrupt(sensorP1), call_INT1, CHANGE );    // ISR for INT0
  attachInterrupt(digitalPinToInterrupt(sensorP2), call_INT2, CHANGE );    // ISR for INT1
  attachInterrupt(digitalPinToInterrupt(sensorP3), call_INT3, CHANGE );   // ISR for INT2
  attachInterrupt(digitalPinToInterrupt(sensorP4), call_INT4, CHANGE );   // ISR for INT3
  attachInterrupt(digitalPinToInterrupt(sensorP5), call_INT5, CHANGE );   // ISR for INT4
  attachInterrupt(digitalPinToInterrupt(sensorP6), call_INT6, CHANGE );   // ISR for INT5
  
  lastPollMillis = millis();
  lastDebugMillis = millis();
}



/****************************************************************
      LOOP
****************************************************************/
void loop()
{

  // Ping times are managed here
  for (uint8_t i = 0; i < sonarNum; i++){
    if (millis() >= pingTimer[i]){
      pingTimer[i] +=  pingInterval * sonarNum;
      
      digitalWrite(triggerPin[i], HIGH);    // HIGH pulse for at least 10µs
      delayMicroseconds(10);
      digitalWrite(triggerPin[i], LOW);     // Set LOW again
    }
  }

  // Here is where the data is to be processed or published out
  if (millis() - lastDebugMillis >= debugInterval)
  {

    noInterrupts();

    doMeasurements(); // Retrieve measurements
    
    range_sensor_1.range = distance[0];
    range_sensor_2.range = distance[1];
    range_sensor_3.range = distance[2];
    range_sensor_4.range = distance[3];
    range_sensor_5.range = distance[4];
    range_sensor_6.range = distance[5];

    // Update time stamp
    range_sensor_1.header.stamp = nh.now();
    range_sensor_2.header.stamp = nh.now();
    range_sensor_3.header.stamp = nh.now();
    range_sensor_4.header.stamp = nh.now();
    range_sensor_5.header.stamp = nh.now();
    range_sensor_6.header.stamp = nh.now();

    // Publish in each ROS sensor topic
    pub_range_sensor_1.publish(&range_sensor_1);
    pub_range_sensor_2.publish(&range_sensor_2);
    pub_range_sensor_3.publish(&range_sensor_3);
    pub_range_sensor_4.publish(&range_sensor_4);
    pub_range_sensor_5.publish(&range_sensor_5);
    pub_range_sensor_6.publish(&range_sensor_6);
    interrupts();

    lastDebugMillis = millis();
  }
  nh.spinOnce();//Handle ROS events
}

/****************************************************************
      Retrieve measurement and set next trigger
****************************************************************/
void doMeasurements()
{
  // First read will be 0 (no distance  calculated yet)
  
  // Calculates the speed of all sensors
  for (int i = 0; i < sonarNum; i++)
  {
    distance[i] = travelTime[i] / 2.0 * (float)soundSpeed / (1000000.0);   // in meters
  }
}



/****************************************************************
      INTERRUPT handling
****************************************************************/
// The PIN register is used to read the digital value of the pin.
// INTerrupt number 0 (pin 2 on Mega)

void call_INT1()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP1);    // Slower ; Read pin 2
  // pinRead = PINE >> 4 & B00000001;        // Faster ; Read pin 2/PE4
  pinRead = PINE & B00010000;        // Faster ; Read pin 2/PE4
  interruptHandler(pinRead, INTSensor1);
}


// INTerrupt number 1 (pin 3 on Mega)
void call_INT2()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP2);    // Slower ; Read pin 3
  // pinRead = PINE >> 5 & B00000001;    // Faster ; Read pin 3/PE5
  pinRead = PINE & B00100000;    // Faster ; Read pin 3/PE5
  interruptHandler(pinRead, INTSensor2);
}


// INTerrupt number 2 (pin 18 on Mega)
void call_INT3()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP3);    // Slower ; Read pin 18
  // pinRead = PIND >> 3 & B00000001;    // Faster ; Read pin 18/PD3
  pinRead = PIND & B00001000;    // Faster ; Read pin 18/PD3
  interruptHandler(pinRead, INTSensor3);
}


// INTerrupt number 3 (pin 19 on Mega)
void call_INT4()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP4);    // Slower ; Read pin 19
  // pinRead = PIND >> 2 & B00000001;    // Faster ; Read pin 19/PD2
  pinRead = PIND & B00000100;    // Faster ; Read pin 19/PD2
  interruptHandler(pinRead, INTSensor4);
}


// INTerrupt number 4 (pin 20 on Mega)
void call_INT5()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP5);      // Slower ; Read pin 20
  // pinRead = PIND >> 1 & B00000001;  // Faster ; Read pin 20/PD1
  pinRead = PIND & B00000010;  // Faster ; Read pin 20/PD1
  interruptHandler(pinRead, INTSensor5);
}


// INTerrupt number 5 (pin 21 on Mega)
void call_INT6()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP6);      // Slower ; Read pin 21
  pinRead = PIND & B00000001;  // Faster ; Read pin 21/PD0
  interruptHandler(pinRead, INTSensor6);
}


// Common function for interrupts
void interruptHandler(bool pinState, int nIRQ)
{
  unsigned long currentTime = micros();  // Get current time (in µs)
  if (pinState)
  {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else
  {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}

/****************************************************************
      Auxiliaries functions
****************************************************************/
void setupMultipleSonarIntervals() {
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < sonarNum; i++)
  pingTimer[i] = pingTimer[i - 1] + pingInterval;
}


// END
