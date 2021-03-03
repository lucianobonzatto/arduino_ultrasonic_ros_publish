/****************************************************************
      SENSORS STATEMENTS
****************************************************************/
#define sonarNum 6          // Total number of interrupts
#define pingInterval 40      // How many milliseconds between the measurement of each sensor; keep > 5ms
#define debugInterval 200    // How many milliseconds between each publication ; keep > 200ms
#define soundSpeed 343.0  // Speed of sound in m/s

// Triggers pins
#define triggerPin1 4      // Pin number for the sensor_1 trigger
#define triggerPin2 5      // Pin number for the sensor_2 trigger
#define triggerPin3 6      // Pin number for the sensor_3 trigger
#define triggerPin4 7      // Pin number for the sensor_4 trigger
#define triggerPin5 8      // Pin number for the sensor_5 trigger
#define triggerPin6 9      // Pin number for the sensor_6 trigger
const int triggerPin[6] = {4, 5, 6, 7, 8, 9};

// Interrupts pins in Mega: PIN 2, PIN 3, PIN 18, PIN 19, PIN 20, PIN 21
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


/****************************************************************
      SETUP
****************************************************************/
void setup(){

  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial
  }

  setupMultipleSonarIntervals();

  Serial.println("--- Serial monitor started ---");
  pinMode(triggerPin[0], OUTPUT);   // Set individual triggerpin as output
  pinMode(triggerPin[1], OUTPUT);   // Set individual triggerpin as output
  pinMode(triggerPin[2], OUTPUT);   // Set individual triggerpin as output
  pinMode(triggerPin[3], OUTPUT);   // Set individual triggerpin as output
  pinMode(triggerPin[4], OUTPUT);   // Set individual triggerpin as output
  pinMode(triggerPin[5], OUTPUT);   // Set individual triggerpin as output


  // Manage interrupt pins here
  pinMode(sensorP1, INPUT);    // Set interrupt pin 02 (INT0) as INPUT (sensor 1)
  pinMode(sensorP2, INPUT);    // Set interrupt pin 03 (INT1) as INPUT (sensor 2)
  pinMode(sensorP3, INPUT);    // Set interrupt pin 18 (INT2) as INPUT (sensor 3)
  pinMode(sensorP4, INPUT);    // Set interrupt pin 19 (INT3) as INPUT (sensor 4)
  pinMode(sensorP5, INPUT);    // Set interrupt pin 20 (INT4) as INPUT (sensor 5)
  pinMode(sensorP6, INPUT);    // Set interrupt pin 21 (INT5) as INPUT (sensor 6)


  attachInterrupt(digitalPinToInterrupt(sensorP1), call_INT1, CHANGE );   // ISR for INT0
  attachInterrupt(digitalPinToInterrupt(sensorP2), call_INT2, CHANGE );   // ISR for INT1
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
    noInterrupts(); // Pause interrupts
    
    doMeasurements(); // Retrieve measurements
    
    Serial.println();
    Serial.println("S__1     S__2     S__3     S__4     S__5     S__6    ");
    for (int i = 0; i < sonarNum; i++) //sonarNum; i++)
    {
      Serial.print(distance[i]);
      Serial.print(" --- ");
    }
    
    interrupts(); // Enable interrupts

    Serial.println();
    lastDebugMillis = millis();
  }
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
  pinRead = PINE & B00010000;        // Faster ; Read pin 2/PE4
  interruptHandler(pinRead, INTSensor1);
}


// INTerrupt number 1 (pin 3 on Mega)
void call_INT2()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP2);    // Slower ; Read pin 3
  pinRead = PINE & B00100000;    // Faster ; Read pin 3/PE5
  interruptHandler(pinRead, INTSensor2);
}


// INTerrupt number 2 (pin 18 on Mega)
void call_INT3()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP3);    // Slower ; Read pin 18
  pinRead = PIND & B00001000;    // Faster ; Read pin 18/PD3
  interruptHandler(pinRead, INTSensor3);
}


// INTerrupt number 3 (pin 19 on Mega)
void call_INT4()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP4);    // Slower ; Read pin 19
  pinRead = PIND & B00000100;    // Faster ; Read pin 19/PD2
  interruptHandler(pinRead, INTSensor4);
}


// INTerrupt number 4 (pin 20 on Mega)
void call_INT5()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP5);      // Slower ; Read pin 20
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
