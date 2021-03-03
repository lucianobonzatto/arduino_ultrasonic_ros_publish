/****************************************************************
      SENSORS STATEMENTS
****************************************************************/
#define numINT 6          // Total number of interrupts
#define triggerPin 4      // Pin number for the common trigger
#define pingDelay 40      // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 200    // How many milliseconds between each ROS publich ; keep > 200ms
#define soundSpeed 343.0  // Speed of sound in m/s

// Interrupts pins in Mega: 2, 3, 18, 19, 20, 21
#define sensorP1 2 // Sensor pin 1 in port 2 
#define sensorP2 3
#define sensorP3 18
#define sensorP4 19
#define sensorP5 20
#define sensorP6 21


volatile unsigned long travelTime[numINT];  // Place to store traveltime of the pusle
volatile unsigned long startTime[numINT];   // Place to store ping times (interrupt)
float distance[numINT];                     // Calculated distances in cm
unsigned long lastPollMillis;
unsigned long lastDebugMillis;


/****************************************************************
      SETUP
****************************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial
  }
  Serial.println("--- Serial monitor started ---");
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
  // Poll every x ms
  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement(); //  Retrieve measurement and set next trigger
    lastPollMillis = millis();
  }

  // Print every y ms (comment out in production)
  if (millis() - lastDebugMillis >= debugDelay)
  {

    noInterrupts();
    Serial.println();
    Serial.println("S__1     S__2     S__3     S__4     S__5     S__6    ");
    for (int i = 0; i < numINT; i++) //numINT; i++)
    {
      Serial.print(distance[i]);
      Serial.print(" --- ");
    }
    
    interrupts();

    Serial.println();
    lastDebugMillis = millis();
  }
}

/****************************************************************
      Retrieve measurement and set next trigger
****************************************************************/
void doMeasurement()
{
  // First read will be 0 (no distance  calculated yet)

  // Read the previous result (pause interrupts while doing so)

  noInterrupts();   // cli()
  // Calculates the speed of all sensors
  for (int i = 0; i < numINT; i++)
  {
    distance[i] = travelTime[i] / 2.0 * (float)soundSpeed / (1000000.0);   // in meters
  }
  interrupts();   // sei();

  // Initiate next trigger
  // digitalWrite(triggerPin, LOW);  // rest of loop already takes > 2µs
  // delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);    // HIGH pulse for at least 10µs
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);     // Set LOW again
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
  interruptHandler(pinRead, 0);
}


// INTerrupt number 1 (pin 3 on Mega)
void call_INT2()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP2);    // Slower ; Read pin 3
  // pinRead = PINE >> 5 & B00000001;    // Faster ; Read pin 3/PE5
  pinRead = PINE & B00100000;    // Faster ; Read pin 3/PE5
  interruptHandler(pinRead, 1);
}


// INTerrupt number 2 (pin 18 on Mega)
void call_INT3()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP3);    // Slower ; Read pin 18
  // pinRead = PIND >> 3 & B00000001;    // Faster ; Read pin 18/PD3
  pinRead = PIND & B00001000;    // Faster ; Read pin 18/PD3
  interruptHandler(pinRead, 2);
}


// INTerrupt number 3 (pin 19 on Mega)
void call_INT4()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP4);    // Slower ; Read pin 19
  // pinRead = PIND >> 2 & B00000001;    // Faster ; Read pin 19/PD2
  pinRead = PIND & B00000100;    // Faster ; Read pin 19/PD2
  interruptHandler(pinRead, 3);
}


// INTerrupt number 4 (pin 20 on Mega)
void call_INT5()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP5);      // Slower ; Read pin 20
  // pinRead = PIND >> 1 & B00000001;  // Faster ; Read pin 20/PD1
  pinRead = PIND & B00000010;  // Faster ; Read pin 20/PD1
  interruptHandler(pinRead, 4);
}


// INTerrupt number 5 (pin 21 on Mega)
void call_INT6()
{
  byte pinRead;
  // pinRead = digitalRead(sensorP6);      // Slower ; Read pin 21
  pinRead = PIND & B00000001;  // Faster ; Read pin 21/PD0
  interruptHandler(pinRead, 5);
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
// END
