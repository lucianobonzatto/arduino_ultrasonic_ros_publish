#include <NewPing.h>

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h> 

#define SONAR_NUM     6 // Number or sensors.
#define MAX_DISTANCE 600 // Max distance in cm.
#define MIN_DISTANCE 20 // Min distance in cm
#define FIELD_OF_VIEW 0.26 // field of view // 0.26 rad = 14.89 degree
#define PING_INTERVAL 100 // Milliseconds between pings.

unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t oldSensorReading[SONAR_NUM];
uint8_t currentSensor = 0; // Which sensor is active.



//create an object which represents the ROS node.
ros::NodeHandle nh;

NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE),
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
  NewPing(10, 11, MAX_DISTANCE),
  NewPing(12, 13, MAX_DISTANCE),
};

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


void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = FIELD_OF_VIEW;
  range_name.min_range = MIN_DISTANCE; // cm;
  range_name.max_range = MAX_DISTANCE; // cm;
}



void setupMultipleSonarIntervals() {
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}



void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = float(sonar[currentSensor].ping_result / US_ROUNDTRIP_CM);
}



void oneSensorCycle() { // Do something with the results.

  // Update distance value to last valid read for each sensor
  for (uint8_t i = 0; i < SONAR_NUM; i++){
    cm[i] = returnLastValidRead(i, cm[i]);
  }

  // Print in serial port
  //printInSerialPort();

  // Publish results in ROS topics
  publishInRosTopics();
}



//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, float cm) { // uint8_t
  if (cm != 0) {
    oldSensorReading[sensorArray] = cm;
    return cm;
  } 
  else return oldSensorReading[sensorArray];
}


// Print results in serial port
void printInSerialPort(){
  for (uint8_t i = 0; i < SONAR_NUM; i++) {    
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm       ");
  }
  Serial.println("\n");
}



void publishInRosTopics(){
  //Serial.println("Publica");
  // Update range in each sensor_msgs
  range_sensor_1.range = cm[0];
  range_sensor_2.range = cm[1];
  range_sensor_3.range = cm[2];
  range_sensor_4.range = cm[3];
  range_sensor_5.range = cm[4];
  range_sensor_6.range = cm[5];

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
}



void setup() {

  Serial.begin(115200);
  setupMultipleSonarIntervals();

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
}


long int conter = 0;
void loop() {

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;

      if (i == 0 && currentSensor == SONAR_NUM - 1){
        
        // Update, print in serial and publish in topics
        oneSensorCycle();
      }

      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck); // Reads the current sensor value
    }
  }
  
  // Handle ROS events
  nh.spinOnce();
  
}
