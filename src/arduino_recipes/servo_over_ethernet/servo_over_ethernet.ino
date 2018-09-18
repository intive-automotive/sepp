
// Recipe for SEPP actuator board - arduino code for connecting actuators to SEPP
// 2018-09-18 by Andreas Lachenschmidt <a.lachenschmidt@web.de>
//
// Changelog:
//      2018-09-18 - initial commit

/* ============================================
SEPP actuator board code is placed under the MIT license
Copyright (c) 2018 Andreas Lachenschmidt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Servo.h>
#include <SPI.h> // needed for Arduino versions later than 0018
#include <Ethernet.h>

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#define PIN_9 9
#define PIN_10 10
#define PIN_11 11
#define PIN_13 13

#define PIN_MOTOR1 PIN_9
#define PIN_MOTOR2 PIN_13

#define DRIVE 2
#define REVERSE 3
#define BREAK 4
#define EMERGANCY_BREAK 5

#define STEER_VALUE_MAP_INPUT_MIN 0L
#define STEER_VALUE_MAP_INPUT_MAX 2000L
#define STEER_VALUE_MAP_OUTPUT_MIN 70L
#define STEER_VALUE_MAP_OUTPUT_MAX 120L

#define THRUST_REVERSE_MIN 96L
#define THRUST_REVERSE_MAX 98L
#define THRUST_FOREWARD_MIN 89L
#define THRUST_FOREWARD_MAX 84L

#define THRUST_REVERSE_MAP_INPUT_MIN 0L
#define THRUST_REVERSE_MAP_INPUT_MAX 2000L
#define THRUST_FOREWARD_MAP_INPUT_MIN 0L
#define THRUST_FOREWARD_MAP_INPUT_MAX 2000L

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress actuatorAdress(192, 168, 1, 179);
IPAddress server(192, 168, 1, 178);

// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

std_msgs::String ping_msg;

int16_t thrustValue = 0;
int16_t thrustMotorValue = 0;
int16_t steerValue = 0;
int16_t steerMotorValue = 0;
int16_t directionValue = 0;
bool emergencyStopValue = false;

int thrustReverseMax = 0;
int thrustReverseMin = 0;
int thrustForewardMax = 0;
int thrustForewardMin = 0;

// message parameters
uint16_t period = 1000;
uint32_t last_time = 0;

Servo motor1;
Servo motor2;

void messageCbSteer(const std_msgs::Int16 &int16_msg_steer)
{
  steerValue = int16_msg_steer.data;
}

void messageCbThrust(const std_msgs::Int16 &int16_msg_thrust)
{
  if (EMERGANCY_BREAK == directionValue || BREAK == directionValue)
  {
    thrustValue = 90;
  }
  else if (DRIVE == directionValue)
  {
    thrustValue = map(int16_msg_thrust.data, THRUST_FOREWARD_MAP_INPUT_MIN, THRUST_FOREWARD_MAP_INPUT_MAX, thrustForewardMin, thrustForewardMax);
  }
  else if (REVERSE == directionValue)
  {
    thrustValue = map(int16_msg_thrust.data, THRUST_REVERSE_MAP_INPUT_MIN, THRUST_REVERSE_MAP_INPUT_MAX, thrustReverseMin, thrustReverseMax);
  }
}

void messageCbDirection(const std_msgs::Int16 &int16_msg_direction)
{
  directionValue = int16_msg_direction.data;
}

void messageCbEmergancyStop(const std_msgs::Bool &bool_msg_emergancyStop)
{
  emergencyStopValue = bool_msg_emergancyStop.data;
}

ros::Subscriber<std_msgs::Int16> sub_thrust("decided_thrust", &messageCbThrust);
ros::Subscriber<std_msgs::Int16> sub_steer("decided_steer", &messageCbSteer);
ros::Subscriber<std_msgs::Int16> sub_direction("decided_direction", &messageCbDirection);
ros::Subscriber<std_msgs::Bool> sub_emergancyStop("emergancyStop", &messageCbEmergancyStop);
ros::Publisher pub_actuatorPing("actuator_ping", &ping_msg);

char ping_message_string[23] = "actuator borad active!";

void setup()
{
  // start the Ethernet and UDP:
  Ethernet.begin(mac, actuatorAdress);

  Serial.begin(9600);
  delay(1000);

  // set pin of servo 1
  motor1.attach(PIN_MOTOR1);
  // set pin of servo 2
  motor2.attach(PIN_MOTOR2);

  delay(200);

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  while (!nh.connected())
  {
    nh.spinOnce();
  }

  if (!nh.getParam("/decided_thrust/thrustReverseMax", &thrustReverseMax))
  {
    thrustReverseMax = THRUST_REVERSE_MAX;
    ;
    Serial.println("Parameter nicht gelesen");
  }
  Serial.println(thrustReverseMax);

  if (!nh.getParam("/decided_thrust/thrustReverseMin", &thrustReverseMin))
  {
    thrustReverseMin = THRUST_REVERSE_MIN;
  }

  if (!nh.getParam("/decided_thrust/thrustForewardMax", &thrustForewardMax))
  {
    thrustForewardMax = THRUST_FOREWARD_MAX;
  }

  if (!nh.getParam("/decided_thrust/thrustForewardMin", &thrustForewardMin))
  {
    thrustForewardMin = THRUST_FOREWARD_MIN;
  }

  nh.subscribe(sub_thrust);
  nh.subscribe(sub_steer);
  nh.subscribe(sub_direction);
  nh.subscribe(sub_emergancyStop);
  nh.advertise(pub_actuatorPing);
}

void loop()
{

  if (millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      ping_msg.data = ping_message_string;
      pub_actuatorPing.publish(&ping_msg);
      Serial.println(thrustValue);
    }
    else
    {
      Serial.println("Not Connected");
    }
  }
  steerMotorValue = map(steerValue, STEER_VALUE_MAP_INPUT_MIN, STEER_VALUE_MAP_INPUT_MAX, STEER_VALUE_MAP_OUTPUT_MIN, STEER_VALUE_MAP_OUTPUT_MAX);
  motor1.write(steerMotorValue);

  thrustMotorValue = thrustValue;
  motor2.write(thrustMotorValue);
  Serial.println(thrustMotorValue);

  nh.spinOnce();
  delay(1);
}
