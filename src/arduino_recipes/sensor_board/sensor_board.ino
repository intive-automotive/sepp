// Recipe for SEPP sensor board - arduino code for connecting sensors to SEPP
// 2018-09-18 by Andreas Lachenschmidt <a.lachenschmidt@web.de>
//
// Changelog:
//      2018-09-18 - initial commit

/* ============================================
SEPP sensor board code is placed under the MIT license
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
#include <SPI.h> // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include "SR04.h"

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

// ROS includes
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

// gyro includes
#include "Wire.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#define PIN_38 38
#define PIN_39 39
#define PIN_40 40
#define PIN_41 41
#define PIN_42 42
#define PIN_43 43
#define PIN_44 44
#define PIN_45 45
#define PIN_46 46
#define PIN_47 47
#define PIN_48 48
#define PIN_49 49
#define PIN_50 30
#define PIN_51 31
#define PIN_32 32
#define PIN_33 33

// front
#define TRIG_PIN_1 PIN_38
#define ECHO_PIN_1 PIN_39
#define TRIG_PIN_2 PIN_42
#define ECHO_PIN_2 PIN_43
#define TRIG_PIN_3 PIN_46
#define ECHO_PIN_3 PIN_47
#define TRIG_PIN_4 PIN_50
#define ECHO_PIN_4 PIN_51

// back
#define TRIG_PIN_5 PIN_40
#define ECHO_PIN_5 PIN_41
#define TRIG_PIN_6 PIN_44
#define ECHO_PIN_6 PIN_45
#define TRIG_PIN_7 PIN_48
#define ECHO_PIN_7 PIN_49
#define TRIG_PIN_8 PIN_32
#define ECHO_PIN_8 PIN_33

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
IPAddress sensorAdress(192, 168, 1, 177);
// Set the rosserial socket server IP address
IPAddress server(192, 168, 1, 178);

// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a publisher for ultrasonic front left
sensor_msgs::Range range_msg_uss_fl;
ros::Publisher uss_fl("uss_fl", &range_msg_uss_fl);

// Make a publisher for ultrasonic front middle left
sensor_msgs::Range range_msg_uss_fml;
ros::Publisher uss_fml("uss_fml", &range_msg_uss_fml);

// Make a publisher for ultrasonic front middle right
sensor_msgs::Range range_msg_uss_fmr;
ros::Publisher uss_fmr("uss_fmr", &range_msg_uss_fmr);

// Make a publisher for ultrasonic front right
sensor_msgs::Range range_msg_uss_fr;
ros::Publisher uss_fr("uss_fr", &range_msg_uss_fr);

// Make a publisher for ultrasonic back left
sensor_msgs::Range range_msg_uss_bl;
ros::Publisher uss_bl("uss_bl", &range_msg_uss_bl);

// Make a publisher for ultrasonic back middle left
sensor_msgs::Range range_msg_uss_bml;
ros::Publisher uss_bml("uss_bml", &range_msg_uss_bml);

// Make a publisher for ultrasonic back middle right
sensor_msgs::Range range_msg_uss_bmr;
ros::Publisher uss_bmr("uss_bmr", &range_msg_uss_bmr);

// Make a publisher for ultrasonic back right
sensor_msgs::Range range_msg_uss_br;
ros::Publisher uss_br("uss_br", &range_msg_uss_br);

// front
SR04 ss1 = SR04(ECHO_PIN_1, TRIG_PIN_1);
SR04 ss2 = SR04(ECHO_PIN_2, TRIG_PIN_2);
SR04 ss3 = SR04(ECHO_PIN_3, TRIG_PIN_3);
SR04 ss4 = SR04(ECHO_PIN_4, TRIG_PIN_4);

// back
SR04 ss5 = SR04(ECHO_PIN_5, TRIG_PIN_5);
SR04 ss6 = SR04(ECHO_PIN_6, TRIG_PIN_6);
SR04 ss7 = SR04(ECHO_PIN_7, TRIG_PIN_7);
SR04 ss8 = SR04(ECHO_PIN_8, TRIG_PIN_8);

// header ultrasound
uint32_t seqCounter = 0;
uint8_t ssSensorIdentifier = 0;
const uint8_t MAX_SSSENSORIDENTIFIER_INDEX = 7;

// message parameters
uint16_t period = 150;
uint32_t last_time = 0;
// front
char frame_id_USS1[] = "/uss1";
char frame_id_USS2[] = "/uss2";
char frame_id_USS3[] = "/uss3";
char frame_id_USS4[] = "/uss4";
// back
char frame_id_USS5[] = "/uss5";
char frame_id_USS6[] = "/uss6";
char frame_id_USS7[] = "/uss7";
char frame_id_USS8[] = "/uss8";

// front
float ss1Distance = 0;
float ss2Distance = 0;
float ss3Distance = 0;
float ss4Distance = 0;

// back
float ss5Distance = 0;
float ss6Distance = 0;
float ss7Distance = 0;
float ss8Distance = 0;

void setup()
{
  // start the Ethernet and UDP:
  Ethernet.begin(mac, sensorAdress);

  Serial.begin(9600);
  delay(1000);

  Serial.println("Setup...");

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Start to be polite
  // front
  nh.advertise(uss_fl);
  nh.advertise(uss_fml);
  nh.advertise(uss_fmr);
  nh.advertise(uss_fr);
  // back
  nh.advertise(uss_bl);
  nh.advertise(uss_bml);
  nh.advertise(uss_bmr);
  nh.advertise(uss_br);

  // define uss data front left
  range_msg_uss_fl.radiation_type = 0;
  range_msg_uss_fl.field_of_view = 0.15;
  range_msg_uss_fl.min_range = 0.001;
  range_msg_uss_fl.max_range = 3.5;
  range_msg_uss_fl.range = ss1Distance / 100.0;
  range_msg_uss_fl.header.stamp = nh.now();
  range_msg_uss_fl.header.frame_id = frame_id_USS1;
  range_msg_uss_fl.header.seq = seqCounter;

  // define uss data front middle left
  range_msg_uss_fml.radiation_type = 0;
  range_msg_uss_fml.field_of_view = 0.15;
  range_msg_uss_fml.min_range = 0.001;
  range_msg_uss_fml.max_range = 3.5;
  range_msg_uss_fml.range = ss2Distance / 100.0;
  range_msg_uss_fml.header.stamp = nh.now();
  range_msg_uss_fml.header.frame_id = frame_id_USS2;
  range_msg_uss_fml.header.seq = seqCounter;

  // define uss data front middle right
  range_msg_uss_fmr.radiation_type = 0;
  range_msg_uss_fmr.field_of_view = 0.15;
  range_msg_uss_fmr.min_range = 0.001;
  range_msg_uss_fmr.max_range = 3.5;
  range_msg_uss_fmr.range = ss3Distance / 100.0;
  range_msg_uss_fmr.header.stamp = nh.now();
  range_msg_uss_fmr.header.frame_id = frame_id_USS3;
  range_msg_uss_fmr.header.seq = seqCounter;

  // define uss data front right
  range_msg_uss_fr.radiation_type = 0;
  range_msg_uss_fr.field_of_view = 0.15;
  range_msg_uss_fr.min_range = 0.001;
  range_msg_uss_fr.max_range = 3.5;
  range_msg_uss_fr.range = ss4Distance / 100.0;
  range_msg_uss_fr.header.stamp = nh.now();
  range_msg_uss_fr.header.frame_id = frame_id_USS4;
  range_msg_uss_fr.header.seq = seqCounter;

  // define uss data back left
  range_msg_uss_bl.radiation_type = 0;
  range_msg_uss_bl.field_of_view = 0.15;
  range_msg_uss_bl.min_range = 0.001;
  range_msg_uss_bl.max_range = 3.5;
  range_msg_uss_bl.range = ss5Distance / 100.0;
  range_msg_uss_bl.header.stamp = nh.now();
  range_msg_uss_bl.header.frame_id = frame_id_USS5;
  range_msg_uss_bl.header.seq = seqCounter;

  // define uss data back middle left
  ss6Distance = ss6.Distance();
  range_msg_uss_bml.radiation_type = 0;
  range_msg_uss_bml.field_of_view = 0.15;
  range_msg_uss_bml.min_range = 0.001;
  range_msg_uss_bml.max_range = 3.5;
  range_msg_uss_bml.range = ss6Distance / 100.0;
  range_msg_uss_bml.header.stamp = nh.now();
  range_msg_uss_bml.header.frame_id = frame_id_USS6;
  range_msg_uss_bml.header.seq = seqCounter;

  // define uss data back middle right
  range_msg_uss_bmr.radiation_type = 0;
  range_msg_uss_bmr.field_of_view = 0.15;
  range_msg_uss_bmr.min_range = 0.001;
  range_msg_uss_bmr.max_range = 3.5;
  range_msg_uss_bmr.range = ss7Distance / 100.0;
  range_msg_uss_bmr.header.stamp = nh.now();
  range_msg_uss_bmr.header.frame_id = frame_id_USS7;
  range_msg_uss_bmr.header.seq = seqCounter;

  // define uss data back right
  range_msg_uss_br.radiation_type = 0;
  range_msg_uss_br.field_of_view = 0.15;
  range_msg_uss_br.min_range = 0.001;
  range_msg_uss_br.max_range = 3.5;
  range_msg_uss_br.range = ss8Distance / 100.0;
  range_msg_uss_br.header.stamp = nh.now();
  range_msg_uss_br.header.frame_id = frame_id_USS8;
  range_msg_uss_br.header.seq = seqCounter;
}

void loop()
{
  if (millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
        // send uss data front left
        ss1Distance = ss1.Distance();
        range_msg_uss_fl.range = ss1Distance / 100.0;
        range_msg_uss_fl.header.seq = seqCounter;
        range_msg_uss_fl.header.stamp = nh.now();
        uss_fl.publish(&range_msg_uss_fl);

        // define uss data front middle left
        ss2Distance = ss2.Distance();
        range_msg_uss_fml.range = ss2Distance / 100.0;
        range_msg_uss_fml.header.seq = seqCounter;
        range_msg_uss_fml.header.stamp = nh.now();
        uss_fml.publish(&range_msg_uss_fml);

        // send uss data front middle right
        ss3Distance = ss3.Distance();
        range_msg_uss_fmr.range = ss3Distance / 100.0;
        range_msg_uss_fmr.header.stamp = nh.now();
        range_msg_uss_fmr.header.seq = seqCounter;
        uss_fmr.publish(&range_msg_uss_fmr);

        // send uss data front right
        ss4Distance = ss4.Distance();
        range_msg_uss_fr.range = ss4Distance / 100.0;
        range_msg_uss_fr.header.stamp = nh.now();
        range_msg_uss_fr.header.seq = seqCounter;
        uss_fr.publish(&range_msg_uss_fr);

        // send uss data front right
        ss5Distance = ss5.Distance();
        range_msg_uss_bl.range = ss5Distance / 100.0;
        range_msg_uss_bl.header.stamp = nh.now();
        range_msg_uss_bl.header.seq = seqCounter;
        uss_bl.publish(&range_msg_uss_bl);
 
        // send uss data back middle left
        ss6Distance = ss6.Distance();
        range_msg_uss_bml.range = ss6Distance / 100.0;
        range_msg_uss_bml.header.stamp = nh.now();
        range_msg_uss_bml.header.seq = seqCounter;
        uss_bml.publish(&range_msg_uss_bml);

        // send uss data back middle right
        ss7Distance = ss7.Distance();
        range_msg_uss_bmr.range = ss7Distance / 100.0;
        range_msg_uss_bmr.header.stamp = nh.now();
        range_msg_uss_bmr.header.seq = seqCounter;
        uss_bmr.publish(&range_msg_uss_bmr);

        // send uss data back right
        ss8Distance = ss8.Distance();
        range_msg_uss_br.range = ss8Distance / 100.0;
        range_msg_uss_br.header.stamp = nh.now();
        range_msg_uss_br.header.seq = seqCounter;
        uss_br.publish(&range_msg_uss_br);
    }
    else
    {
      Serial.println("Not Connected");
    }
  }
  nh.spinOnce();
  delay(1);
  seqCounter++; //overrun allowed
}
