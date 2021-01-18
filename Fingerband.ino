#include "MPU9250.h"
#include "eeprom_utils.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

MPU9250 mpu;
static uint32_t prev_sample_ms;

char ssid[] = "WiFimodem-F66B"; // I don't care. You don't know where I am!
char pass[] = "2hc5hb1ee1";

WiFiUDP Udp;
const IPAddress outIp(192, 168, 1, 11);
const unsigned int outPort = 9999;
const unsigned int localPort = 8888;
OSCErrorCode error;

int meanCounter = 0;
float offsets[3] = {0,0,0};
float prevVal = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Hello!");
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);

  digitalWrite(D4, LOW);
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    digitalWrite(D4, HIGH);
    delay(100);
    digitalWrite(D4, LOW);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Udp.begin(localPort);
  sendOSCMessage("Booting...");
  if (!mpu.setup(0x68)) {
    while (1) {
      sendOSCMessage("MPU connection failed! Restarting ...");
      delay(5000);
      ESP.reset(); // Let's just reset and see if it fixes the issue
    }
  }
  EEPROM.begin(0x80);
  delay(5000);
  digitalWrite(D4, HIGH);
  Serial.println("EEPROM start");
  if (!isCalibrated()) {
      Serial.println("Need Calibration!!");
  }
  Serial.println("EEPROM calibration value is : ");
  printCalibration();
  Serial.println("Loaded calibration value is : ");
  loadCalibration(&offsets[0]);
  prev_sample_ms = millis();
  sendOSCMessage("Ready to play");
  flashLED(300, 50);
}

void loop() {
  if (mpu.update()) {
    if (millis() > prev_sample_ms + 10) {
      OSCMessage msg("/xyz");
      
      // Handle xAxis values ----------------------
      float xAxis = mpu.getEulerX();
      if (xAxis > 0)
      {
        prevVal = xAxis;
        xAxis = 0;
      }
      else if (xAxis < 0)
      {
        if (prevVal > 0) 
        {
          offsets[0] = prevVal;
        }
        prevVal = xAxis;
        Serial.println(xAxis);
        Serial.println(offsets[0]);
        xAxis += offsets[0];
        Serial.println(xAxis);
        Serial.println("-------------");
      }
      // ------------------------------------------
      
      msg.add(xAxis).add(mpu.getEulerY()).add(mpu.getEulerZ() - offsets[2]);
      Udp.beginPacket(outIp, outPort);
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
      prev_sample_ms = millis();
    }
  }
  OSCMessage msg;
  int size = Udp.parsePacket();
  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/calibrate", calibrate);
      Serial.println("Calibration invoked");
    } else {
      error = msg.getError();
      Serial.print("Error: ");
      Serial.println(error);
    }
  }
}

void sendOSCMessage(char *message) 
{
  Serial.println(message);
  OSCMessage msg("/msg");
  msg.add(message);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void calibrate(OSCMessage &msg) {
  sendOSCMessage("Calibrating...");
  flashLED(1000, 200);
  mpu.calibrateAccelGyro();
  flashLED(1000, 200);
  uint32_t startTime = millis();
  float xSum = 0;
  float ySum = 0;
  float zSum = 0;
  while (startTime + 1000 > millis())
  {
    if (mpu.update()) {
      xSum += mpu.getEulerX();
      ySum += mpu.getEulerY();
      zSum += mpu.getEulerZ();
      meanCounter++;
    }
  }
//  offsets[0] = xSum / meanCounter;
  offsets[1] = ySum / meanCounter;
  offsets[2] = zSum / meanCounter;
  Serial.print("Mean counter: ");
  Serial.println(meanCounter);
  saveCalibration(&offsets[0]);
  printCalibration(); 
  flashLED(500, 100);
  String outMsg = "Calibration done. XYZ offsets are: ";
  outMsg += offsets[0];
  outMsg += ", ";
  outMsg += offsets[1];
  outMsg += ", ";
  outMsg += offsets[2];
  int len = outMsg.length();
  char charray[len];
  outMsg.toCharArray(charray, len);
  sendOSCMessage(charray);
  meanCounter = 0;
}

void flashLED(int time_ms, int speed_ms)
{
  unsigned int ledState = HIGH;
  for (int i = 0; i < time_ms / speed_ms; i++)
  {
    digitalWrite(D4, ledState);
    ledState = !ledState;
    delay(speed_ms);
  }
  digitalWrite(D4, LOW);
}
