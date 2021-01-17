#include "MPU9250.h"
#include "eeprom_utils.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

MPU9250 mpu;
static uint32_t prev_sample_ms;

char ssid[] = "WiFimodem-F66B";
char pass[] = "2hc5hb1ee1";

WiFiUDP Udp;
const IPAddress outIp(192, 168, 1, 11);
const unsigned int outPort = 9999;
const unsigned int localPort = 8888;
OSCErrorCode error;

int meanCounter = 0;

float xAxisOffset = 0;
float yAxisOffset = 0;
float zAxisOffset = 0;

float prevVal = 0;
unsigned int timer = 0;
unsigned int period;
int frequency;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Hello!");
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);
  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. ");
      delay(5000);
    }
  }
  EEPROM.begin(0x80);
  delay(5000);
  digitalWrite(D4, HIGH);
  setupEEPROM();

  prev_sample_ms = millis();

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
}

void loop() {
  if (mpu.update()) {
    if (millis() > prev_sample_ms + 10) {
      OSCMessage msg("/xyz");
      msg.add(mpu.getEulerX() - xAxisOffset).add(GetYAxisSignal()).add(mpu.getEulerZ() - zAxisOffset);
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
    } else {
      error = msg.getError();
      Serial.print("Error: ");
      Serial.println(error);
    }
  }
}

float GetYAxisSignal()
{
  float newData = mpu.getEulerY();
  float res = 0;
  float sensitivity = 0.5;
  if (newData > prevVal + sensitivity)
    res = 1;
  else if (newData < prevVal - sensitivity)
    res = -1;
  prevVal = newData;
  return res;
}

void calibrate(OSCMessage &msg) {
  flashLED(1000, 200);
  mpu.calibrateAccelGyro();
  saveCalibration();
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
  xAxisOffset = xSum / meanCounter;
  yAxisOffset = ySum / meanCounter;
  zAxisOffset = zSum / meanCounter;
  Serial.print("Mean counter: ");
  Serial.println(meanCounter);
  Serial.print("X offset = ");
  Serial.println(xAxisOffset);
  Serial.print("Y offset = ");
  Serial.println(yAxisOffset);
  Serial.print("Z offset = ");
  Serial.println(zAxisOffset);
  flashLED(1000, 200);
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
