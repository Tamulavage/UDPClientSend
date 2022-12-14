#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DSOX.h>
#include "secrets.h"

#define A_BUTTON_PIN 4
#define B_BUTTON_PIN 3
#define POT_PIN A1
#define HEARTBEAT_PIN   LED_BUILTIN

#define POT_ENABLED true
#define BUTTONS_ENABLED true
#define WIFI_ENABLED true
#define IMU_ENABLED true

const long unsigned heartBeatOnTime  = 500 ;// time(milliseconds) of heart beat time
long unsigned lastHeartBeatTime;   // time in milliseconds of last heart beat status change
bool heartBeatStatus = HIGH;        // current status of heart beat, start high

int status = WL_IDLE_STATUS;
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
const IPAddress serverAddress(192, 168, 1, 200);

const unsigned int localPort = 2390;
char  clientMsg[25] = "";
WiFiUDP Udp;

int buttonAState = 0;
int buttonBState = 0;
int potValue = 0;
int lastPotValue = 0;
const int sensitivityPOT = 35;

int convertRoll, convertPitch;
float lastAy = 0;
float lastAz = 0;
const float sensitivityAcc = .15;

float Ax, Ay, Az;
bool angleChanged = false;

void setup() {

  Serial.begin(9600);

  GPIOSetUp();
  printRed();

  if (IMU_ENABLED) {
    IMUSetup();
  }

  if (WIFI_ENABLED) {
    wifiSetUp();
    printBlue();
    wifiConnect();
  }

  Udp.begin( localPort);

  printGreen();
  Serial.println("\Set up completed...");
}

void loop () {

  heart_beat();

  if (POT_ENABLED) {
    processPOT();
  }

  if (BUTTONS_ENABLED) {
    processButtons();
  }

  if (IMU_ENABLED) {
    processIMU();
  }
}

void processIMU() {
  if (IMU.accelerationAvailable()) {
    delay(30); // addd or overloads chip which would freeze aplication
    IMU.readAcceleration(Ax, Ay, Az);
    angleChanged = false;

    if (Ay > (lastAy + sensitivityAcc) || Ay < (lastAy - sensitivityAcc)) {
      angleChanged = true;
    }
    else if (Az > (lastAz + sensitivityAcc) || Az < (lastAz - sensitivityAcc)) {
      angleChanged = true;
    }

    if (angleChanged) {
      lastAy = Ay;
      lastAz = Az;

      convertRoll = calculateRoll(Ay, Az);
      sprintf(clientMsg, "--ANGLE-ROLL%d", convertRoll );
      sendMsg(clientMsg) ;

      convertPitch = calculatePitch(Ax, Ay, Az);
      sprintf(clientMsg, "--ANGLE-PITCH%d", convertPitch );
      sendMsg(clientMsg) ;
    }
  }
}

void processPOT() {
  potValue = analogRead(POT_PIN);

  if (potValue > (lastPotValue + sensitivityPOT) || potValue <  (lastPotValue - sensitivityPOT)) {
    sprintf(clientMsg, "--POT%d", potValue);
    sendMsg(clientMsg) ;
    lastPotValue = potValue;
  }
}

void processButtons() {

  int lastbuttonAState = buttonAState;
  int lastbuttonBState = buttonBState;

  buttonAState = digitalRead(A_BUTTON_PIN);
  buttonBState = digitalRead(B_BUTTON_PIN);

  if (buttonAState != lastbuttonAState)
  {
    if (buttonAState == HIGH) {
      strcpy(clientMsg, "--BUTTONA_ON");
    }
    else {
      strcpy(clientMsg, "--BUTTONA_OFF");
    }
    sendMsg(clientMsg) ;
  }

  if (buttonBState != lastbuttonBState)
  {
    if (buttonBState == HIGH) {
      strcpy(clientMsg, "--BUTTONB_ON");
    }
    else {
      strcpy(clientMsg, "--BUTTONB_OFF");
    }
    sendMsg(clientMsg) ;
  }
}

int calculateRoll(float y, float z) {
  return (atan2(y, z) * 180 / PI) * 10;
}

int calculatePitch(float x, float y, float z) {
  return (atan2(-x, sqrt(y * y + z * z)) * 180 / PI) * 10;
}


void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  Serial.print("IP Address: ");
  Serial.println( WiFi.localIP());

  // print the received signal strength:
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void wifiConnect() {

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();
}

void IMUSetup() {
  if (!IMU.begin()) {
    Serial.println("\IMU Chip not initializing...");
    while (1);
  }
  Serial.println("\IMU Chip setup completed...");
}

void GPIOSetUp() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);
  
  pinMode(A_BUTTON_PIN, INPUT);
  pinMode(B_BUTTON_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
}

void wifiSetUp() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
}

void sendMsg(char clientMsg[]) {
  Serial.println(clientMsg);
  Udp.beginPacket(serverAddress, localPort);
  Udp.write(clientMsg);
  Udp.endPacket();
  Udp.flush();
  delay(20); // minor delaty to not overload chip/udp/receiver 
}

void printGreen() {
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDR, LOW);
}

void printBlue() {
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);
}

void printRed() {
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDR, HIGH);
}

void heart_beat() {
  if (millis() - lastHeartBeatTime >= heartBeatOnTime) {
    // time to swap status of the heart beat LED and update it
    lastHeartBeatTime = millis();
    heartBeatStatus = !heartBeatStatus;           // invert current heart beat status value
    digitalWrite(HEARTBEAT_PIN, heartBeatStatus);  // update LED with new status
  }
}
