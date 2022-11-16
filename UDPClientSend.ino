
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>
#include "secrets.h"

#define A_BUTTON 4
#define B_BUTTON 3
#define POT_PIN A1

#define pot_enabled true
#define button_enabled true
#define wifi_enabled true

int status = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
IPAddress serverAddress(192, 168, 1, 200);

unsigned int localPort = 2390;
char packetBuffer[256];
char  clientMsg[25] = "";
WiFiUDP Udp;

int lastA_ButtonState = 0 ;
int A_BUTTONstate = 0;
int lastB_ButtonState = 0 ;
int B_BUTTONstate = 0;
int potValue = 0;
int lastPotValue = 0;
int sensitivityPOT = 35;

int convertRoll, convertPitch;
float lastAx = 0;
float lastAy = 0;
float lastAz = 0;
float sensitivityAcc = .05;

float Ax, Ay, Az;
boolean angleChanged = false;
float roll, pitch;

void setup() {

  Serial.begin(9600);

  GPIOSetUp();
  printRed();

  if (!IMU.begin()) {
    Serial.println("\IMU Chip not initializing...");
    while (1);
  }
  Serial.println("\IMU Chip setup completed...");

  if(wifi_enabled) {
    wifiSetUp();
    printBlue();
    wifiConnect();
  }


  Udp.begin( localPort);
  Serial.println("\nConnection completed...");

  printGreen();
  Serial.println("\Set up completed...");
}

void loop() {

  lastA_ButtonState = A_BUTTONstate;
  lastB_ButtonState = B_BUTTONstate;

  A_BUTTONstate = digitalRead(A_BUTTON);
  B_BUTTONstate = digitalRead(B_BUTTON);
  potValue = analogRead(POT_PIN);

  if (pot_enabled && (potValue > (lastPotValue + sensitivityPOT) || potValue <  (lastPotValue - sensitivityPOT))) {
    sprintf(clientMsg, "--POT%d", potValue);
    sendMsg(clientMsg) ;
    lastPotValue = potValue;
  }

  if (button_enabled) {
    if (A_BUTTONstate == HIGH && A_BUTTONstate != lastA_ButtonState)
    {
      strcpy(clientMsg, "--BUTTONA_ON");
      sendMsg(clientMsg) ;
    }

    if (A_BUTTONstate == LOW && A_BUTTONstate != lastA_ButtonState)
    {
      strcpy(clientMsg, "--BUTTONA_OFF");
      sendMsg(clientMsg) ;
    }

    if (B_BUTTONstate == HIGH && B_BUTTONstate != lastB_ButtonState)
    {
      strcpy(clientMsg, "--BUTTONB_ON");
      sendMsg(clientMsg) ;
    }

    if (B_BUTTONstate == LOW && B_BUTTONstate != lastB_ButtonState)
    {
      strcpy(clientMsg, "--BUTTONB_OFF");
      sendMsg(clientMsg) ;
    }
  }


  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
    angleChanged = false;
    if (Ay > (lastAy + sensitivityAcc) || Ay < (lastAy - sensitivityAcc)) {
      angleChanged = true;
    }
    else if (Az > (lastAz + sensitivityAcc) || Az < (lastAz - sensitivityAcc)) {
      angleChanged = true;
    }

    if (angleChanged) {
      //lastAx = Ax;
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

int calculateRoll(float y, float z) {
  roll = atan2(y, z) * 180 / PI;
  return roll * 10;
}

int calculatePitch(float x, float y, float z) {
  pitch = atan2(-x, sqrt(y * y + z * z)) * 180 / PI;
  return pitch * 10;
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
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

void GPIOSetUp() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(A_BUTTON, INPUT);
  pinMode(B_BUTTON, INPUT);
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
