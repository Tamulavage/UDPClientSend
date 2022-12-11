#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>
#include "secrets.h"

#define A_BUTTON 4
#define B_BUTTON 3
#define POT_PIN A1
#define heart_beat_pin   LED_BUILTIN

#define pot_enabled true
#define button_enabled true
#define wifi_enabled true

const long unsigned heart_beat_freq = 1000; // time(milliseconds) of heart beat frequency
long countOfRunTime = 0;
long unsigned heart_beat_on_off_time; // the time the LED is on and off - 1/2 frequency
long unsigned last_heart_beat_time;   // time in milliseconds of last heart beat status change
bool heart_beat_status = HIGH;        // current status of heart beat, start high


int status = WL_IDLE_STATUS;
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
const IPAddress serverAddress(192, 168, 1, 200);

unsigned int localPort = 2390;
//char packetBuffer[256];
char  clientMsg[25] = "";
WiFiUDP Udp;

int A_BUTTONstate = 0;
int B_BUTTONstate = 0;
int potValue = 0;
int lastPotValue = 0;
const int sensitivityPOT = 35;

int convertRoll, convertPitch;
float lastAy = 0;
float lastAz = 0;
const float sensitivityAcc = .05;

float Ax, Ay, Az;
boolean angleChanged = false;

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

  pinMode(heart_beat_pin, OUTPUT);
  heart_beat_on_off_time = heart_beat_freq / 2; // LED is on and off at 1/2 frequency time


  printGreen();
  Serial.println("\Set up completed...");
}

void loop () {
  
  heart_beat();
  int lastA_ButtonState = A_BUTTONstate;
  int lastB_ButtonState = B_BUTTONstate;

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
//  roll = atan2(y, z) * 180 / PI;
  return (atan2(y, z) * 180 / PI) * 10;
}

int calculatePitch(float x, float y, float z) {
//  pitch = atan2(-x, sqrt(y * y + z * z)) * 180 / PI;
  return (atan2(-x, sqrt(y * y + z * z)) * 180 / PI) * 10;
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

//  IPAddress ip = WiFi.localIP();
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
  Udp.flush();
  //delay(100);
  countOfRunTime=0;
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
  if (millis() - last_heart_beat_time >= heart_beat_on_off_time) {
    // time to swap status of the heart beat LED and update it
    last_heart_beat_time = millis();
    heart_beat_status = !heart_beat_status;           // invert current heart beat status value
    digitalWrite(heart_beat_pin, heart_beat_status);  // update LED with new status
    if(heart_beat_status){
      countOfRunTime++;
      Serial.println(countOfRunTime);
    }
  }
}
