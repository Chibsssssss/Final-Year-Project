#define BLYNK_TEMPLATE_ID "TMPL2_Rjbd6Ir"
#define BLYNK_TEMPLATE_NAME "Home Automation FYP"
#define BLYNK_AUTH_TOKEN "1ZMDHWdICjWk8GvF5ZvIkTFnZeaLCkrB"

#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <SPI.h>
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>

#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>  
#include <NTPClient.h>
#include <WiFiUdp.h>

// Replace with your network credentials
char ssid[] = "CHIBS-MIFI";
char pass[] = "Nzubeblaise1.";
char auth[] = "1ZMDHWdICjWk8GvF5ZvIkTFnZeaLCkrB";

// Define sensor and actuator pins
#define DHTPIN 13 
#define DHTTYPE DHT22
#define MOTION_PIN 4
#define GAS_SENSOR_PIN 36
#define CURRENT_SENSOR_PIN 39
#define CURRENT_SENSOR_PIN2 34
#define CURRENT_SENSOR_PIN3 33
#define LDR_PIN 35
#define FLAME_SENSOR_PIN 16
#define DOOR_LOCK_PIN 12

#define BULB1_RELAY_PIN 25
#define BULB2_RELAY_PIN 26
#define APPLIANCE_RELAY_PIN 32
#define FAN_RELAY_PIN 14
#define ALARM_RELAY_PIN 27

byte readCard[4];
String MasterTag = "E34D8DFC";	// REPLACE this Tag ID with your Tag ID!!!
String tagID = "";

// Create sensor and actuator objects
DHT dht(DHTPIN, DHTTYPE);

MFRC522DriverPinSimple ss_pin(5); // Configurable, see typical pin layout above.
MFRC522DriverSPI driver{ss_pin}; // Create SPI driver.
MFRC522 rfid{driver};  // Create MFRC522 instance.

const float VREF = 3.3; // Reference voltage of the ESP32
const int ADC_RESOLUTION = 4095; // ESP32 has a 12-bit ADC
const float SENSOR_SENSITIVITY = 0.066; // 
// Offset voltage when there is no current flowing
const float NO_CURRENT_VOLTAGE = VREF / 2; 

int lockState = 0;  // State of the solenoid lock (true = open, false = closed)
int faceUnlockState = 0;
int doorLockState = 0;  // 0 = locked, 1 = unlocked
int faceUnlockEnabled = 0;  // 0 = disabled, 1 = enabled

BlynkTimer timer;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000); // Adjust time zone offset if necessary
int currentHour = 0; // Variable to store the current hour

// Structure to send/receive data
typedef struct struct_message {
  float temp;
  float light;
  bool motion;
  int gas_level;
  int hour;
  float current;
  float lightCommand;
  float fanCommand;
  float alarmCommand;
  float energyCommand;
} struct_message;

struct_message sensorData;

// Peer MAC address of Fuzzy Logic ESP32 (replace with actual address)
uint8_t peerAddress[] = {0xa0, 0xa3, 0xb3, 0x2f, 0x93, 0x00};

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function when data is received
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) { 
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  
  Serial.println("Actuator commands received:");
  Serial.printf("Light: %.2f, Fan: %.2f, Alarm: %.2f, Energy: %.2f\n", 
                sensorData.lightCommand, sensorData.fanCommand, 
                sensorData.alarmCommand, sensorData.energyCommand);
}

void sendSensorData() {
  // Read sensor data
  sensorData.temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  int lightValue = analogRead(LDR_PIN);
  sensorData.light = map(lightValue, 0, 4095, 4095, 0);
  sensorData.motion = digitalRead(MOTION_PIN);
  sensorData.gas_level = analogRead(GAS_SENSOR_PIN);

  float currentSensorValue = analogRead(CURRENT_SENSOR_PIN);
  float sensorVoltage = (currentSensorValue * VREF) / ADC_RESOLUTION;
  sensorData.current = (sensorVoltage - NO_CURRENT_VOLTAGE) * 0.707 / SENSOR_SENSITIVITY;

  float currentSensorValue2 = analogRead(CURRENT_SENSOR_PIN2);
  // Convert the analog reading to voltage
  float sensorVoltage2 = (currentSensorValue2 * VREF) / ADC_RESOLUTION;
  float applianceCurrent2 = (sensorVoltage2 - NO_CURRENT_VOLTAGE) * 0.707 / SENSOR_SENSITIVITY;
  float appliancePower2 = applianceCurrent2*220;

  float currentSensorValue3 = analogRead(CURRENT_SENSOR_PIN3);
  // Convert the analog reading to voltage
  float sensorVoltage3 = (currentSensorValue3 * VREF) / ADC_RESOLUTION;
  float applianceCurrent3 = (sensorVoltage3 - NO_CURRENT_VOLTAGE) * 0.707 / SENSOR_SENSITIVITY;
  float appliancePower3 = applianceCurrent3*220;

  sensorData.hour = timeClient.getHours();

  int motionSensorState = digitalRead(MOTION_PIN);  // Read the state of the motion sensor
  int flameSensorState = digitalRead(FLAME_SENSOR_PIN);


  // Send sensor data to Fuzzy Logic ESP32
  esp_err_t result = esp_now_send(peerAddress, (const uint8_t *) &sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("Sent sensor data");
  } else {
    Serial.println("Error sending sensor data");
  }

  // Update Blynk interface
  Blynk.virtualWrite(V1, sensorData.temp);
  Blynk.virtualWrite(V2, sensorData.light);
  Blynk.virtualWrite(V3, sensorData.current * 220);
  Blynk.virtualWrite(V4, flameSensorState);
  Blynk.virtualWrite(V5, humidity);
  Blynk.virtualWrite(V6, sensorData.gas_level);
  Blynk.virtualWrite(V8, doorLockState);
  Blynk.virtualWrite(V10, faceUnlockState);
  Blynk.virtualWrite(V11, appliancePower2);
  Blynk.virtualWrite(V12, appliancePower3);
  Blynk.virtualWrite(V17, sensorData.lightCommand);
  Blynk.virtualWrite(V18, sensorData.fanCommand);
  Blynk.virtualWrite(V0, sensorData.alarmCommand);
  Blynk.virtualWrite(V20, sensorData.energyCommand);
    
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);  // Keep this for connecting WiFi only
  }
  Serial.println("\nWiFi connected at IP ");
  Serial.print(WiFi.localIP());

  Blynk.begin(auth, ssid, pass);
  timeClient.begin();

  // Initialize sensors and actuators
  dht.begin();
  pinMode(MOTION_PIN, INPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(DOOR_LOCK_PIN, OUTPUT);
  pinMode(BULB1_RELAY_PIN, OUTPUT);
  pinMode(BULB2_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(APPLIANCE_RELAY_PIN, OUTPUT);

  digitalWrite(DOOR_LOCK_PIN, LOW);
  digitalWrite(BULB1_RELAY_PIN, LOW);
  digitalWrite(BULB2_RELAY_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(APPLIANCE_RELAY_PIN, LOW);

  pinMode(ALARM_RELAY_PIN, OUTPUT);
  SPI.begin();        
  rfid.PCD_Init(); 

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Set timer to call sendSensorData every 2 seconds
  timer.setInterval(2000L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();  // Runs the Blynk timer
  timeClient.update();

  // Check RFID
  while (getID()) {
    if (tagID == MasterTag && faceUnlockEnabled == 0) {
      Serial.println("RFID MATCH, Door OPENED");
      doorLockState = 1;
      digitalWrite(DOOR_LOCK_PIN, HIGH);  // Unlock the door
    } else {
      Serial.println("RFID NO MATCH or Face Unlock ACTIVE, Door remains CLOSED");
      doorLockState = 0;
      digitalWrite(DOOR_LOCK_PIN, LOW);  // Lock the door
    }
  }
}

boolean getID() 
{
  // Getting ready for Reading PICCs
  if ( ! rfid.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
  return false;
  }
  if ( ! rfid.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
  return false;
  }
  tagID = "";
  for ( uint8_t i = 0; i < 4; i++) { // The MIFARE PICCs that we use have 4 byte UID
  //readCard[i] = mfrc522.uid.uidByte[i];
  tagID.concat(String(rfid.uid.uidByte[i], HEX)); // Adds the 4 bytes in a single String variable
  }
  tagID.toUpperCase();
  rfid.PICC_HaltA(); // Stop reading
  return true;
}

//Read Face Unlock State from ESP32CAM
BLYNK_WRITE(V10) {
  int faceUnlockState = param.asInt();
  
  if (faceUnlockState == 1) {
    Serial.println("Face Unlock ENABLED, Door OPENED");
    faceUnlockEnabled = 1;  // Face unlock is now active
    doorLockState = 1;  // Set door to unlocked state
    digitalWrite(DOOR_LOCK_PIN, HIGH);  // Unlock the door
  } else {
    Serial.println("Face Unlock DISABLED, Control released to RFID/Blynk");
    faceUnlockEnabled = 0;  // Face unlock is now inactive
    // We don't lock/close the door immediately; RFID/Blynk can take over.
  }
}

// Function to control Bulb 1
BLYNK_WRITE(V13) {
  int switchState = param.asInt();
  digitalWrite(BULB1_RELAY_PIN, switchState ? HIGH : LOW);
  Serial.println(switchState ? "Bulb 1 set HIGH" : "Bulb 1 set LOW");
}

// Function to control Bulb 2
BLYNK_WRITE(V14) {
  int switchState = param.asInt();
  digitalWrite(BULB2_RELAY_PIN, switchState ? HIGH : LOW);
  Serial.println(switchState ? "Bulb 2 set HIGH" : "Bulb 2 set LOW");
}

// Function to control Fan
BLYNK_WRITE(V15) {
  int switchState = param.asInt();
  digitalWrite(FAN_RELAY_PIN, switchState ? HIGH : LOW);
  Serial.println(switchState ? "Fan set HIGH" : "Fan set LOW");
}

// Function to control Appliance
BLYNK_WRITE(V16) {
  int switchState = param.asInt();
  digitalWrite(APPLIANCE_RELAY_PIN, switchState ? HIGH : LOW);
  Serial.println(switchState ? "Appliance set HIGH" : "Appliance set LOW");
}

BLYNK_WRITE(V8) {
  int switchState = param.asInt();
  
  if (faceUnlockEnabled == 0) {  // Only allow control if face unlock is not active
    if (switchState == 1) {
      Serial.println("Blynk Switch ON, Door OPENED");
      doorLockState = 1;  // Set door to unlocked state
      digitalWrite(DOOR_LOCK_PIN, HIGH);  // Unlock the door
    } else {
      Serial.println("Blynk Switch OFF, Door CLOSED");
      doorLockState = 0;  // Set door to locked state
      digitalWrite(DOOR_LOCK_PIN, LOW);   // Lock the door
    }
  } else {
    Serial.println("Face Unlock is active, Blynk cannot control the door");
  }
}
