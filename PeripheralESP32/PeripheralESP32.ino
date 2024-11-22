#define BLYNK_TEMPLATE_ID "TMPL2_Rjbd6Ir"
#define BLYNK_TEMPLATE_NAME "Home Automation FYP"
#define BLYNK_AUTH_TOKEN "1ZMDHWdICjWk8GvF5ZvIkTFnZeaLCkrB"
// Peripheral ESP32 Code

#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <SPI.h>
//#include <MFRC522.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
#include <BlynkTimer.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000); // Adjust time zone offset if necessary
int currentHour = 0; // Variable to store the current hour


// Replace with your network credentials
char ssid[] = "Prof";
char pass[] = "nzube123";
char auth[] = "1ZMDHWdICjWk8GvF5ZvIkTFnZeaLCkrB";

// Define sensor and actuator pins
#define DHTPIN 13 
#define DHTTYPE DHT22
#define MOTION_PIN 4
#define GAS_SENSOR_PIN 36
#define CURRENT_SENSOR_PIN 39
#define LDR_PIN 35
#define DOOR_LOCK_PIN 12
#define FAN_RELAY_PIN 14
#define BULB_RELAY_PIN 27
#define ALARM_RELAY_PIN 26

// RFID reader pins
#define SS_PIN  5  
#define RST_PIN 21

// Create sensor and actuator objects
DHT dht(DHTPIN, DHTTYPE);
//MFRC522 rfid(SS_PIN, RST_PIN); 

BlynkTimer timer;

// Structure to send sensor data
typedef struct struct_message {
  float temp;
  float light;
  bool motion;
  int gas_level;
  int hour;
  float current;
} struct_message;

// Create a struct_message 
struct_message sensorData;

// Peer MAC address of Fuzzy Logic ESP32 (replace with actual address)
uint8_t broadcastAddress[] = {0xd4, 0x8a, 0xfc, 0x9f, 0x2d, 0xcc};

// Variable to store received actuator commands
float actuatorCommands[4] = {0}; // door, fan, bulb, alarm

// Callback function when data is received
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&actuatorCommands, incomingData, sizeof(actuatorCommands));
  Serial.println("Received actuator commands:");
  Serial.printf("Light: %d, Fan: %d, Alarm: %d, Energy: %d\n", 
                actuatorCommands[0], actuatorCommands[1], 
                actuatorCommands[2], actuatorCommands[3]);

  // Control actuators based on received commands
  float lightValue = actuatorCommands[0];
  float fanValue = actuatorCommands[1];
  float AlarmValue = actuatorCommands[2];
  float EnergyValue = actuatorCommands[3];
  Serial.print("LIGHT: ");
  Serial.print(lightValue);
  Serial.print("||");
  Serial.print("FAN: ");
  Serial.print(fanValue);
  Serial.print("||");
  Serial.print("ALARM: ");
  Serial.print(AlarmValue);
  Serial.print("||");
  Serial.print("ENERGY: ");
  Serial.print(EnergyValue);
  Serial.print("||");

/*
  digitalWrite(DOOR_LOCK_PIN, actuatorCommands[0]);
  digitalWrite(FAN_RELAY_PIN, actuatorCommands[1]);
  digitalWrite(BULB_RELAY_PIN, actuatorCommands[2]);
  digitalWrite(ALARM_RELAY_PIN, actuatorCommands[3]);
*/


}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected at IP ");
  Serial.print(WiFi.localIP());

  Blynk.begin(auth, ssid, pass);
  //BlynkTimer.begin();
  timeClient.begin();
  // Initialize sensors and actuators
  dht.begin();
  pinMode(MOTION_PIN, INPUT_PULLUP);
  //pinMode(GAS_SENSOR_PIN, INPUT);
  //pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(DOOR_LOCK_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(BULB_RELAY_PIN, OUTPUT);
  pinMode(ALARM_RELAY_PIN, OUTPUT);
  SPI.begin();        
  //rfid.PCD_Init(); 

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void syncTimeWithBlynk() {
  Blynk.syncVirtual(V0);  // Sync with a virtual pin (use any available pin)
}

void loop() {
  Blynk.run();
  timeClient.update();

  // Read sensor data
  sensorData.temp = dht.readTemperature();
  //sensorData.humidity = dht.readHumidity();
  sensorData.light = analogRead(LDR_PIN);
  sensorData.motion = digitalRead(MOTION_PIN);
  sensorData.gas_level = analogRead(GAS_SENSOR_PIN);
  sensorData.current = analogRead(CURRENT_SENSOR_PIN);

  // Get hour of the day from Blynk
  sensorData.hour = timeClient.getHours();

  Serial.print("Temperature ");
  Serial.print(sensorData.temp);
  Serial.print(" Humidity ");
  Serial.print(dht.readHumidity());
  Serial.print(" LIGHT: ");
  Serial.print(sensorData.light);
  Serial.print(" MOTION: ");
  Serial.print(sensorData.motion);
  Serial.print(" GAS LEVEL: ");
  Serial.print(sensorData.gas_level);
  Serial.print(" CURRENT ");
  Serial.print(sensorData.current);
  Serial.print(" HOUR: ");
  Serial.print(sensorData.hour);

  /*
  // Check for RFID tag
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      sensorData.rfid_present = true; 
      rfid.PICC_HaltA();
    }
  } else {
    sensorData.rfid_present = false;
  }
  */

  // Send sensor data to Fuzzy Logic ESP32
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *) &sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    Serial.println("Sent sensor data");
  } else {
    Serial.println(" Error sending sensor data");
  }

  // Update Blynk interface (example)
  Blynk.virtualWrite(V1, 35);
  Blynk.virtualWrite(V2, sensorData.light); 
  Blynk.virtualWrite(V3, sensorData.hour); 
  Blynk.virtualWrite(V4, sensorData.current); 
  Blynk.virtualWrite(V5, dht.readHumidity());
  Blynk.virtualWrite(V6, sensorData.gas_level);

  delay(2000); 
}
