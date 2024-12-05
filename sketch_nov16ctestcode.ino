#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// Wi-Fi credentials
const char* WIFI_NETWORK = "Siuuuu";
const char* WIFI_PASSWORD = "22221111";
WiFiClient client;

// ThingSpeak Credentials
#define CHANNEL_ID 2748986
#define CHANNEL_API_KEY "96QJ8O8SMU38U0BP"

// Water Flow Sensor Definitions
#define FLOW_SENSOR_PIN 16
volatile uint16_t pulseCount = 0;
float flowRate = 0.0;
unsigned long lastFlowTime = 0;
const float FLOW_THRESHOLD = 35; // Updated Threshold in L/min

// MQ-6 Gas Sensor Definitions
#define MQ6_ANALOG_PIN 34
const float RL = 10.0;  // Load resistance in kilo-ohms (kΩ)
float R0 = 10.0;        // Sensor resistance in clean air (needs calibration)
const float CLEAN_AIR_FACTOR = 10.0;
const float BUTANE_CURVE[3] = {2.3, 0.21, -0.47}; // Butane curve constants
const float GAS_THRESHOLD = 100.0; // Updated Threshold in ppm for butane

// Flame Sensor Definitions
const int flameSensorPins[5] = {27, 26, 25, 33, 32};

// Buzzer Definitions
#define BUZZER_PIN 17

// Function Prototypes
void IRAM_ATTR pulseCounter();
float readMQ6();
float calculatePPM(float ratio);
void connectToWiFi();

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Setup Wi-Fi
  connectToWiFi();
  ThingSpeak.begin(client);

  // Setup Water Flow Sensor
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  lastFlowTime = millis();

  // Calibrate MQ-6 Sensor
  Serial.println("Calibrating MQ-6 sensor in clean air...");
  float calibrationSum = 0;
  for (int i = 0; i < 50; i++) {
    calibrationSum += readMQ6();
    delay(1000);
  }
  R0 = calibrationSum / 50.0 / CLEAN_AIR_FACTOR;  // Calculate R0 in clean air
  Serial.print("MQ-6 Calibration complete. R0: ");
  Serial.println(R0);

  // Setup Flame Sensors
  for (int i = 0; i < 5; i++) {
    pinMode(flameSensorPins[i], INPUT);
  }

  // Setup Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("All sensors initialized.");
}

void loop() {
  // Water Flow Sensor Processing
  unsigned long currentTime = millis();
  if (currentTime - lastFlowTime >= 1000) {
    lastFlowTime = currentTime;
    flowRate = (pulseCount / 450.0) * 60.0;
    pulseCount = 0;
    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
  }

  // MQ-6 Gas Sensor Processing
  float rs = readMQ6();        // Read current Rs value from the sensor
  if (rs <= 0) {
    rs = 1; // Prevent division by zero or negative values
  }
  float ratio = rs / R0;       // Calculate Rs/R0
  float butanePPM = calculatePPM(ratio);  // Convert to ppm for butane

  // Prevent NaN or negative values due to calculation issues
  if (isnan(butanePPM) || butanePPM < 0) {
    butanePPM = 0;
  }

  Serial.print("MQ-6 Ratio (Rs/R0): ");
  Serial.print(ratio);
  Serial.print("\tButane PPM: ");
  Serial.println(butanePPM);

  // Flame Sensor Processing
  bool flameDetected = false;
  for (int i = 0; i < 5; i++) {
    bool isFlameDetected = digitalRead(flameSensorPins[i]) == LOW;
    Serial.print("Raw Reading Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(digitalRead(flameSensorPins[i]));

    if (isFlameDetected) {
      flameDetected = true;
    }
  }
  Serial.println("-----------------------------");

  // Check Conditions for Alert
 bool flameCondition = !flameDetected;
 bool flowRateCondition = flowRate > 0 && flowRate < FLOW_THRESHOLD;
 bool butanePPMCondition = butanePPM > GAS_THRESHOLD;
 bool alertCondition = !flameDetected && flowRate > 0 && flowRate < FLOW_THRESHOLD && butanePPM > GAS_THRESHOLD;


  // Buzzer Alert
  if (flameCondition) {
    Serial.println("Alert Condition Triggered!");
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer

    if (flowRateCondition) {
      Serial.println("Alert Condition Triggered!");
      digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
    } else {
      digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer

      if (butanePPMCondition) {
        Serial.println("Alert Condition Triggered!");
        digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
      } else {
        digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer
      }
      
      if (alertCondition) {
        Serial.println("Alert Condition Triggered!");
        digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
      } else {
        digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer
      }
    }
  }

  // Send Data to ThingSpeak
  ThingSpeak.setField(1, flowRate);
  ThingSpeak.setField(2, butanePPM);
  ThingSpeak.setField(3, flameDetected ? 0 : 1);  // 1 if no flame detected, otherwise 0
  ThingSpeak.setField(4, alertCondition ? 1 : 0); // Alert condition status

  int writeSuccess = ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
  if (writeSuccess == 200) {
    Serial.println("Data sent to ThingSpeak successfully.");
  } else {
    Serial.print("Error sending data to ThingSpeak. Error code: ");
    Serial.println(writeSuccess);
  }

  // Delay before next loop iteration
  delay(15000);
}

// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_NETWORK);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(" Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ISR for pulse counting
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// Function to read the MQ-6 sensor and calculate Rs (sensor resistance)
float readMQ6() {
  int analogValue = analogRead(MQ6_ANALOG_PIN);
  Serial.print("analogValue:");
  Serial.println(analogValue);

  if (analogValue == 0) {
    analogValue = 1; // Prevent division by zero
  }
  float sensorVoltage = (analogValue / 4095.0) * 3.3;
  Serial.print("sensorVoltage:");
  Serial.println(sensorVoltage);
  float RS = ((3.3 * RL) / sensorVoltage) - RL;
  Serial.print("RS:");
  Serial.println(RS);
  return RS;
}

// Function to convert Rs/R0 ratio to butane PPM
float calculatePPM(float ratio) {
  float ppmLog = (log10(ratio) - BUTANE_CURVE[1]) / BUTANE_CURVE[2] + BUTANE_CURVE[0];
  float ppm = pow(10, ppmLog);
  return ppm;
}
