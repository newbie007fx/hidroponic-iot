#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#define tdsSensorPin 35
#define temperatureSensorPin 25
#define phSensorPin 36
#define flowSensorPin 34
#define nutritionWaterDistanceTrigPin 5
#define nutritionWaterDistanceEchoPin 18
#define rawWaterDistanceTrigPin 13
#define rawWaterDistanceEchoPin 14
// in 1
#define nutritionPumpPin 23
// in 2
#define mixingPumpPin 22
// in 3
#define nutritionWaterPumpPin 21
// in 4
#define rawWaterPumpPin 19

#define VREF 3.3
#define SCOUNT 30
#define SOUND_SPEED 0.034

#define STATE_INITIATE 0
#define STATE_ON 1
#define STATE_OFF 2
#define STATE_AUTOMATION 3
#define STATE_NOT_CONNECT 5

int tdsAnalogBuffer[SCOUNT];
int tdsAnalogBufferTemp[SCOUNT];
int tdsAnalogBufferIndex = 0;
int tdsCopyIndex = 0;
float tdsAverageVoltage = 0;

int phAnalogBuffer[SCOUNT];
int phAnalogBufferTemp[SCOUNT];
int phAnalogBufferIndex = 0;
int phCopyIndex = 0;
float phAverageVoltage = 0;
float PH4 = 2.29;
float PH7 = 1.5;

float flowVolumePerPulse = 0.000196;
int flowFrequency = 0;
float vol;
long currentTime;
long cloopTime;
float lMinute;
int lastState = STATE_INITIATE;
bool toggleAddNutrition = false;
bool toggleAddWater = false;
bool toggleCoolingDown = false;
bool isAutomationComplete = true;

float tdsValue = 0;
float temperatureValue = 25;
float phValue = 0;
float nutritionWaterDistanceValue = 0;
float rawWaterDistanceValue = 0;

int getDeviceStateRetry = 0;

int automationID;
float targetVolume;
float targetDistance;

const char *ssid = "wifi-test";
const char *password = "Password1234";
const char *mqtt_broker = "odf3bc12.ala.asia-southeast1.emqxsl.com";
const char *topic_device_state_client = "device_state_client";
const char *topic_device_state = "device_state";
const char *topic_read_sensor = "read_sensor";
const char *mqtt_username = "testesp32";
const char *mqtt_password = "testesp32";
const int mqtt_port = 8883;

OneWire oneWire(temperatureSensorPin);
DallasTemperature sensors(&oneWire);
WiFiClientSecure esp_client;
PubSubClient mqtt_client(esp_client);

SemaphoreHandle_t xMutex = NULL;

int deviceState = STATE_INITIATE;

const char *ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  esp_client.setCACert(ca_cert);

  xMutex = xSemaphoreCreateMutex();

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setCallback(mqttCallback);

  connectToMQTT();

  pinMode(tdsSensorPin, INPUT);
  pinMode(phSensorPin, INPUT);
  pinMode(flowSensorPin, INPUT);
  pinMode(rawWaterDistanceTrigPin, OUTPUT);        
  pinMode(rawWaterDistanceEchoPin, INPUT);         
  pinMode(nutritionWaterDistanceTrigPin, OUTPUT);  
  pinMode(nutritionWaterDistanceEchoPin, INPUT);  
  pinMode(nutritionPumpPin, OUTPUT);
  pinMode(mixingPumpPin, OUTPUT);
  pinMode(nutritionWaterPumpPin, OUTPUT);
  pinMode(rawWaterPumpPin, OUTPUT);

  sensors.begin();

  digitalWrite(nutritionPumpPin, HIGH);
  digitalWrite(mixingPumpPin, HIGH);
  digitalWrite(nutritionWaterPumpPin, HIGH);
  digitalWrite(rawWaterPumpPin, HIGH);

  xTaskCreatePinnedToCore(
    loop2,
    "loop2",
    3000,
    NULL,
    0,
    NULL,
    0);
}

int getDeviceState() {
  int state = STATE_INITIATE;
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    state = deviceState;
    xSemaphoreGive(xMutex);
  }

  return state;
}

void setDeviceState(int state) {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    deviceState = state;
    xSemaphoreGive(xMutex);
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void connectToMQTT() {
  while (!mqtt_client.connected()) {
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT Broker as %s...\n", client_id.c_str());
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe(topic_device_state);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" Retrying in 5 seconds.");
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  const char *data = (char *)payload;
  StaticJsonDocument<64> doc;

  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, topic_device_state) == 0) {
    int state = doc["state"];
    if (state == STATE_AUTOMATION) {
      JsonObject data = doc["data"];
      automationID = data["automation_id"];
      targetVolume = data["target_nutrition_volume"];
      targetDistance = data["target_water_distance"];
    }
    setDeviceState(state);
  }

  Serial.println("\n-----------------------");
}

void loop() {
  if (!mqtt_client.connected()) {
    connectToMQTT();
  }
  mqtt_client.loop();

  if (getDeviceStateRetry >= 5) {
    setDeviceState(STATE_NOT_CONNECT);
    getDeviceStateRetry = 0;
  }

  if (getDeviceState() == STATE_INITIATE) {
    Serial.println("requesting device state");
    delay(500);
    requestDeviceState();
    getDeviceStateRetry++;
    delay(1000 * getDeviceStateRetry);
  }

  if (getDeviceState() == STATE_ON || getDeviceState() == STATE_AUTOMATION) {
    runTdsSensor();
    runPhSensor();
    runNutritionWaterDistanceSensor();
    runRawWaterDistanceSensor();

    if (getDeviceState() == STATE_AUTOMATION) {
      runAutomation(targetVolume, targetDistance);
      Serial.println("automation running.");
      Serial.print("target volume nutrition: ");
      Serial.println(targetVolume);
      Serial.print("target distance: ");
      Serial.println(targetDistance);

      if (isAutomationComplete) {
        digitalWrite(nutritionWaterPumpPin, LOW);

        JsonDocument doc;

        doc["action"] = "complete";
        doc["value"] = automationID;

        char data[64];
        serializeJson(doc, data);
        mqtt_client.publish(topic_device_state_client, data);
        setDeviceState(STATE_ON);
      }

      lastState = getDeviceState();
    }
  }

  if (getDeviceState() == STATE_ON) {
    digitalWrite(nutritionWaterPumpPin, LOW);
  } else {
    digitalWrite(nutritionWaterPumpPin, HIGH);
  }

  delay(10);
}

void loop2(void *pvParameters) {
  while (1) {
    if (getDeviceState() == STATE_ON || getDeviceState() == STATE_AUTOMATION) {
      static unsigned long printTimepoint = millis();
      if (millis() - printTimepoint > 1000) {
        Serial.println("running");
        printTimepoint = millis();
        StaticJsonDocument<168> sensorData1;

        JsonObject doc_0 = sensorData1.createNestedObject();
        doc_0["sensor"] = "nutrition_water_level";
        doc_0["value"] = tdsValue;
        JsonObject doc_1 = sensorData1.createNestedObject();
        doc_1["sensor"] = "water_ph";
        doc_1["value"] = phValue;
        JsonObject doc_2 = sensorData1.createNestedObject();
        doc_2["sensor"] = "water_temperature";
        doc_2["value"] = temperatureValue;

        sendSensorData(sensorData1);

        StaticJsonDocument<168> sensorData2;

        JsonObject doc1_0 = sensorData2.createNestedObject();
        doc1_0["sensor"] = "nutrition_water_distance";
        doc1_0["value"] = nutritionWaterDistanceValue;
        JsonObject doc1_1 = sensorData2.createNestedObject();
        doc1_1["sensor"] = "raw_water_distance";
        doc1_1["value"] = rawWaterDistanceValue;

        sendSensorData(sensorData2);

        Serial.print("nutrition water distance: ");
        Serial.println(nutritionWaterDistanceValue);
        Serial.print("raw water distance: ");
        Serial.println(rawWaterDistanceValue);
      }
    } else if (getDeviceState() == STATE_NOT_CONNECT) {
      Serial.println("cannot connect to server");
      delay(5000);
    }
    delay(200);
  }
}

void sendSensorData(StaticJsonDocument<168> objectData) {
  char data[168];
  serializeJson(objectData, data);
  mqtt_client.publish(topic_read_sensor, data);
}

void requestDeviceState() {
  JsonDocument doc;

  doc["action"] = "get";

  char data[64];
  serializeJson(doc, data);
  mqtt_client.publish(topic_device_state_client, data);
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void runTdsSensor() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  if (temperature <= 0) {
    temperature = temperatureValue;
    Serial.println("temperature error");
  }
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    tdsAnalogBuffer[tdsAnalogBufferIndex] = analogRead(tdsSensorPin);
    tdsAnalogBufferIndex++;
    if (tdsAnalogBufferIndex == SCOUNT)
      tdsAnalogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (tdsCopyIndex = 0; tdsCopyIndex < SCOUNT; tdsCopyIndex++)
      tdsAnalogBufferTemp[tdsCopyIndex] = tdsAnalogBuffer[tdsCopyIndex];
    tdsAverageVoltage = getMedianNum(tdsAnalogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
    float compensationCoefficient = 1.0 + 0.03 * (temperature - 25.0);
    float compensationVoltage = tdsAverageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.6;
    Serial.println(tdsValue);
    temperatureValue = temperature;
  }
}

void runPhSensor() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    phAnalogBuffer[phAnalogBufferIndex] = analogRead(phSensorPin);
    phAnalogBufferIndex++;
    if (phAnalogBufferIndex == SCOUNT)
      phAnalogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (phCopyIndex = 0; phCopyIndex < SCOUNT; phCopyIndex++)
      phAnalogBufferTemp[phCopyIndex] = phAnalogBuffer[phCopyIndex];
    phAverageVoltage = getMedianNum(phAnalogBufferTemp, SCOUNT) * (float)VREF / 4095.0;

    float phStep = (PH4 - PH7) / (7 - 4.01);
    phValue = 7.00 + ((PH7 - phAverageVoltage) / phStep);
  }
}

void runNutritionWaterDistanceSensor() {
  digitalWrite(nutritionWaterDistanceTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(nutritionWaterDistanceTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(nutritionWaterDistanceTrigPin, LOW);

  long duration2 = pulseIn(nutritionWaterDistanceEchoPin, HIGH);

  nutritionWaterDistanceValue = duration2 * SOUND_SPEED / 2;
}

void runRawWaterDistanceSensor() {
  digitalWrite(rawWaterDistanceTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(rawWaterDistanceTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rawWaterDistanceTrigPin, LOW);

  long duration1 = pulseIn(rawWaterDistanceEchoPin, HIGH);

  rawWaterDistanceValue = duration1 * SOUND_SPEED / 2;
}

void runAutomation(float targetVolume, float targetDistance) {
  currentTime = millis();
  if (lastState != STATE_AUTOMATION && getDeviceState() == STATE_AUTOMATION) {
    vol = 0;
    flowFrequency = 0;
    currentTime = millis();
    cloopTime = currentTime;
    lMinute = 0;
    toggleAddNutrition = true;
    toggleAddWater = true;
    isAutomationComplete = false;
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), flow, RISING);

    digitalWrite(nutritionWaterPumpPin, HIGH);
    digitalWrite(nutritionPumpPin, LOW);
  }
  Serial.print("volume: ");
  Serial.println(vol);
  if (vol < targetVolume) {
    if (currentTime >= (cloopTime + 100)) {
      cloopTime = currentTime;
      if (flowFrequency != 0) {
        lMinute = (flowFrequency * flowVolumePerPulse);
        vol = vol + lMinute * 1000;
        flowFrequency = 0;
      }
    }
  } else {
    if (toggleAddNutrition) {
      digitalWrite(nutritionPumpPin, HIGH);
      detachInterrupt(digitalPinToInterrupt(flowSensorPin));
      toggleAddNutrition = false;
      digitalWrite(mixingPumpPin, LOW);
      digitalWrite(rawWaterPumpPin, LOW);
    }

    if (nutritionWaterDistanceValue <= targetDistance && toggleAddWater) {
      digitalWrite(rawWaterPumpPin, HIGH);
      toggleAddWater = false;
    }

    if (toggleAddWater == false && toggleAddNutrition == false && toggleCoolingDown == false) {
      cloopTime = currentTime;
      toggleCoolingDown = true;
    }

    if (toggleCoolingDown && currentTime >= (cloopTime + 35000)) {
      isAutomationComplete = true;
      digitalWrite(mixingPumpPin, HIGH);
    }
  }
}

void flow() {
  flowFrequency++;
}