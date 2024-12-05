#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MQ135.h"
#include "DHT.h"

#define MODEM_BAUDRATE                      (115200)
#define MODEM_DTR_PIN                       (25)
#define MODEM_TX_PIN                        (26)
#define MODEM_RX_PIN                        (27)
#define BOARD_PWRKEY_PIN                    (4)
#define BOARD_POWERON_PIN                   (12)
#define MODEM_RING_PIN                      (33)
#define MODEM_RESET_PIN                     (5)
#define MODEM_RESET_LEVEL                   HIGH
#define SerialAT                            Serial1

#define BOARD_ADC_PIN                       (35)
#define BOARD_BAT_ADC_PIN                   (35)
#define MODEM_GPS_ENABLE_GPIO               (-1)

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#define TINY_GSM_MODEM_A7670

#define SerialMon Serial
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon

#define DUMP_AT_COMMANDS

#define TINY_GSM_USE_GPRS true

float longitude = 0;
float latitude = 0;

const char apn[]      = "<apn>";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char *broker = "<broker-address>";
const char *topic = "topic";

#include <TinyGsmClient.h>  

#ifdef DUMP_AT_COMMANDS  
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

TinyGsmClient  client(modem);
PubSubClient   mqtt(client);

uint32_t lastPublishTime = 0;

#define MQ135_PIN 39
#define DHTPIN 13       
#define DHTTYPE DHT22  
#define SOUND_SENSOR_AO 34  
#define DUST_SENSOR 32
#define LED_POWER 15

int gp2y10_value = 0;
int ky037_value = 0;
int gpsSent = 0;

// Add relay pin definition
#define RELAY_PIN 14 // Pin connected to relay input

DHT dht(DHTPIN, DHTTYPE); 

void setup() {
    Serial.begin(115200); // Initialize Serial communication
    dht.begin();  

    #ifdef BOARD_POWERON_PIN
        pinMode(BOARD_POWERON_PIN, OUTPUT);
        digitalWrite(BOARD_POWERON_PIN, HIGH);
    #endif

    pinMode(LED_POWER, OUTPUT);
    pinMode(SOUND_SENSOR_AO, INPUT);
    pinMode(MQ135_PIN, INPUT); // Set the MQ-135 pin as an input

    // Set relay pin as output
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Ensure relay is off initially

    // Turn on DC boost to power on the modem
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);

    // Set modem reset pin, reset modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    // Turn on modem
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW); delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH); delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Set modem baud
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    Serial.println("Start modem...");
    delay(3000);

    if (!modem.init()) {
        SerialMon.println("Failed to restart modem, delaying 10s and retrying");
        return;
    }

    String name = modem.getModemName();
    SerialMon.print("Modem Name: ");
    SerialMon.println(name);

    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);

    // GPRS connection
    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    // GPRS connection parameters
    SerialMon.print("Connecting to ");
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected()) {
        SerialMon.println("GPRS connected");
    }

    Serial.println("Enabling GPS/GNSS/GLONASS");
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO)) {
        Serial.print(".");
    }
    Serial.println();
    Serial.println("GPS Enabled");

    modem.setGPSBaud(115200);

    getGpsInfo();

    // MQTT Broker setup
    mqtt.setServer(broker, 1883);
}

void loop() {
    // Ensure we are connected to the network
    if (!modem.isNetworkConnected()) {
        SerialMon.println("Network disconnected");
        if (!modem.waitForNetwork(180000L, true)) {
            SerialMon.println(" fail");
            delay(10000);
            return;
        }
        if (modem.isNetworkConnected()) {
            SerialMon.println("Network re-connected");
        }

        // Ensure GPRS is still connected
        if (!modem.isGprsConnected()) {
            SerialMon.println("GPRS disconnected!");
            SerialMon.print("Connecting to ");
            SerialMon.print(apn);
            if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
                SerialMon.println(" fail");
                delay(10000);
                return;
            }
            if (modem.isGprsConnected()) {
                SerialMon.println("GPRS reconnected");
            }
        }
    }

    // Check MQTT connection
    if (!mqtt.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        if (mqttConnect()) {
            lastPublishTime = millis(); // Reset publish time on reconnect
        }
        delay(100);
        return;
    }

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    float air_quality = getAirQualityLevel();
    float noise_level = getNoiseLevel();
    float dust_level = getDustLevel();

    // Create a JSON object
    StaticJsonDocument<200> jsonDoc; // Adjust the size as needed
    jsonDoc["tem"] = temperature;
    jsonDoc["hum"] = humidity;
    jsonDoc["gas"] = air_quality;
    jsonDoc["noi"] = noise_level;
    jsonDoc["dus"] = dust_level;

    // Publish data every 5 seconds
    if (millis() - lastPublishTime >= 900000) { // 15 mins
        if (gpsSent == 0) {
            jsonDoc["lon"] = longitude;
            jsonDoc["lat"] = latitude;
            gpsSent = 1;
        }

        // Serialize JSON to string
        String jsonPayload;
        serializeJson(jsonDoc, jsonPayload);

        mqtt.publish(topic, jsonPayload.c_str());
        SerialMon.print("Published data: ");
        SerialMon.println(jsonPayload);
        lastPublishTime = millis(); // Update last publish time
    }
    
    mqtt.loop(); // Maintain MQTT connection

    // Control the relay (for the cooling fan)
    if (temperature > 30) { // Adjust the temperature threshold as needed
        digitalWrite(RELAY_PIN, HIGH); // Turn the fan on
    } else {
        digitalWrite(RELAY_PIN, LOW); // Turn the fan off
    }

    delay(2000); // Delay for 2 seconds before the next reading
}

float getNoiseLevel(){
  int soundLevel = analogRead(SOUND_SENSOR_AO);
  float decibels = (soundLevel / 1023.0) * 5.0 * 20.0;
  return decibels;
}

float getDustLevel(){
    digitalWrite(LED_POWER,LOW); // power on the LED
    delayMicroseconds(280);

    gp2y10_value = analogRead(DUST_SENSOR);

    delayMicroseconds(40);
    digitalWrite(LED_POWER,HIGH); // turn the LED off
    delayMicroseconds(9680);

    float gp2y10_voltage = gp2y10_value * (5.0 / 4095.0);  // ESP32 has 12-bit ADC
    float dustDensity = 170 * gp2y10_voltage - 0.1;     // Conversion factor

    return dustDensity;
}

float getHumidity() {
    return dht.readHumidity();
}

float getTemperature() {
    return dht.readTemperature();
}

void getGpsInfo(){
    float lat2      = 0;
    float lon2      = 0;
    float speed2    = 0;
    float alt2      = 0;
    int   vsat2     = 0;
    int   usat2     = 0;
    float accuracy2 = 0;
    int   year2     = 0;
    int   month2    = 0;
    int   day2      = 0;
    int   hour2     = 0;
    int   min2      = 0;
    int   sec2      = 0;
    uint8_t    fixMode   = 0;
    
    for (;;) {
        Serial.println("Requesting current GPS/GNSS/GLONASS location");
        if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                        &year2, &month2, &day2, &hour2, &min2, &sec2)) {

            latitude = lat2;
            longitude = lon2;

            break;
        } else {
            Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
            delay(15000L);
        }
    }

    Serial.println("Disabling GPS");
    modem.disableGPS();
}

bool mqttConnect() {
    SerialMon.print("Connecting to MQTT... ");
    // Reconnect to MQTT broker
    if (mqtt.connect("GSMClient", "user", "pass")) {
        SerialMon.println("connected");
        return true;
    } else {
        SerialMon.print("failed, rc=");
        SerialMon.print(mqtt.state());
        return false;
    }
}

float getAirQualityLevel(){
    // Get the sensor reading (from the analog input)
    MQ135 analogValue = MQ135(MQ135_PIN);

    float ppm = analogValue.getPPM();
    return ppm;

}