/*
  See:
  1) https://github.com/gonium/esp8266-dht22-sensor for the inspiration for reading the DHT22 sensor values
  2) https://github.com/256dpi/arduino-mqtt/blob/master/examples/AdafruitHuzzahESP8266/AdafruitHuzzahESP8266.ino
*/

#include <ESP8266WiFi.h>
#include <MQTTClient.h>
#include <RunningAverage.h>

#include <DHT.h>

#define SENSOR_DHT22 1
#define DHTTYPE DHT22

#define DHTPIN  2
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

const char* ssid = "********";
const char* password = "********";

const char* mqttServer = "1.2.3.4";
const char* tempTopic = "dht22/temperature";
const char* humiTopic = "dht22/humidity";

// The sensor ID. Used by the server (e.g. OpenHab implementation) to identify where the data is coming from.
const char* sensorid = "dht22";

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient net;
MQTTClient client;

RunningAverage* temp_aggregator;
RunningAverage* hum_aggregator;
bool sensor_ok = false;

enum SensorState {
  MEASURED_OK,
  MEASURED_FAILED,
  TOO_EARLY
};

const long sensorReadIntervalMillis = 10000;
unsigned long previousSensorReadMillis;

unsigned long lastConnectionTime = 0;              // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 15L * 1000L; // delay between updates, in milliseconds

unsigned long resetPeriod = 86400000; // 1 day

/* Connect to the WiFi network and print a lot of debug information */
void connectWiFi() {
  Serial.print("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(" done");
}

void connectMQTT() {
  Serial.println("");
  String mqttClientId = "humidori-" + String(millis());
  Serial.print("Connecting " + mqttClientId + " to MQTT broker");

  client.begin(mqttServer, net);

  while (!client.connect((char*) mqttClientId.c_str())) {
    Serial.print(".");
  }
  Serial.println(" done");
}

void connect() {
  connectWiFi();
  connectMQTT();
}

// read the sensor with the given bus index. Sets temperature in temp,
// humidity in humidity. Returns the SensorState.
int read_sensor(float& temp, float& humidity) {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();

  if (currentMillis - previousSensorReadMillis >= sensorReadIntervalMillis) {
    // save the last time you read the sensor
    previousSensorReadMillis = currentMillis;

    // Read temp&hum from DHT22
    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();          // Read humidity (percent)
    temp = dht.readTemperature(false);     // Read temperature as Celsius

    //Serial.print("Free heap:");
    //Serial.println(ESP.getFreeHeap(),DEC);

    if (isnan(temp) || temp == 85.0 || temp == (-127.0)) {
      Serial.println("Failed to read from sensor");
      // resetting the previous measurement time so that a failed attempt
      // will be repeated with the next query.
      previousSensorReadMillis = currentMillis - 2000;
      if (previousSensorReadMillis < 0) previousSensorReadMillis = 0;
      return MEASURED_FAILED;
    } else {
      return MEASURED_OK;
    }
  } else {
    return TOO_EARLY; // no measurement taken - time not elapsed
  }
}

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("");
  Serial.println("Wifi temperature/humidity sensor v0.1");

  temp_aggregator = new RunningAverage(6);
  hum_aggregator = new RunningAverage(6);
  sensor_ok = false;

  connect();

  Serial.println("Setup done");
}

void updateAggregators(float& temp, float& humidity) {
 switch (read_sensor(temp, humidity)) {
    case MEASURED_OK:
      sensor_ok = true;
      //Serial.print("Updating accumulator w/ new measurements: ");
      //Serial.print(" temperature: ");
      //Serial.print(temp);
      //Serial.print(", humidity: ");
      //Serial.println(humidity);
      temp_aggregator->addValue(temp);
      hum_aggregator->addValue(humidity);
      break;
    case MEASURED_FAILED:
      Serial.println("Measurement failed");
      sensor_ok = false;
      break;
    case TOO_EARLY:
      ;;
      break;
  } 
}

void loop() {

  client.loop();
  delay(10);

  if (!client.connected()) {
    connect();
  }
  
  float temp = 0.0;
  float humidity = 0.0;
  updateAggregators(temp, humidity);

  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    lastConnectionTime = millis();
    Serial.println("Publishing data");
    client.publish(tempTopic, String(temp_aggregator->getAverage()));
    client.publish(humiTopic, String(hum_aggregator->getAverage()));
  }
  // reset after resetPeriod to avoid memory leaks
  //if (millis() > resetPeriod) {
  //  ESP.restart();
  //}
}

void messageReceived(String topic, String payload, char * bytes, unsigned int length) {
  Serial.print("incoming: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.print(payload);
  Serial.println();
}
