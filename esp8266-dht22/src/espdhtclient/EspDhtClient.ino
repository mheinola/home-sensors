/*
 See:
  1) https://www.arduino.cc/en/Tutorial/ScanNetworks
  2) https://www.arduino.cc/en/Tutorial/WiFiWebClientRepeating
  3) https://github.com/gonium/esp8266-dht22-sensor for the inspiration for reading the DHT22 sensor values
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <RunningAverage.h>

#define SENSOR_DHT22 1
#define DHTTYPE DHT22

const char* ssid = "********";
const char* password = "********";

// The sensor ID. Used by the server (e.g. OpenHab implementation) to identify where the data is coming from.
const char* sensorid = "********";

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;
IPAddress server(1,2,3,4); // Server to send data (e.g. OpenHab)

#include <DHT.h>
#define DHTPIN  2
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

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
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds

String restUrl = "";

// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("sending data...");
    // send the HTTP request:
    restUrl = "/sensor/" + String(sensorid) + "/temperature/" + String(temp_aggregator->getAverage()) + "/humidity" + String(hum_aggregator->getAverage());
    client.println("GET " + restUrl + +" HTTP/1.1");
    client.println("Host: " + String(sensorid));
    client.println("User-Agent: ESP01/1.0");
    client.println("Connection: close");
    client.println();

    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void printMacAddress() {
  // the MAC address of your Wifi shield
  byte mac[6];

  // print your MAC address:
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
  }
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
  }
}

/* Connect to the WiFi network and print a lot of debug information */
void connectWiFi() {
  Serial.println("Scanning available networks...");
  listNetworks();
  delay(10000);

  Serial.println("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  printMacAddress();
  printWifiStatus();
}

// read the sensor with the given bus index. Sets temperature in temp,
// humidity in humidity. Returns the SensorState.
int read_sensor(float& temp, float& humidity) {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();

  if(currentMillis - previousSensorReadMillis >= sensorReadIntervalMillis) {
    // save the last time you read the sensor
    previousSensorReadMillis = currentMillis;

    // Read temp&hum from DHT22
    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();          // Read humidity (percent)
    temp = dht.readTemperature(false);     // Read temperature as Celsius

    //Serial.print("Free heap:");
    //Serial.println(ESP.getFreeHeap(),DEC);

    if (isnan(temp) || temp==85.0 || temp==(-127.0)) {
      Serial.println("Failed to read from sensor");
      // resetting the previous measurement time so that a failed attempt
      // will be repeated with the next query.
      previousSensorReadMillis=currentMillis-2000;
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

  connectWiFi();
}

void loop() {

      float temp = 0.0;
      float humidity = 0.0;
      switch (read_sensor(temp, humidity)) {
        case MEASURED_OK:
          sensor_ok = true;
          Serial.print("Updating accumulator w/ new measurements: ");
          Serial.print(" temperature: ");
          Serial.print(temp);
          Serial.print(", humidity: ");
          Serial.println(humidity);
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

    // if there's incoming data from the net connection.
      // send it out the serial port.  This is for debugging
      // purposes only:
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
      }

      // if ten seconds have passed since your last connection,
      // then connect again and send data:
      if (millis() - lastConnectionTime > postingInterval) {
        httpRequest();
      }
}

