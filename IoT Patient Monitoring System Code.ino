/****************************************
 * Include Libraries
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h> 
#include <DHT.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h" 
  
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
 
float beatsPerMinute;
int beatAvg;

const int PushButton=32;
int buzzer = 2; 
int x=0;
 
#define WIFISSID "LAI-2_2.4GHz" // Put your WifiSSID here
#define PASSWORD "LAI0148878881" // Put your wifi password here
#define TOKEN "BBFF-EXqrex4p8TucfAifliba5gUA4qkUBK" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "mymqttclient" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
                                           //it should be a random and unique ascii string and different from all other devices

/****************************************
 * Define Constants
 ****************************************/
#define VARIABLE_LABEL "ECG_sensor_data" // ubidots variable label
#define VARIABLE_LABEL_1 "surrounding_temperature"
#define VARIABLE_LABEL_2 "surrounding_humidity"
#define VARIABLE_LABEL_3 "body_temperature"
#define VARIABLE_LABEL_4 "bpm"
#define VARIABLE_LABEL_5 "bpm_avg"
#define DEVICE_LABEL "ESP32_Patient_Monitoring" // ubidots device label
#define SENSOR A0 // Set the A0 as SENSOR
#define DHTTYPE DHT22
uint8_t DHTPin=4;
DHT dht(DHTPin, DHTTYPE);
char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[100];
char topic[150];


// Space to store values to send
char str_sensor[10];
char str_sensor1[10];
char str_sensor2[10];
char str_sensor3[10];
char str_sensor4[10];
char str_sensor5[10];
/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

/****************************************
 * Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFISSID, PASSWORD);
  // Assign the pin as INPUT 
  pinMode(SENSOR, INPUT);
  pinMode(DHTPin, INPUT); 
  pinMode(PushButton, INPUT);
  pinMode(buzzer, OUTPUT); 
  dht.begin();
  Serial.println();
  Serial.print("Waiting for WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback); 
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
  Serial.println("MAX30105 was not found. Please check wiring/power. ");
  while (1);
  }
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  } 
 

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  long irValue = particleSensor.getIR();
  
  float sensor = analogRead(SENSOR); 
  float sensor1= dht.readTemperature(); 
  float sensor2= dht.readHumidity();  
  float temperature = particleSensor.readTemperature();
  if (checkForBeat(irValue) == true)
  {
  //We sensed a beat!
  long delta = millis() - lastBeat;
  lastBeat = millis();
 
  beatsPerMinute = 60 / (delta / 1000.0);
  }
  int Push_button_state = digitalRead(PushButton);
  if (x==0)
  {
  if ( Push_button_state == HIGH )
  { 
  digitalWrite (buzzer, HIGH); //turn buzzer on
  x=x+1;
  }
  }
  else
  { 
  if (Push_button_state == HIGH)
  { 
  digitalWrite (buzzer, LOW); //turn buzzer on
  x=x-1;
  }
  }
  dtostrf(sensor, 4, 2, str_sensor);
  dtostrf(sensor1, 4, 2, str_sensor1);
  dtostrf(sensor2, 4, 2, str_sensor2);
  dtostrf(temperature, 4,2, str_sensor3);
  dtostrf(beatsPerMinute, 4,2, str_sensor4);
  dtostrf(beatAvg, 4,2, str_sensor5);
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload content
//  sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label   
//  sprintf(payload, "%s {\"value\": %s", payload, str_sensor); // Adds the value
//  sprintf(payload, "%s } }", payload); // Closes the dictionary brackets
//  
  sprintf(payload, "{\"%s\":%s,", VARIABLE_LABEL, str_sensor); // Adds the variable label
  sprintf(payload, "%s \"%s\":%s,", payload, VARIABLE_LABEL_1, str_sensor1); // Adds the variable label
  sprintf(payload, "%s \"%s\":%s}", payload, VARIABLE_LABEL_2, str_sensor2); // Adds the variable label
  Serial.println("Publishing data to Ubidots Cloud");
  Serial.println(payload);
  client.publish(topic, payload); 

  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload content
  sprintf(payload, "{\"%s\":%s,", VARIABLE_LABEL_3, str_sensor3); // Adds the variable label
  sprintf(payload, "%s \"%s\":%s,", payload, VARIABLE_LABEL_4, str_sensor4); // Adds the variable label
  sprintf(payload, "%s \"%s\":%s}", payload, VARIABLE_LABEL_5, str_sensor5); // Adds the variable label
  Serial.println("Publishing data to Ubidots Cloud");
  Serial.println(payload);
  client.publish(topic, payload); 
  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
}
