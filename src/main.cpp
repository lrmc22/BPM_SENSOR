#include <Arduino.h>

/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <math.h>

 //generated pb interface
#include <pb_encode.h>
#include "interface.pb.h"

//Router parameters
const char* ssid        = "......."; // Enter your WiFi name
const char* password    =  "......"; // Enter WiFi password
const char* mqttServer  = "192.168.1.78"; //RPI MQTT Server IP

//PB variables
bool status;

// MQTT Topic
const char* bpmTopic = "/senso-care/sensors/bpm-superesp8266";

//Our two communicating objects in MQTT
WiFiClient espClient;
PubSubClient client(espClient);


MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
//Sent parameters
 void sendSerialised(float value, const char* topic)
{
  uint8_t buffer[sensocare_messages_Measure_size];
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  //Set protobuf structure values
  measure.timestamp = (uint64_t) time(NULL);
  measure.value = value;

//Encoding Measure fields
  status = pb_encode(&stream, sensocare_messages_Measure_fields, &measure);

  if (!status)
  {
    Serial.println("Encoding failed"); // Fail
  }
  /*
  //Prinf buffer in serial monitor
  for(int i = 0; i < sensocare_messages_Measure_size; i++ )
  {
    Serial.print(buffer[i]);
  }
  Serial.print(" ");
  // Envoyer le buffer
  client.publish(topic, (char*)buffer);
  */
}
void wifiSetup()
{
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Init wifi connection to router Bbox in our case
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

//We wait until connection is done
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(2*3600, 0, mqttServer);
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected())
  {

    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
     {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                    //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
}

void loop()
{
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE;                    //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();

  //Send Value to the dedicated topic
  sendSerialised(beatsPerMinute,bpmTopic); 
} 
 