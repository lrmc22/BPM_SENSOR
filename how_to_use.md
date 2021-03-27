# Prerequisites

In this project we are using this configuration : 

- Visual Studio Code 1.52.1 or above
- Install C/C++ plugin for Visual Studio Code : https://code.visualstudio.com/docs/languages/cpp
- PlateformIO core 5.0.4  : https://platformio.org/install/ide?install=vscode
- PlateformIO home 3.3.1
- Hardware must be wired according to what sensor you want to implement
- Use a NodeMCU (ESP12) microcontroller 



# Clone our project

- Go to the explorer tab in VSCode and close all folders

- Expand all and click "Clone Repository" a search bar will appear

- Copy the the repository link and paste it in the search bar and click "Clone from GitHub"

  

# Install Nanopb (Google Protocol Buffer based library) to serialise data 

- In the PlateformIO home, select "Libraries tab"

- Type "Nanopb" in the search bar and click on the Petteri Aimonen library

  You can check out the documentation and project here https://github.com/nanopb/nanopb

- Click "Add to project" and select the current project

  

# Install PubSubClient library to use MQTT protocol to send data to a MQTT server

- In the PlateformIO home, select "Libraries tab"

- Type "PubSubClient" in the search bar and click on Nick O'Leary library

  You can check out the documentation and project here https://github.com/knolleary/pubsubclient

- Click "Add to project" and select the current project



# Add our NanoPb interface definition
- In the platform.ini file : add https://github.com/Senso-Care/Interface value to lib_deps key.

  ```ini
  [env:nodemcuv2]
  platform = espressif8266
  board = nodemcuv2
  framework = arduino
  monitor_speed = 115200
  lib_deps = 
  	nanopb/Nanopb@^0.4.4
  	knolleary/PubSubClient@^2.8
  	https://github.com/Senso-Care/Interface
  ```

  

# Get started with your own sensor(s)

This is how your main.cpp should look like at the beginning : 

```c++
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Here include all your sensor libraries

//generated pb interface
#include <pb_encode.h>
#include "interface.pb.h"

//PB variables
bool status;

// MQTT Topic : you can set multiple topics here
const char* myTopic = "/senso-care/sensors/myTopic";

//Our two communicating objects in MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//Declare all your variables here 



//PB sent buffer
uint8_t buffer[sensocare_messages_Measure_size];

void sendSerialised(sensocare_messages_Measure measure, const char *topic)
{
  memset(buffer, '\0', sensocare_messages_Measure_size);
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sensocare_messages_Measure_size);
  status = pb_encode(&stream, sensocare_messages_Measure_fields, &measure);
  if (!status)
  {
    Serial.println("Encoding failed"); // Fail
  }

  //Prinf buffer in serial monitor
  /*for(int i = 0; i < sensocare_messages_Measure_size; i++ )
  {
    Serial.print(buffer[i]);
  }
  Serial.println(" ");*/
  // Envoyer le buffer
  client.publish(topic, (char *)buffer, stream.bytes_written);
}

void createMessageAndSend(float value, const char *topic)
{
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  measure.value.fValue = value;
  measure.which_value = sensocare_messages_Measure_fValue_tag;
  sendSerialised(measure, topic);
}
void createMessageAndSend(int value, const char *topic)
{
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  measure.value.iValue = value;
  measure.which_value = sensocare_messages_Measure_iValue_tag;
  sendSerialised(measure, topic);
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
       //Setup all your sensors here

      // We setup the wifi connection
      wifiSetup();

      //link between client and MQTT server
      client.setServer(mqttServer,1883);
}
void loop() 
{
    //Set as many values as you want to send to a topic
    int value = 0;

    //If the client is disconnected we attempt to reconnect  
    if (!client.connected())
    {
        reconnect();
    }
    //Send serialised data to Mqtt server 
    createMessageAndSend(value, myTopic); 
    //delay(1000);
}

```



