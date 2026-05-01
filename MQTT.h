#include <WiFiManager.h>                              
#include <ESP8266WiFi.h>                             
#include <PubSubClient.h>

#ifndef MQTT_h
#define MQTT_h

WiFiClient espClient;
PubSubClient client(espClient);

//WiFi credentials (declared externally in the baseline file)
extern const char* ssid;
extern const char* password;

//MQTT Broker Credential (declared externally in the baseline file)
extern const char* mqtt_server; 
extern const char* mqtt_username;
extern const char* mqtt_password;
extern const int mqtt_port;

//MQTT Topic name 
String payload;
extern const String topic;

extern float x_robot;
extern float y_robot;
extern float z_ang_robot;

String get_wifi_status(int status){
  switch(status){
    case WL_IDLE_STATUS:         return "WL_IDLE_STATUS";
    case WL_SCAN_COMPLETED:      return "WL_SCAN_COMPLETED";
    case WL_NO_SSID_AVAIL:       return "WL_NO_SSID_AVAIL";
    case WL_CONNECT_FAILED:      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:     return "WL_CONNECTION_LOST";
    case WL_CONNECTED:           return "WL_CONNECTED";
    case WL_DISCONNECTED:        return "WL_DISCONNECTED";
    default:                     return "";
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print(". Message: ");

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  int i1 = messageTemp.indexOf(',');
  int i2 = messageTemp.indexOf(',', i1+1);
  String firstValue  = messageTemp.substring(0, i1);
  String secondValue = messageTemp.substring(i1 + 1, i2);
  String thirdValue  = messageTemp.substring(i2 + 1);

  x_robot     = firstValue.toFloat();
  y_robot     = secondValue.toFloat();
  z_ang_robot = thirdValue.toFloat();
}

void reconnect() {
  //Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");   
    String clientId = "ESP8266-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)){
      client.subscribe(topic.c_str(),0);
      Serial.println("connected");
      delay(500);
    }
    delay(500);
    Serial.println("MQTT Client state: " + String(client.state()));
  }
}

extern char pubString1[8];

void wifi_mqtt_init(){
  //WIFI connection status checking
  //WiFi.mode(WIFI_STA);
  int status = WL_IDLE_STATUS;
  
  WiFi.begin(ssid, password);
  Serial.println(get_wifi_status(status));
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED){
    status = WiFi.status();
    Serial.println(get_wifi_status(status));
    delay(200);
  }
  delay(200);
  
  Serial.println("\nConnected to the WiFi network");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void mqtt_clean(){
  for(int i = 0; i<100; i++){
    client.loop();
  }
}

void mqtt_rebound(){
  if (!client.connected()){
    reconnect();
    client.subscribe(topic.c_str(),0);
  }
}

#endif
