#include <WiFi.h>
#include "PubSubClient.h"

// New motor pin assignments
#define motorRightPin1 26 // Right motor pin 1
#define motorRightPin2 27 // Right motor pin 2
#define enableRightPin 14 // Right motor enable pin

#define motorLeftPin1 33 // Left motor pin 1
#define motorLeftPin2 25 // Left motor pin 2
#define enableLeftPin 32 // Left motor enable pin

int left_min_pwm_forward = 110;
int left_min_pwm_backward = 115;
int right_min_pwm_forward = 110;
int right_min_pwm_backward = 115;

WiFiClient espClient22;
PubSubClient client(espClient22);

// Replace with your network credentials (STATION)
const char* ssid = "Redmi";
const char* password = "123456789";
const char* mqtt_server = "192.168.1.117";

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(9600);
  initWiFi();
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Initialize motor pins as OUTPUT
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(enableRightPin, OUTPUT);
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(enableLeftPin, OUTPUT);
}

void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  
  String messageInfo;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageInfo += (char)message[i];
  }
  Serial.println();

  if(topic == "/robot6_left_forward"){
      int pwm_value = messageInfo.toInt();
      if(pwm_value != 0)
        pwm_value = map(left_min_pwm_forward, 255, 0, 255, pwm_value);
      analogWrite(enableLeftPin, pwm_value);      
      digitalWrite(motorLeftPin1, LOW);
      digitalWrite(motorLeftPin2, HIGH);
      Serial.print("robot6_left_forward : ");
      Serial.println(pwm_value);
  }
  if(topic == "/robot6_right_forward"){
      int pwm_value = messageInfo.toInt();
      if(pwm_value != 0)
        pwm_value = map(right_min_pwm_forward, 255, 0, 255, pwm_value);
      analogWrite(enableRightPin, pwm_value);
      digitalWrite(motorRightPin1, LOW);
      digitalWrite(motorRightPin2, HIGH);
      Serial.print("robot6_right_forward : ");
      Serial.println(pwm_value);
  }
  if(topic == "/robot6_left_backward"){
      int pwm_value = messageInfo.toInt();
      if(pwm_value != 0)
        pwm_value = map(left_min_pwm_backward, 255, 0, 255, pwm_value);
      analogWrite(enableLeftPin, pwm_value);
      digitalWrite(motorLeftPin1, HIGH);
      digitalWrite(motorLeftPin2, LOW);
      Serial.print("robot6_left_backward : ");
      Serial.println(pwm_value);
  }
  if(topic == "/robot6_right_backward"){
      int pwm_value = messageInfo.toInt();
      if(pwm_value != 0)
        pwm_value = map(right_min_pwm_backward, 255, 0, 255, pwm_value);
      analogWrite(enableRightPin, pwm_value);
      digitalWrite(motorRightPin1, HIGH);
      digitalWrite(motorRightPin2, LOW);
      Serial.print("robot6_right_backward : ");
      Serial.println(pwm_value);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client21")) {
      Serial.println("connected");  
      client.subscribe("/robot6_left_forward");
      client.subscribe("/robot6_right_forward");
      client.subscribe("/robot6_left_backward");
      client.subscribe("/robot6_right_backward");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if ((WiFi.status() != WL_CONNECTED)){
    Serial.println("Reconnecting to WiFi...");
    initWiFi();
  }
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop()){
    client.connect("ESP32Client21");
  }
  delay(100);
}
