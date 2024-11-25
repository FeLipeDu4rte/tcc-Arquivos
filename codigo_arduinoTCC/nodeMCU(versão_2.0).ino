#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DFRobot_SHT20.h"

// Configuração do MQTT
const char *BROKER_MQTT = "broker.hivemq.com";
int BROKER_PORT = 1883;
WiFiClient espClient;
PubSubClient MQTT(espClient);

#define TOPIC_TEMPERATURE "topic_sensor_temperature"
#define TOPIC_HUMIDITY "topic_sensor_humidity"
#define TOPIC_RPM "topic_sensor_rpm"
#define TOPIC_SPEED "topic_sensor_speed"
static char strTemperature[10] = {0};
static char strHumidity[10] = {0};
static char strRPM[10] = {0};
static char strSpeed[10] = {0};

// Configuração Wi-Fi
const char* ssid = "Redmi";
const char* password = "12345678";

// Configuração do sensor SHT20
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

// Configuração do encoder
volatile int pulsos = 0;
int rpm = 0;
float ms = 0;
float pi = 3.1415;
float convert = 3.6;
float kmh = 0;

const int pinoEncoder = 2; // Pino do encoder
const int pulsos_por_volta = 20; // Ajuste conforme o seu encoder

unsigned long millis_old = 0;

// Função chamada pelo encoder
void contarPulsos() {
  pulsos++;
}

// Inicializa o Wi-Fi
void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Conectando à rede Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.println(WiFi.localIP());
}

// Reconecta ao broker MQTT
void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    if (MQTT.connect("esp8266_mqtt")) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falha (rc=");
      Serial.print(MQTT.state());
      Serial.println("). Tentando novamente...");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializa o sensor SHT20
  sht20.initSHT20();
  delay(100);

  // Configura o pino do encoder
  pinMode(pinoEncoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinoEncoder), contarPulsos, FALLING);

  // Inicializa Wi-Fi e MQTT
  initWiFi();
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
}

void loop() {
  // Reconexão Wi-Fi e MQTT
  if (!WiFi.isConnected()) initWiFi();
  if (!MQTT.connected()) reconnectMQTT();
  MQTT.loop();

  // Leitura do sensor SHT20
  float temperatura = sht20.readTemperature();
  float umidade = sht20.readHumidity();

  // Cálculo do RPM e velocidade
  if (millis() - millis_old >= 1000) {
    detachInterrupt(digitalPinToInterrupt(pinoEncoder));

    rpm = (60.0 * 1000.0 / pulsos_por_volta) * pulsos / (millis() - millis_old);
    ms = (2 * pi * 0.01 * rpm) / 60; // Considerando um raio de 0.01m
    kmh = ms * convert;

    millis_old = millis();
    pulsos = 0;

    attachInterrupt(digitalPinToInterrupt(pinoEncoder), contarPulsos, FALLING);
  }

  // Publicação dos dados no MQTT
  sprintf(strTemperature, "%.2f", temperatura);
  sprintf(strHumidity, "%.2f", umidade);
  sprintf(strRPM, "%d", rpm);
  sprintf(strSpeed, "%.2f", kmh);

  MQTT.publish(TOPIC_TEMPERATURE, strTemperature);
  MQTT.publish(TOPIC_HUMIDITY, strHumidity);
  MQTT.publish(TOPIC_RPM, strRPM);
  MQTT.publish(TOPIC_SPEED, strSpeed);

  // Imprime os dados no Serial Monitor
  Serial.print("Temperatura: ");
  Serial.print(temperatura, 2);
  Serial.print("C Umidade: ");
  Serial.print(umidade, 2);
  Serial.print("% RPM: ");
  Serial.print(rpm);
  Serial.print(" Velocidade: ");
  Serial.print(kmh, 2);
  Serial.println(" km/h");

  delay(1000);
}
