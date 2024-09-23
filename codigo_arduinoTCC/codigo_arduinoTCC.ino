#include "DFRobot_SHT20.h"

// Defina o endereço I2C do SHT20
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

volatile int pulsos = 0; // Contador de pulsos do encoder
int rpm = 0;

float ms = 0;
float pi = 3.1415;
float convert = 3.6;
float kmh = 0;

const int pinoEncoder = 2; // Pino do encoder
const int pinoMotor = 3;   // Pino do motor
const int pulsos_por_volta = 20; // Ajuste conforme o seu encoder

unsigned long millis_old = 0;

void contador() {
  // Incrementa contador
  pulsos++;
}

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor SHT20
  sht20.initSHT20();
  delay(100);
  Serial.println("Inicialização do sensor concluída!");

  // Configura o pino do encoder
  pinMode(pinoEncoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinoEncoder), contador, FALLING);

  // Configura o pino do motor
  pinMode(pinoMotor, OUTPUT);
}

void loop() {
  // Lê a umidade
  float umidade = sht20.readHumidity();
  // Lê a temperatura
  float temperatura = sht20.readTemperature();

  // Atualiza contador a cada 1000 milissegundos, ou seja, 1 segundo
  if (millis() - millis_old >= 1000) {
    // Desabilita interrupção enquanto faz o cálculo
    detachInterrupt(digitalPinToInterrupt(pinoEncoder));

    // Cálculo de RPM
    rpm = (60.0 * 1000.0 / pulsos_por_volta) * pulsos / (millis() - millis_old);
    
    // Calcula a velocidade em m/s
    ms = (2 * pi * 0.01 * rpm) / 60; // Considerando um raio de 0.01m (ajuste conforme necessário)
    kmh = ms * convert;

    // Reseta os pulsos e o tempo
    millis_old = millis();
    pulsos = 0;

    // Habilita interrupção novamente
    attachInterrupt(digitalPinToInterrupt(pinoEncoder), contador, FALLING);
  }

  // Imprime dados no Serial Monitor
  Serial.print("Tempo: ");
  Serial.print(millis());   // Tempo do sistema Arduino
  Serial.print(" Temperatura: ");
  Serial.print(temperatura, 1);   // Apenas uma casa decimal
  Serial.print("C");
  Serial.print(" Umidade: ");
  Serial.print(umidade, 1);   // Apenas uma casa decimal
  Serial.print("%");
  Serial.print(" RPM: ");
  Serial.print(rpm);
  Serial.print(" Velocidade: ");
  Serial.print(kmh);
  Serial.print(" km/h ");
  Serial.print(ms);
  Serial.println(" m/s");
  
  // Controle do motor (exemplo: ligar por 2 segundos, depois desligar)
  digitalWrite(pinoMotor, HIGH); // Liga o motor
  delay(2000); // Motor ligado por 2 segundos
  digitalWrite(pinoMotor, LOW);  // Desliga o motor

  delay(1000); // Atraso antes da próxima leitura
}

// Função chamada quando o encoder detecta um pulso
void contarEncoder() {
  pulsos++;
}
