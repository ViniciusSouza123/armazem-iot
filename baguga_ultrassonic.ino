#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h> 
#include <Arduino.h>
#include <Update.h>

//input pin
const int buttonPin = 45;
const int pinPhot = 1;
const int pino_trigger = 4;
const int pino_echo = 5;
const int redLed = 3;
const int blueLed = 8;
const int botaoPin = 2;

// Variável para rastrear o estado atual do sistema
// true = LED 1 Aceso / LED 2 Apagado
// false = LED 1 Apagado / LED 2 Aceso
bool estadoAtualLED = true; 

// Variável para debounce e detecção de borda
int estadoAnteriorBotao = HIGH;

//output pin
const int buzzerPin = 47;
const int redPin = 17;
const int greenPin = 18;
const int bluePin = 21;

long duration; // Variável para armazenar a duração do pulso
int distanceCm; // Variável para armazenar a distância em cm

//temperatura
#define DHTPIN         10
#define DHTTYPE        DHT22

DHT dht(DHTPIN, DHTTYPE);

//constantes para leitura do sensor de iluminação
const float RESISTOR_FIXO = 10000;
const float GAMMA = 0.7;
const float RL10 = 50;

// ----- VARIÁVEIS DE INTERRUPÇÃO E DEBOUNCE -----
volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;
volatile bool buttonState = false;
volatile int bounceCount = 0;

void IRAM_ATTR handleButtonISR() {
  unsigned long now = millis();
  if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
    // le a borda
    if (digitalRead(buttonPin) == LOW) {
      // alterna estado lógico (isso apenas atualiza flag; o envio ao Cloud acontece no loop)
      buttonState = !buttonState;
    }
    bounceCount++;
    lastDebounceTime = now;
  }
}

Uma interrupção (ISR) que dispara quando o botão é pressionado

Um debounce interno, evitando múltiplos disparos falsos

Alterna uma variável buttonState somente quando o botão é realmente pressionado

Conta quantos disparos ocorreram (bounceCount) — útil para debug




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("André é faggot!!!");
  dht.begin();

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pino_trigger, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  pinMode(pinPhot, INPUT);
  pinMode(pino_echo, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(botaoPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonISR, FALLING);

  // Define o estado inicial
  digitalWrite(blueLed, HIGH);  // LED 1 Aceso (HIGH)
  digitalWrite(redLed, LOW);   // LED 2 Apagado (LOW)

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10); // this speeds up the simulation
  noTone(buzzerPin);

  // 1. Leitura do estado atual do botão
  int estadoAtualBotao = digitalRead(botaoPin);

  // 1. Limpa o pino TRIG para garantir um pulso limpo (LOW)
  digitalWrite(pino_trigger, LOW);
  delayMicroseconds(2); // Pequeno atraso

  // 2. Envia um pulso de 10 microssegundos (HIGH) no pino TRIG
  digitalWrite(pino_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pino_trigger, LOW);

  // 3. Mede a duração do pulso no pino ECHO
  // pulseIn() retorna o tempo em microssegundos que o pino ECHO ficou HIGH
  duration = pulseIn(pino_echo, HIGH);

  // 4. Calcula a distância
  // A distância é calculada pela fórmula: (duração / 2) / 29.1, ou simplesmente
  // distância = duração / 58
  distanceCm = duration / 58;

  float temperatura = dht.readTemperature();

  bool perigo = (temperatura < 0 || temperatura > 25);

  // Leitura do sensor de iluminacao
  int fotoValue = analogRead(pinPhot); // Fotorresistor
  float fotoVoltage = fotoValue * 3.3 / 4095.0;
  if (fotoVoltage >= 3.29) fotoVoltage = 3.29;
  float fotoResistance = RESISTOR_FIXO * fotoVoltage / (3.3 - fotoVoltage); //divisor de tensao
  if (fotoResistance <= 0) fotoResistance = 1;
  float lux = pow((RL10 * 1000 * pow(10, GAMMA)) / fotoResistance, 1.0 / GAMMA);

  bool hasOnlyAmbientLight = (lux < 2000); // Luz detectada

  // Cores RGB com base no potenciômetro
  int r = 0, g = 0, b = 0;

  // 2. Detecção de Borda de Descida (Button Press)
  // O botão foi pressionado (passou de HIGH para LOW)
  if (estadoAnteriorBotao == HIGH && estadoAtualBotao == LOW) {
    
    // 3. Inverte o estado do sistema
    estadoAtualLED = !estadoAtualLED; // Inverte true para false, ou false para true
    
    // 4. Aplica o novo estado aos LEDs
    if (estadoAtualLED) {
      digitalWrite(blueLed, HIGH);  // LED 1 Aceso
      digitalWrite(redLed, LOW);   // LED 2 Apagado
    } else {
      digitalWrite(blueLed, LOW);   // LED 1 Apagado
      digitalWrite(redLed, HIGH);  // LED 2 Aceso
    }
    
    // Pequeno atraso para "debounce" e evitar múltiplas leituras de um único clique
    delay(50); 
  }

  // 5. Atualiza o estado anterior do botão para a próxima iteração
  estadoAnteriorBotao = estadoAtualBotao;

  if(buttonState){
      noTone(buzzerPin);
      Serial.println("Sistema desativado!!! ");

      analogWrite(redPin,   0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin,  0);

      delay(1000);
    } else {
        if(perigo)
        {
          tone(buzzerPin, 1000);
          Serial.println("!!!PERIGO, CAGUEI NAS CALÇAS!!!");

          analogWrite(redPin,   0);
          analogWrite(greenPin, 0);
          analogWrite(bluePin,  0);

          delay(1000);

          return;
        }

        if(hasOnlyAmbientLight && !perigo)
        {
          if (distanceCm < 133) {
            // Vermelho → Laranja → Amarelo
            r = 255;
            g = map(distanceCm, 0, 133, 0, 254); //165
            b = 0;
          } else if (distanceCm < 266) {
            // Amarelo → Branco
            r = 255;
            g = 255;
            b = map(distanceCm, 133, 266, 0, 255);
          } else {
            // Branco → Azul Turquesa → Azul
            r =  map(distanceCm, 266, 399, 255, 0); // f(a) = 765 - (51 / 273) * a
            g =  map(distanceCm, 266, 399, 255, 0);  // f(a) = 765 - (51 / 273) * a
            b = 255;
          }

          Serial.println("LED: ON");
          Serial.print("RGB: ");
          Serial.println((String)r + ", " + (String)g + ", "+ (String)b);
          Serial.print("Temperatura: ");
          Serial.println(temperatura);

          // 5. Exibe os valores no Serial Monitor
          Serial.print("Distancia: ");
          Serial.print(distanceCm);
          Serial.println(" cm");

          analogWrite(redPin,   r);
          analogWrite(greenPin, g);
          analogWrite(bluePin,  b);

          delay(1000);

          return;
        } else
        {
          Serial.println("LED: OFF");

          analogWrite(redPin,   0);
          analogWrite(greenPin, 0);
          analogWrite(bluePin,  0);

          delay(1000);
        }
      }
  delay(10);
}
