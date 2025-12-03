#include <Arduino.h>
#include <Update.h>
#include <DHT.h>

//input pin
const int buttonPin = 45;
const int pinPot = 15;
const int pinPhot = 1;
const int pino_trigger = 4;
const int pino_echo = 5;

//output pin
const int buzzerPin = 47;
const int redPin = 17;
const int greenPin = 18;
const int bluePin = 21;

long duration; // Variável para armazenar a duração do pulso (Manter para Sensor Ultrassonico)
int distanceCm; // Variável para armazenar a distância em cm

// Pino e tipo do DHT, alem de inicializar o DHT (Para DHT11, altere apenas o tipo)
#define DHTPIN         10
#define DHTTYPE        DHT22

DHT dht(DHTPIN, DHTTYPE);


//constantes para leitura do sensor de iluminação (Manter para LDR)
const float RESISTOR_FIXO = 10000;
const float GAMMA = 0.7;
const float RL10 = 50;

// ----- VARIÁVEIS DE INTERRUPÇÃO E DEBOUNCE (Utilizar para Debounce em botao) -----
volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;
volatile bool buttonState = false;
volatile int bounceCount = 0;

// Codigo para interrupção externa no botao, copie e cole no código
void IRAM_ATTR handleButtonISR() {
  unsigned long now = millis();
  if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
    // le a borda
    if (digitalRead(buttonPin) == LOW) {
      // alterna estado lógico
      buttonState = !buttonState;
    }
    bounceCount++;
    lastDebounceTime = now;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello World!");
  dht.begin(); // Incializa o DHT

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pino_trigger, OUTPUT);

  pinMode(pinPhot, INPUT);
  pinMode(pinPot, INPUT);
  pinMode(pino_echo, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonISR, FALLING); // Ativa a interrupção externa, copie e cole junto com o código de interrupção externa acima

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  noTone(buzzerPin); // Desativa o buzzer

  /* USO DE ULTRASSONICO
  Copiar e colar etapas a seguir para medir distancia em CM
  */
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


  
  float temperatura = dht.readTemperature(); // Le a temperatura do DHT

  bool perigo = (temperatura < 0 || temperatura > 25);

  // Leitura do sensor de iluminacao
  int fotoValue = analogRead(pinPhot); // Fotorresistor
  float fotoVoltage = fotoValue * 3.3 / 4095.0;
  if (fotoVoltage >= 3.29) fotoVoltage = 3.29;
  float fotoResistance = RESISTOR_FIXO * fotoVoltage / (3.3 - fotoVoltage); //divisor de tensao
  if (fotoResistance <= 0) fotoResistance = 1;
  float lux = pow((RL10 * 1000 * pow(10, GAMMA)) / fotoResistance, 1.0 / GAMMA);

  bool hasOnlyAmbientLight = (lux < 2000); // Luz detectada

  //leitura do potenciometro bruto
  int potValue = analogRead(pinPot);

  // Cores RGB com base no potenciômetro
  int r = 0, g = 0, b = 0;

  // Colocar esse IF/ELSE para desligar tudo quando o botao é pressionado
  if(buttonState){
      noTone(buzzerPin);
      Serial.println("Sistema desativado");

      analogWrite(redPin,   0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin,  0);
    } else {
        if(perigo || distanceCm < 100)
        {
          potValue = 0;
          tone(buzzerPin, 1000);
          Serial.println("PERIGO");

          analogWrite(redPin,   0);
          analogWrite(greenPin, 0);
          analogWrite(bluePin,  0);

          delay(1000);

          return;
        }

        if(hasOnlyAmbientLight && !perigo)
        {
          if (potValue < 1365) {
            // map(var, A, B, C, D): le o valor do potenciometro e converte do range de A -> B para C -> D
            // Vermelho → Laranja → Amarelo
            r = 255;
            g = map(potValue, 0, 1364, 0, 254); //165
            b = 0;
          } else if (potValue < 2730) {
            // Amarelo → Branco
            r = 255;
            g = 255;
            b = map(potValue, 1364, 2729, 0, 255);
          } else {
            // Branco → Azul Turquesa → Azul
            r =  map(potValue, 2730, 4095, 255, 0); // f(a) = 765 - (51 / 273) * a
            g =  map(potValue, 2730, 4095, 255, 0);  // f(a) = 765 - (51 / 273) * a
            b = 255;
          }

          Serial.println("LED: ON");
          Serial.print("RGB: ");
          Serial.println((String)r + ", " + (String)g + ", "+ (String)b);
          Serial.print("Temperatura: ");
          Serial.println(temperatura);
          Serial.print("Pot: ");
          Serial.println(potValue);
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
