const int ledPin = 2; // Define pinos no ESP

/* INFORMAÇÕES IMPORTANTES:
No Arduino IDE:
Selecione a placa: ESP32 S3 DEVMODULE
Selecione a porta
Conecte o ESP32 S3 na porta COM
*/

/* PINOS RECOMENDADOS ESP32 S3:
Teoricamente todos os pinos podem servir como analogicos e digitais, porém evite pinos em extremidades como o 0

Idealmente se o WIFI for utilizado, para leitura analogica priorize os pinos de ADC 1:
1, 2, 3, 4, 5, 6, 7, 8, 9, 10

Fora isso, utilize livremente os pinos, porem evite pinos como o 0 ou pinos com numeros muito altos

(Obrigado Rick Muds pelo aulão e contribuição da pinagem)

PINOS RECOMENDADOS ESP32 NORMAL:
Analogica: 30, 31, 32, 33, 34, 35
Digital: Qualquer outra
Placa ESP32 Normal: ESP32 DEVKITV1 ou DOITY alguma coisa
*/


void setup() {
  Serial.begin(115200); // Obrigatorio
  pinMode(ledPin, OUTPUT); // INPUT para leitores, OUTPUT para saidas
  
  digitalRead(Componente); // Leitura de componentes digitais (
  digitalWrite(ledPin, LOW); // Escrita para componentes digitais
  analogRead(Componente); // Leitura de componentes analogicos
  analogWrite(Componente, Valor); // Escrita para componentes analogicos


digitalWrite(ledgreen, LOW); // quando o LED esta conectado ao VCC, quando bota que ele ta low ele liga, e quando bota HIGH desliga, 
  e quando esta conectado ao ground é diferente (conecte no ground)


digital write serve para ligar e desligar pinos (LED, BUZZER, ) // DIGITAL READ SERVE PARA LER O ESTADO DE UM PINO(ON ou OFF) // 
ANALOG READ serve para ler o estado de um fotoressistor, DHT, potenciometro,etc


digitalWrite(buzzer, HIGH)   buzzer ativo

  tone(buzzer, 1000);   // toca 1000 Hz buzzer passivo
  delay(500);
  

| Componente        | digitalRead | digitalWrite | analogRead | analogWrite         |
| ----------------- | ----------- | ------------ | ---------- | ------------------- |
| **Botão**         | ✔           | —            | ❌          | ❌                   |
| **Fotoresistor**  | ❌           | —            | ✔          | ❌                   |
| **DHT11/22**      | ❌           | ❌            | ❌          | ❌                   |
| **LED**           | ❌           | ✔            | ❌          | ✔ (brilho)          |
| **Ultrassônico**  | ❌           | ✔ (TRIG)     | ❌          | ❌                   |
| **Potenciômetro** | ❌           | —            | ✔          | ❌                   |
| **Servo**         | ❌           | ❌            | ❌          | ❌ (comando próprio) |
| **LED RGB**       | ❌           | ✔            | ❌          | ✔                   |
| **Buzzer**        | ❌           | ✔            | ❌          | ✔                   |




int state = digitalRead(button1);

if (state == LOW) {
  Serial.println("button1: ON");   // pressionado
} else {
  Serial.println("button1: OFF");  // solto
}

delay(200);



INPUT_PULLUP = o pino fica “puxado para cima” automaticamente.

Assim:

Botão solto → HIGH (1)

Botão apertado → LOW (0) porque liga o pino ao GND.

  

  Serial.print("Hello world!");
}

void loop() {
  delay(2000);
  Serial.println("Hello world!");
}




void loop() {
  float temperatura = dht.readTemperature();
  float umidade = dht.readHumidity();

  Serial.print("Temperatura: ");
  Serial.println(temperatura);

  Serial.print("Umidade: ");
  Serial.println(umidade);

  delay(2000);
}


 int potValue = analogRead(potenc);   lendo o potenciometro
  Serial.print("Potenciometro: ");
  Serial.println(potValue);



int fotoValue = analogRead(fotoPin);   // lendo o fotoresistor
Serial.print("Fotoresistor (ADC): ");
Serial.println(fotoValue);



int ledState = digitalRead(ledPin);

Serial.print("LED: ");
Serial.println(ledState);






void loop() {
  float umidade = dht.readHumidity();
  float temperatura = dht.readTemperature();

  Serial.print("Temp: ");
  Serial.print(temperatura);
  Serial.print(" °C | Umidade: ");
  Serial.print(umidade);
  Serial.println(" %");

  delay(2000);
}


#include <DHT.h>

// ---------- PINOS ----------
#define BOTAO_PIN 10

#define POT_PIN   1
#define FOTO_PIN  2

#define DHT_PIN   4
#define DHT_TYPE  DHT22   // ou DHT11

// ---------- OBJETO DHT ----------
DHT dht(DHT_PIN, DHT_TYPE);

// ---------- CONTROLE ----------
bool lendo = true;
bool ultimoEstadoBotao = HIGH;

void setup() {
  Serial.begin(115200);

  pinMode(BOTAO_PIN, INPUT_PULLUP);

  dht.begin();
}

void loop() {

  // -------- BOTÃO (TOGGLE) --------
  bool estadoBotao = digitalRead(BOTAO_PIN);

  if (estadoBotao == LOW && ultimoEstadoBotao == HIGH) {
    lendo = !lendo;      // pausa / volta
    delay(200);          // debounce simples
  }

  ultimoEstadoBotao = estadoBotao;

  // -------- LEITURAS --------
  if (lendo) {
    int potValue  = analogRead(POT_PIN);
    int fotoValue = analogRead(FOTO_PIN);

    float temperatura = dht.readTemperature();
    float umidade     = dht.readHumidity();

    Serial.println("---- LEITURAS ----");

    Serial.print("Potenciometro: ");
    Serial.println(potValue);

    Serial.print("Fotoresistor: ");
    Serial.println(fotoValue);

    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");

    Serial.print("Umidade: ");
    Serial.print(umidade);
    Serial.println(" %");

    Serial.println();
  }

  delay(1000);
}




