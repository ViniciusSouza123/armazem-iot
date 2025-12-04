#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h> 
#include <Arduino.h>
#include <Update.h>

// --- Configurações de Rede e MQTT ---
const char* ssid = "Wokwi-GUEST"; // Rede padrão do Wokwi
const char* password = "";        // Nenhuma senha
const char* mqtt_server = "broker.hivemq.com"; // Broker MQTT público
const int mqtt_port = 1883;

// --- Variáveis Globais para Controle de Tempo ---
long lastMsg = 0;
const int INTERVALO_PUB = 5000; // Publicar a cada 5 segundos (5000 ms)

const char* topic_comando = "dispositivo/esp32/led/comando"; // Tópico para receber comandos
const char* topic_status = "dispositivo/esp32/led/status";   // Tópico para publicar o status

const char* topic_status_led = "dispositivo/led/status"; 
const char* topic_status_sonic = "dispositivo/sonic/status"; 
const char* topic_status_dht = "dispositivo/dht/status"; 
const char* topic_status_pot = "dispositivo/pot/status"; 
const char* topic_status_ldr = "dispositivo/ldr/status"; 
const char* topic_comando_button = "dispositivo/button/command"; 

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

// --- Objetos de Conexão ---
WiFiClient espClient;
PubSubClient client(espClient);

// --------------------------------------------------------------------------------

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// Função de Callback: Chamada quando uma mensagem MQTT é recebida
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: [");
  Serial.print(topic);
  Serial.print("] ");

  // Converte o payload para String e imprime
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

}

void reconnect() {
  // Loop até que estejamos conectados
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    // Tenta se conectar com um ID de cliente único
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Tenta conectar
    if (client.connect(clientId.c_str())) {
      Serial.println("conectado");
      // Uma vez conectado, inscreva-se no tópico de comando
      client.subscribe(topic_comando);
      Serial.print("Inscrito no tópico: ");
      Serial.println(topic_comando);
      
      // Publica uma mensagem de inicialização
      client.publish(topic_status, "DISPOSITIVO INICIADO");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" Tente novamente em 5 segundos");
      // Espera 5 segundos antes de tentar novamente
      delay(5000);
    }
  }
}

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

// Função para ler todos os sensores e publicar os dados no MQTT
void publishSensorData() {
  
  // --- Leitura dos Dados (Você já faz isso no loop, mas vamos centralizar aqui) ---
  
  // Distância Ultrassônica (necessário recriar a medição aqui ou passar como parâmetro/global)
  // Como a medição está no loop(), usaremos a variável global 'distanceCm'
  String distanceStr = String(distanceCm);
  client.publish(topic_status_sonic, distanceStr.c_str());
  Serial.print("MQTT PUB [Distância]: "); Serial.println(distanceStr);
  
  // Temperatura DHT
  float temperatura = dht.readTemperature();
  String tempStr = String(temperatura, 1); // 1 casa decimal
  client.publish(topic_status_dht, tempStr.c_str());
  Serial.print("MQTT PUB [Temperatura]: "); Serial.println(tempStr);

  // Potenciômetro
  int potValue = analogRead(pinPot);
  String potStr = String(potValue);
  client.publish(topic_status_pot, potStr.c_str());
  Serial.print("MQTT PUB [Potenciômetro]: "); Serial.println(potStr);

  // LDR (Lux, usando o cálculo que você já tem)
  // Para evitar repetição de código, você pode mover a lógica do LDR para uma função separada,
  // ou simplesmente recalcular o lux aqui, como abaixo:
  int fotoValue = analogRead(pinPhot);
  float fotoVoltage = fotoValue * 3.3 / 4095.0;
  if (fotoVoltage >= 3.29) fotoVoltage = 3.29;
  float fotoResistance = RESISTOR_FIXO * fotoVoltage / (3.3 - fotoVoltage);
  if (fotoResistance <= 0) fotoResistance = 1;
  float lux = pow((RL10 * 1000 * pow(10, GAMMA)) / fotoResistance, 1.0 / GAMMA);
  
  String luxStr = String(lux, 0); // Sem casas decimais
  client.publish(topic_status_ldr, luxStr.c_str());
  Serial.print("MQTT PUB [Lux]: "); Serial.println(luxStr);

  // Botão (Status Lógico)
  String buttonStr = buttonState ? "OFF" : "ON"; // Lógica inversa: 'buttonState' true = sistema desativado
  client.publish(topic_comando_button, buttonStr.c_str());
  Serial.print("MQTT PUB [Botão/Sistema]: "); Serial.println(buttonStr);
  
  // Se quiser enviar o status da cor do LED (r, g, b), você precisaria calcular r, g, b aqui também,
  // ou criar variáveis globais para eles. Para simplificar, vou mandar só o status ON/OFF do LED geral.
  String ledStatus = (buttonState || temperatura < 0 || temperatura > 25 || distanceCm < 100) ? "OFF" : "ON";
  client.publish(topic_status_led, ledStatus.c_str());
  Serial.print("MQTT PUB [LED Status]: "); Serial.println(ledStatus);

}

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

  pinMode(pinPhot, INPUT);
  pinMode(pinPot, INPUT);
  pinMode(pino_echo, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonISR, FALLING);

  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10); // this speeds up the simulation

  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Processa mensagens recebidas e mantém o "ping"

  noTone(buzzerPin);

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

  //leitura do potenciometro bruto
  int potValue = analogRead(pinPot);

  // Cores RGB com base no potenciômetro
  int r = 0, g = 0, b = 0;

  long now = millis();
  if (now - lastMsg > INTERVALO_PUB) {
    lastMsg = now;
    publishSensorData(); // CHAMA A NOVA FUNÇÃO A CADA 5 SEGUNDOS
  }

  if(buttonState){
      noTone(buzzerPin);
      Serial.println("Sistema desativado!!! ");

      analogWrite(redPin,   0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin,  0);
    } else {
        if(perigo || distanceCm < 100)
        {
          potValue = 0;
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
          if (potValue < 1365) {
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
