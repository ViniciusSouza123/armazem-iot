#include <WiFi.h>
#include <PubSubClient.h>

// --- Configurações de Rede e MQTT ---
const char* ssid = "Wokwi-GUEST"; // Rede padrão do Wokwi (ALTERE PARA NOME DO WIFI)
const char* password = "";        // Senha (ALTERE PARA SENHA DO WIFI)
const char* mqtt_server = "broker.hivemq.com"; // Broker MQTT público
const int mqtt_port = 1883; // Porta (MANTER)

// --- Tópicos MQTT ---
const char* topic_comando = "guga/esp32/componente/comando"; // Tópico para receber comandos (Livre para alterar, coloque o caminho que voce quiser)
const char* topic_status = "guga/esp32/componente/status";   // Tópico para publicar o status (Livre para alterar, coloque o caminho que voce quiser)

// --- Configuração do LED ---
const int ledPin = 2; // O LED interno do ESP32 ou o LED D2, comum no Wokwi

// --- Objetos de Conexão ---
WiFiClient espClient;
PubSubClient client(espClient);

// --------------------------------------------------------------------------------
/* SETUP DO WIFI, APENAS COPIAR E COLAR
NO APLICATIVO NO CELULAR:
Adicione conexão, adicione o ClientID (ESP32ClientGuga)
Adicione os tipos de paineis:
Panel name: Tanto faz, apenas nome
Topic: coloque o caminho igual ao caminho dos topics
Payload on: Mensagem passada quando liga o switch
Payload off: Mensagem passada quando desliga o switch
Altere as mensagens do Payload de acordo com o IF la embaixo
*/


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

// Função de Callback: Chamada quando uma mensagem MQTT é recebida (COPIAR E COLAR TAMBEM)
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

  // Verifica o tópico e o comando (Altere os IFs da mensagem de acordo com as mensagens escritas no payload do MQTT)
  if (String(topic) == topic_comando) {
    if (message == "LIGAR") {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED LIGADO!");
      client.publish(topic_status, "LED LIGADO"); // Publica o status
    } else if (message == "DESLIGAR") {
      digitalWrite(ledPin, LOW);
      Serial.println("LED DESLIGADO!");
      client.publish(topic_status, "LED DESLIGADO"); // Publica o status
    } else {
      Serial.println("Comando inválido. Use LIGAR ou DESLIGAR.");
    }
  }
}

void reconnect() {
  // Loop até que estejamos conectados
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    // Tenta se conectar com um ID de cliente único
    String clientId = "ESP32Clientvini";
    
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

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  
  // Desliga o LED no início
  digitalWrite(ledPin, LOW); 

  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Processa mensagens recebidas e mantém o "ping"
}
