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
  
  digitalRead(Componente); // Leitura de componentes digitais
  digitalWrite(ledPin, LOW); // Escrita para componentes digitais
  analogRead(Componente); // Leitura de componentes analogicos
  analogWrite(Componente, Valor); // Escrita para componentes analogicos

  Serial.print("Hello world!");
}

void loop() {
  delay(2000);
  Serial.println("Hello world!");
}
