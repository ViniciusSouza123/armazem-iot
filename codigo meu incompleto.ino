#define button1 4
#define button3 18
#define button2 6
#define potenc 16
#define ledred 12
#define ledgreen 13

volatile bool systemPaused = false;      // controla se o sistema está pausado
volatile bool interruptFlag = false;     // indica que houve interrupção

// ======= INTERRUPÇÃO DO BOTÃO 3 =======
void IRAM_ATTR toggleSystem() {
  interruptFlag = true;   // marca que o botão 3 foi pressionado
}

void setup() {
  Serial.begin(115200);
  Serial.println("OH DAMN");

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  pinMode(ledgreen, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(potenc, INPUT);

  // LEDs começam apagados (ligados ao GND)
  digitalWrite(ledgreen, LOW);
  digitalWrite(ledred, LOW);

  // INTERRUPÇÃO NO BUTTON3 (detecta borda de descida)
  attachInterrupt(digitalPinToInterrupt(button3), toggleSystem, FALLING);
}

void loop() {

  int potValue = analogRead(potenc);
  Serial.print("Potenciometro: ");
  Serial.println(potValue);

  // ======= TRATAMENTO DA INTERRUPÇÃO =======
  if (interruptFlag) {
    interruptFlag = false;       // limpa a flag

    systemPaused = !systemPaused;  // alterna ativo/pausado

    if (systemPaused) {
      Serial.println("SISTEMA PAUSADO!");
      digitalWrite(ledgreen, LOW);
      digitalWrite(ledred, LOW);
    } else {
      Serial.println("SISTEMA ATIVO!");
    }

    delay(200); // debounce leve
  }


  // ====== SE O SISTEMA ESTIVER PAUSADO, NÃO FUNCIONA BOTÃO 1/2 ======
  if (systemPaused) return;


  // ====== TOGGLE DO LED VERDE (button1) ======
  static bool ledGreenState = false;
  static bool lastButton1 = HIGH;

  int button1State = digitalRead(button1);

  if (lastButton1 == HIGH && button1State == LOW) {
    ledGreenState = !ledGreenState;
    digitalWrite(ledgreen, ledGreenState ? HIGH : LOW);
  }

  lastButton1 = button1State;


  // ====== TOGGLE DO LED VERMELHO (button2) ======
  static bool ledRedState = false;
  static bool lastButton2 = HIGH;

  int button2State = digitalRead(button2);

  if (lastButton2 == HIGH && button2State == LOW) {
    ledRedState = !ledRedState;
    digitalWrite(ledred, ledRedState ? HIGH : LOW);
  }

  lastButton2 = button2State;

  delay(30);
}
