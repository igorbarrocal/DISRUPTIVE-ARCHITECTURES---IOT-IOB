#include <Arduino.h>

constexpr int ADC_POT1 = 34;    // POT1: sinal
constexpr int ADC_POT2 = 35;    // POT2: limiar
constexpr gpio_num_t LED_PROCESS = GPIO_NUM_4; 
constexpr gpio_num_t LED_LIMIT   = GPIO_NUM_2; 

constexpr int FS_HZ = 1000;     // taxa de amostragem
constexpr int WIN_MS = 200;     // janela de 200 ms
constexpr int N = FS_HZ * WIN_MS / 1000;

static uint16_t buf1[N], buf2[N];

void setup() {
  Serial.begin(115200);
  pinMode(LED_PROCESS, OUTPUT);
  pinMode(LED_LIMIT, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_POT1, ADC_11db);
  analogSetPinAttenuation(ADC_POT2, ADC_11db);

  Serial.println("#seq,ts_ms,sample,pot1_raw,pot2_raw"); // cabeçalho CSV
}

void loop() {
  static uint32_t seq = 0;
  const uint32_t Ts_us = 1000000UL / FS_HZ;
  uint32_t t0 = millis();

  // 1) Coleta da janela
  for (int i = 0; i < N; ++i) {
    buf1[i] = analogRead(ADC_POT1);
    buf2[i] = analogRead(ADC_POT2);
    delayMicroseconds(Ts_us);
  }

  // 2) Impressão em formato CSV
  uint32_t ts = millis() - t0;
  for (int i = 0; i < N; ++i) {
    Serial.print(seq); Serial.print(',');
    Serial.print(ts);  Serial.print(',');
    Serial.print(i);   Serial.print(',');
    Serial.print(buf1[i]); Serial.print(',');
    Serial.println(buf2[i]);
  }
  seq++;

  // 3) Feedback visual: LED_PROCESS pisca
  digitalWrite(LED_PROCESS, HIGH); delay(20); digitalWrite(LED_PROCESS, LOW);

  // 4) Comandos vindos do Python
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("LED ON"))  digitalWrite(LED_LIMIT, HIGH);
    if (cmd.equalsIgnoreCase("LED OFF")) digitalWrite(LED_LIMIT, LOW);
  }
}