#include <Wire.h>
#include <SimpleFOC.h>

// ---------------------------------------------------------------------------
// Configuração do sensor MT6701 via I2C
// Endereço: 0x06
// Resolução: 14 bits (16384 posições por volta)
// Registrador inicial do ângulo: 0x03
// ---------------------------------------------------------------------------
MagneticSensorI2C sensor = MagneticSensorI2C(0x06, 14, 0x03, 8);

// Intervalo de exibição via Serial (em milissegundos)
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 100;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Inicializa o barramento I2C nos pinos configurados para a ESP32
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz (Fast Mode)

  // Inicializa o sensor MT6701 na SimpleFOC
  sensor.init();

  Serial.println("=================================================");
  Serial.println(" SimpleFOC - Teste do Sensor MT6701 (I2C)");
  Serial.println("=================================================");
}

void loop() {
  // A função update() deve ser chamada continuamente para atualizar as leituras do sensor
  sensor.update();

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();

    // getMechanicalAngle() retorna o ângulo mecânico no intervalo 0 a 2PI rad
    float angleRad = sensor.getMechanicalAngle();

    // getVelocity() retorna a velocidade angular instantânea em rad/s
    float velocityRadS = sensor.getVelocity();

    // Determina o sentido de rotação com base no sinal da velocidade
    const char* dirStr = (velocityRadS > 0.1f)  ? "CW " :
                         (velocityRadS < -0.1f) ? "CCW" : " - ";

    Serial.print("Angulo: ");
    Serial.print(angleRad, 4);
    Serial.print(" rad | Velocidade: ");
    Serial.print(velocityRadS, 4);
    Serial.print(" rad/s | Sentido: ");
    Serial.println(dirStr);
  }
}