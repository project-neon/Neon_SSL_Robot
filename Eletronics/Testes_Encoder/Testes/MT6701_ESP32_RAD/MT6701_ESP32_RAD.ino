/*
 * ============================================================================
 *  Leitura do encoder magnético MT6701 (modo I2C) - ESP32
 *  Objetivo: ler ângulo, calcular RPM e identificar sentido de rotação
 *
 *  Depois de validar aqui na ESP32, a mesma lógica pode ser portada para o
 *  STM32 (BluePill) trocando apenas a camada de I2C (Wire -> HAL_I2C).
 *  Por isso o código está separado em:
 *    1) Camada de leitura I2C bruta (mt6701_readAngleRaw)
 *    2) Camada de conversão física (contagens -> graus)
 *    3) Camada de cálculo de velocidade (RPM + sentido)
 * ============================================================================
 */

#include <Wire.h>

// ---------------------------------------------------------------------------
// Configurações do sensor
// ---------------------------------------------------------------------------
#define MT6701_I2C_ADDR   0x06   // Endereço fixo do MT6701 no modo I2C

// Registradores do MT6701 (datasheet - modo I2C):
// 0x03 -> ANGLE_H : bits [13:6] do ângulo (8 bits mais significativos)
// 0x04 -> ANGLE_L : bits [5:0] do ângulo, alinhados nos bits [7:2] do byte
#define MT6701_REG_ANGLE_H  0x03
#define MT6701_REG_ANGLE_L  0x04

#define MT6701_RESOLUTION   16384.0f  // 14 bits => 2^14 contagens por volta (0..16383)

// ---------------------------------------------------------------------------
// Pinos I2C na ESP32 (ajuste conforme sua fiação)
// Muitas placas ESP32 DevKit usam por padrão SDA=21, SCL=22
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define I2C_CLOCK_HZ  400000UL   // 400kHz (Fast Mode) - o MT6701 suporta

// ---------------------------------------------------------------------------
// Variáveis de estado para cálculo de velocidade
// ---------------------------------------------------------------------------
int32_t  lastRawCount   = 0;      // última leitura bruta (0..16383)
uint32_t lastTimeMicros = 0;      // timestamp da última leitura
bool     firstReading = true;

// Filtro simples (média móvel) para suavizar a velocidade exibida
#define VELOCITY_FILTER_SAMPLES 5
float velocityBuffer[VELOCITY_FILTER_SAMPLES] = {0};
uint8_t velocityBufferIndex = 0;

// ---------------------------------------------------------------------------
// 1) CAMADA DE LEITURA I2C BRUTA
//    Lê os dois registradores de ângulo e monta o valor de 14 bits.
//    -> Esta é a função que muda mais quando portar para STM32 (usar HAL_I2C).
// ---------------------------------------------------------------------------
uint16_t mt6701_readAngleRaw() {
  Wire.beginTransmission(MT6701_I2C_ADDR);
  Wire.write(MT6701_REG_ANGLE_H);          // aponta para o registrador inicial
  Wire.endTransmission(false);             // repeated start (sem soltar o barramento)

  Wire.requestFrom(MT6701_I2C_ADDR, 2);    // pede 2 bytes: ANGLE_H e ANGLE_L
  if (Wire.available() < 2) {
    return 0xFFFF; // valor inválido -> sinaliza erro de leitura
  }

  uint8_t angleH = Wire.read();            // 8 bits mais significativos
  uint8_t angleL = Wire.read();            // 6 bits menos significativos (nos bits 7:2)

  // Monta o valor de 14 bits: (angleH << 6) | (angleL >> 2)
  uint16_t raw = ((uint16_t)angleH << 6) | (angleL >> 2);
  return raw; // 0 a 16383
}

// ---------------------------------------------------------------------------
// 2) CAMADA DE CONVERSÃO FÍSICA
//    Converte a contagem bruta (0..16383) em radianos (0..2π)
// ---------------------------------------------------------------------------
float mt6701_rawToRadians(uint16_t raw) {
  return (raw / MT6701_RESOLUTION) * 2.0f * PI;
}

// ---------------------------------------------------------------------------
// 3) CAMADA DE CÁLCULO DE VELOCIDADE E SENTIDO
//    Trata o "wrap-around" (quando passa de 16383 para 0, ou de 0 para 16383)
//    e calcula RPM + sentido de rotação (CW ou CCW)
// ---------------------------------------------------------------------------
// Em vez de retornar uma struct (isso confunde o gerador automático de
// prototypes da IDE do Arduino, causando erro de compilação), passamos os
// resultados por referência. Funciona igual e compila sem problemas.
void mt6701_update(float &angleRad, float &angularVelocity, int8_t &direction) {
  angleRad = 0;
  angularVelocity = 0;
  direction = 0;

  uint16_t rawNow = mt6701_readAngleRaw();
  if (rawNow == 0xFFFF) {
    // Falha de leitura I2C: retorna último ângulo válido conhecido
    angleRad = mt6701_rawToRadians(lastRawCount);
    return;
  }

  uint32_t timeNow = micros();
  angleRad = mt6701_rawToRadians(rawNow);

  if (firstReading) {
    // Na primeira leitura só inicializamos as referências
    lastRawCount   = rawNow;
    lastTimeMicros = timeNow;
    firstReading   = false;
    return;
  }

  // ---- Trata o delta de contagem, incluindo wrap-around da volta 0<->16383 ----
  int32_t delta = (int32_t)rawNow - (int32_t)lastRawCount;

  const int32_t HALF_SCALE = (int32_t)(MT6701_RESOLUTION / 2); // 8192

  if (delta > HALF_SCALE) {
    // Passou de perto do 0 para perto do 16383 -> na verdade girou p/ trás (CCW)
    delta -= (int32_t)MT6701_RESOLUTION;
  } else if (delta < -HALF_SCALE) {
    // Passou de perto do 16383 para perto do 0 -> na verdade girou p/ frente (CW)
    delta += (int32_t)MT6701_RESOLUTION;
  }

  // ---- Calcula tempo decorrido em segundos ----
  uint32_t dtMicros = timeNow - lastTimeMicros; // funciona mesmo com overflow do micros()
  float dtSeconds = dtMicros / 1000000.0f;

  if (dtSeconds > 0.0f) {
    // Fração de volta percorrida nesse intervalo
    float deltaAngleRad = (delta / MT6701_RESOLUTION) * 2.0f * PI;

    float angularVelocityInstant = deltaAngleRad / dtSeconds;

    // ---- Filtro de média móvel para suavizar leitura ----
    velocityBuffer[velocityBufferIndex] = angularVelocityInstant;
    velocityBufferIndex = (velocityBufferIndex + 1) % VELOCITY_FILTER_SAMPLES;

    float velocitySum = 0;
    for (uint8_t i = 0; i < VELOCITY_FILTER_SAMPLES; i++) {
      velocitySum += velocityBuffer[i];
    }

angularVelocity = velocitySum / VELOCITY_FILTER_SAMPLES;

    // ---- Sentido de rotação ----
    // Convenção: delta positivo = valores crescentes = sentido horário (CW)
    //            delta negativo = valores decrescentes = anti-horário (CCW)
    // (Se notar que está invertido na prática, é só inverter o sinal aqui,
    //  pois depende de como o ímã/encoder estão fisicamente montados.)
    if (delta > 0) {
      direction = 1;   // CW
    } else if (delta < 0) {
      direction = -1;  // CCW
    } else {
      direction = 0;   // parado
    }
  }

  lastRawCount   = rawNow;
  lastTimeMicros = timeNow;
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

  Serial.println("=================================================");
  Serial.println(" MT6701 - Leitura de RPM e sentido de rotacao");
  Serial.println("=================================================");

  // Teste inicial de comunicação
  uint16_t testRead = mt6701_readAngleRaw();
  if (testRead == 0xFFFF) {
    Serial.println("ERRO: nao foi possivel comunicar com o MT6701.");
    Serial.println("Verifique fiacao (SDA/SCL/VCC/GND) e endereco I2C (0x06).");
  } else {
    Serial.print("Sensor detectado. Leitura inicial (raw): ");
    Serial.println(testRead);
  }
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // atualiza serial a cada 100ms

void loop() {
  float angleRad = 0;
  float angularVelocity = 0;
  int8_t direction = 0;

  mt6701_update(angleRad, angularVelocity, direction);

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();

    const char* dirStr = (direction == 1) ? "CW " :
                          (direction == -1) ? "CCW" : " - ";

    Serial.print("Angulo: ");
    Serial.print(angleRad, 4);
    Serial.print(" rad | Velocidade: ");
    Serial.print(angularVelocity, 4);
    Serial.print(" rad/s | Sentido: ");
    Serial.println(dirStr);
  }

  // Pequeno delay para não sobrecarregar o barramento I2C
  // (pode reduzir se quiser mais resolução temporal)
  delayMicroseconds(500);
}
