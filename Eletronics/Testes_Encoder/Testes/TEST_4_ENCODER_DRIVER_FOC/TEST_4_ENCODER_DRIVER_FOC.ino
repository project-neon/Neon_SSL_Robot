#include <Wire.h>
#include <SimpleFOC.h>

// ===========================================================================
// SENSOR MT6701
// ===========================================================================
// I2C:
// SDA -> GPIO 21
// SCL -> GPIO 22
//
// Endereço: 0x06
// Resolução: 14 bits
// Registrador do ângulo: 0x03
// ===========================================================================

MagneticSensorI2C sensor = MagneticSensorI2C(0x06, 14, 0x03, 8);


// ===========================================================================
// MOTOR
// ===========================================================================
// GBM2804H-100T
// 12N14P
// 14 polos
// 7 pares de polos
// ===========================================================================

BLDCMotor motor = BLDCMotor(7);


// ===========================================================================
// DRIVER DRV8313
// ===========================================================================
// Fase U -> GPIO 25
// Fase V -> GPIO 26
// Fase W -> GPIO 27
// ENABLE -> GPIO 14
// ===========================================================================

BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 14);


// ===========================================================================
// COMMANDER
// ===========================================================================

Commander command = Commander(Serial);


// Comando de velocidade
void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}


// ===========================================================================
// CONFIGURAÇÃO
// ===========================================================================

const float SUPPLY_VOLTAGE = 16.8f;

// Limite máximo configurado na camada do driver
const float DRIVER_VOLTAGE_LIMIT = 12.0f;

// Limite utilizado pelo FOC neste primeiro teste
const float MOTOR_VOLTAGE_LIMIT = 3.0f;


// ===========================================================================
// SETUP
// ===========================================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================================");
  Serial.println(" ESP32 + MT6701 + DRV8313 + SimpleFOC");
  Serial.println(" Teste FOC - V1");
  Serial.println("=================================================");
  Serial.println();


  // -------------------------------------------------------------------------
  // ENABLE DO DRV8313
  // -------------------------------------------------------------------------
  // Mantém o driver desabilitado durante a inicialização.
  // -------------------------------------------------------------------------

  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

  Serial.println("DRV8313: ENABLE = LOW");


  // -------------------------------------------------------------------------
  // I2C
  // -------------------------------------------------------------------------

  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println("I2C: SDA = GPIO 21");
  Serial.println("I2C: SCL = GPIO 22");
  Serial.println("I2C: 400 kHz");


  // -------------------------------------------------------------------------
  // SENSOR
  // -------------------------------------------------------------------------

  Serial.println();
  Serial.println("Inicializando MT6701...");

  sensor.init();

  Serial.println("MT6701 inicializado");


  // -------------------------------------------------------------------------
  // DRIVER
  // -------------------------------------------------------------------------

  Serial.println();
  Serial.println("Inicializando DRV8313...");

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;

  if (!driver.init()) {

    Serial.println("ERRO: driver.init() falhou");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("DRV8313 inicializado");


  // -------------------------------------------------------------------------
  // ASSOCIAR DRIVER AO MOTOR
  // -------------------------------------------------------------------------

  motor.linkDriver(&driver);


  // -------------------------------------------------------------------------
  // ASSOCIAR SENSOR AO MOTOR
  // -------------------------------------------------------------------------

  motor.linkSensor(&sensor);


  // -------------------------------------------------------------------------
  // LIMITE DE TENSÃO DO MOTOR
  // -------------------------------------------------------------------------

  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;


  // -------------------------------------------------------------------------
  // CONTROLE
  // -------------------------------------------------------------------------

  motor.controller = MotionControlType::velocity;


  // -------------------------------------------------------------------------
  // INICIALIZAÇÃO DO MOTOR
  // -------------------------------------------------------------------------

  Serial.println();
  Serial.println("Inicializando motor...");

  if (!motor.init()) {

    Serial.println("ERRO: motor.init() falhou");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("motor.init() OK");


  // -------------------------------------------------------------------------
  // INICIALIZAÇÃO DO FOC
  // -------------------------------------------------------------------------

  Serial.println();
  Serial.println("Inicializando FOC...");
  Serial.println("O motor podera se mover durante o alinhamento.");
  Serial.println();

  if (!motor.initFOC()) {

    Serial.println("ERRO: motor.initFOC() falhou");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("FOC inicializado com sucesso");


  // -------------------------------------------------------------------------
  // MOTOR PARADO INICIALMENTE
  // -------------------------------------------------------------------------

  motor.target = 0.0f;


  // -------------------------------------------------------------------------
  // COMMANDER
  // -------------------------------------------------------------------------

  command.add('T', doTarget, "Velocidade alvo [rad/s]");


  // -------------------------------------------------------------------------
  // HABILITA DRIVER
  // -------------------------------------------------------------------------

  digitalWrite(14, HIGH);

  Serial.println("DRV8313: ENABLE = HIGH");


  // -------------------------------------------------------------------------
  // INFORMAÇÕES
  // -------------------------------------------------------------------------

  Serial.println();
  Serial.println("=================================================");
  Serial.println(" SISTEMA PRONTO");
  Serial.println("=================================================");
  Serial.println();

  Serial.println("Limites:");
  Serial.print("  Fonte:             ");
  Serial.print(SUPPLY_VOLTAGE);
  Serial.println(" V");

  Serial.print("  Driver:            ");
  Serial.print(DRIVER_VOLTAGE_LIMIT);
  Serial.println(" V");

  Serial.print("  Motor/FOC:         ");
  Serial.print(MOTOR_VOLTAGE_LIMIT);
  Serial.println(" V");

  Serial.println();

  Serial.println("Comandos:");
  Serial.println("  T1   -> +1 rad/s");
  Serial.println("  T2   -> +2 rad/s");
  Serial.println("  T5   -> +5 rad/s");
  Serial.println("  T-1  -> -1 rad/s");
  Serial.println("  T-2  -> -2 rad/s");
  Serial.println("  T0   -> parar");

  Serial.println();
  Serial.println("Aguardando comando...");
}


// ===========================================================================
// LOOP
// ===========================================================================

void loop() {

  // -------------------------------------------------------------------------
  // FOC
  // -------------------------------------------------------------------------
  // Deve ser executado continuamente.
  // -------------------------------------------------------------------------

  motor.loopFOC();


  // -------------------------------------------------------------------------
  // CONTROLE DE MOVIMENTO
  // -------------------------------------------------------------------------

  motor.move(5.0);


  // -------------------------------------------------------------------------
  // COMMANDER
  // -------------------------------------------------------------------------

//  command.run();
}