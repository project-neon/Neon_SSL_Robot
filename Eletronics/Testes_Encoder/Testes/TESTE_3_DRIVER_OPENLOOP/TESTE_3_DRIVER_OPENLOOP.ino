#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 14);

Commander command = Commander(Serial);

void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Driver inicialmente desabilitado
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

  driver.voltage_power_supply = 16.8;
  driver.voltage_limit = 12.0;

  if (!driver.init()) {
    Serial.println("ERRO: driver.init()");
    return;
  }

  motor.linkDriver(&driver);

  // Primeiro teste físico: tensão bastante baixa
  motor.voltage_limit = 3.0;

  motor.controller = MotionControlType::velocity_openloop;

  if (!motor.init()) {
    Serial.println("ERRO: motor.init()");
    return;
  }

  motor.target = 0.0;

  Serial.println("Motor iniciado");


  // Commander
  command.add('T', doTarget, "Velocidade alvo [rad/s]");

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" SimpleFOC - Teste Motor + DRV8313");
  Serial.println(" Modo: Velocity Open-Loop");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("T1   -> 1 rad/s  (~9.5 RPM)");
  Serial.println("T2   -> 2 rad/s  (~19.1 RPM)");
  Serial.println("T5   -> 5 rad/s  (~47.7 RPM)");
  Serial.println("T-2  -> -2 rad/s");
  Serial.println("T0   -> parar");
  Serial.println();
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
}