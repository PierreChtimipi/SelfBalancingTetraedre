#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

// ===== CONFIG =====
Adafruit_BNO055 bno(55, 0x28);
Servo esc;

const int ESC_PIN    = 18;
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

// Contrôle PD
float Kp = 12.0;       // proportionnel
float Kd = 5.0;       // dérivé (amortissement gyro)
float maxU = 300.0;    // correction max (µs)

// Boucle 200 Hz
const unsigned long LOOP_US = 5000;
unsigned long lastLoop = 0;

// Filtre passe-bas anti-vibrations
float filtPitch = 0.0;
const float ALPHA = 0.85;  // 0.7=lisse, 0.95=réactif

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // I2C rapide

  if (!bno.begin()) {
    Serial.println("BNO055 introuvable!");
    while (1) delay(100);
  }
  bno.setExtCrystalUse(true);
  delay(50);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMU);

  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_MIN_US);
  delay(3000);  // armement ESC

  lastLoop = micros();
}

// ===== LOOP 200 Hz =====
void loop() {
  if (micros() - lastLoop < LOOP_US) return;
  lastLoop = micros();

  // Lecture orientation + gyro
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float rawPitch = euler.z();
  if (isnan(rawPitch)) { esc.writeMicroseconds(ESC_MIN_US); return; }

  // Filtre passe-bas
  filtPitch = ALPHA * rawPitch + (1.0f - ALPHA) * filtPitch;

  // PD : correction = position + amortissement
  float u = Kp * (0.0f - filtPitch) - Kd * gyro.z();
  u = constrain(u, -maxU, maxU);

  int cmd = constrain((int)(ESC_MIN_US + u), ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(cmd);

  Serial.printf("P:%.1f R:%.1f C:%d\n", filtPitch, gyro.z(), cmd);
}
