// ================================================================
// CUBLI TÉTRAÈDRE - Équilibre sur sommet
// ESP32 + MPU9250 (I2C) + ESC bidirectionnel
// ================================================================
// PINS À CONFIGURER :
#define PIN_SDA     21      // ← Modifier selon votre câblage
#define PIN_SCL     22      // ← Modifier selon votre câblage
#define PIN_ESC     18      // ← Modifier selon votre câblage
// ================================================================

#include <Wire.h>
#include <ESP32Servo.h>

// ----------------------------------------------------------------
// MPU9250 - Registres I2C
// ----------------------------------------------------------------
#define MPU9250_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_CONFIG          0x1A
#define REG_SMPLRT_DIV      0x19

// ----------------------------------------------------------------
// Configuration ESC
// ----------------------------------------------------------------
#define ESC_MIN_US      1000    // Stop (ou marche arrière max en bidi)
#define ESC_MID_US      1500    // Point neutre bidirectionnel
#define ESC_MAX_US      2000    // Plein avant
#define ESC_ARM_DELAY   3000    // ms d'armement

// ----------------------------------------------------------------
// PID - Paramètres (À TUNER selon votre système)
// ----------------------------------------------------------------
#define PID_KP          8.0f    // Gain proportionnel
#define PID_KI          0.5f    // Gain intégral
#define PID_KD          2.5f    // Gain dérivé

#define ANGLE_SETPOINT  0.0f    // Angle cible en degrés (vertical)
#define ANGLE_DEADZONE  0.3f    // ±° de tolérance sans correction
#define ANGLE_FALLOVER  35.0f   // ±° au-delà = abandon (trop penché)
#define INTEGRAL_LIMIT  200.0f  // Anti-windup

#define LOOP_HZ         200     // Fréquence de boucle (200 Hz)
#define LOOP_US         (1000000 / LOOP_HZ)

// ----------------------------------------------------------------
// Filtre complémentaire
// ----------------------------------------------------------------
#define COMP_ALPHA      0.98f   // 0.98 = favorise gyro, 0.02 = accel

// ================================================================
// Variables globales
// ================================================================
Servo esc;

// IMU raw
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

// IMU calibration offsets (calculés au démarrage)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Angle filtré
float angle     = 0.0f;    // Angle actuel (degrés)
float angle_accel = 0.0f;  // Angle depuis accéléromètre

// PID
float pid_error_prev  = 0.0f;
float pid_integral    = 0.0f;
float pid_output      = 0.0f;

// Timing
unsigned long last_loop_us = 0;
float dt = 0.005f;  // secondes (initialisé à 1/200Hz)

// Etat
bool is_balanced = false;
bool is_fallen   = false;

// ================================================================
// MPU9250 - Fonctions I2C bas niveau
// ================================================================
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t mpu_read(uint8_t reg) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 1);
    return Wire.read();
}

void mpu_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

// ================================================================
// MPU9250 - Init
// ================================================================
void mpu_init() {
    // Réveil
    mpu_write(REG_PWR_MGMT_1, 0x00);
    delay(100);

    // Horloge PLL gyro X
    mpu_write(REG_PWR_MGMT_1, 0x01);

    // Sample rate : 1kHz / (1 + 4) = 200 Hz
    mpu_write(REG_SMPLRT_DIV, 0x04);

    // DLPF à 42 Hz (filtre passe-bas numérique)
    mpu_write(REG_CONFIG, 0x03);

    // Gyro : ±500°/s → sensibilité 65.5 LSB/°/s
    mpu_write(REG_GYRO_CONFIG, 0x08);

    // Accel : ±2g → sensibilité 16384 LSB/g
    mpu_write(REG_ACCEL_CONFIG, 0x00);

    Serial.println("[MPU9250] Initialisé");
}

// ================================================================
// MPU9250 - Lecture
// ================================================================
void mpu_read_all() {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);

    ax_raw = (int16_t)(buf[0]  << 8 | buf[1]);
    ay_raw = (int16_t)(buf[2]  << 8 | buf[3]);
    az_raw = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6..7] = température (ignorée)
    gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
    gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
    gz_raw = (int16_t)(buf[12] << 8 | buf[13]);
}

// ================================================================
// Calibration - 500 échantillons à l'arrêt
// ================================================================
void mpu_calibrate() {
    Serial.println("[CAL] Calibration en cours - NE PAS BOUGER...");
    
    const int N = 500;
    long sum_ax=0, sum_ay=0, sum_az=0;
    long sum_gx=0, sum_gy=0, sum_gz=0;

    for (int i = 0; i < N; i++) {
        mpu_read_all();
        sum_ax += ax_raw; sum_ay += ay_raw; sum_az += az_raw;
        sum_gx += gx_raw; sum_gy += gy_raw; sum_gz += gz_raw;
        delay(5);
    }

    ax_offset = sum_ax / (float)N;
    ay_offset = sum_ay / (float)N;
    // az_offset : on retire 1g (16384 LSB) pour la gravité
    az_offset = sum_az / (float)N - 16384.0f;
    gx_offset = sum_gx / (float)N;
    gy_offset = sum_gy / (float)N;
    gz_offset = sum_gz / (float)N;

    Serial.printf("[CAL] Offsets accel: %.1f / %.1f / %.1f\n",
                  ax_offset, ay_offset, az_offset);
    Serial.printf("[CAL] Offsets gyro:  %.1f / %.1f / %.1f\n",
                  gx_offset, gy_offset, gz_offset);
    Serial.println("[CAL] Calibration terminée !");
}

// ================================================================
// Filtre complémentaire → angle en degrés
// ================================================================
void update_angle() {
    mpu_read_all();

    // Accel corrigé (en g)
    float ax = (ax_raw - ax_offset) / 16384.0f;
    float ay = (ay_raw - ay_offset) / 16384.0f;
    float az = (az_raw - az_offset) / 16384.0f;

    // Gyro corrigé (en °/s)
    // ±500°/s → 65.5 LSB/°/s
    float gy = (gy_raw - gy_offset) / 65.5f;

    // Angle depuis accéléromètre (axe de basculement = Y ici)
    // ⚠️ À adapter selon l'orientation physique du capteur
    angle_accel = atan2f(ax, sqrtf(ay*ay + az*az)) * (180.0f / M_PI);

    // Filtre complémentaire
    angle = COMP_ALPHA * (angle + gy * dt)
          + (1.0f - COMP_ALPHA) * angle_accel;
}

// ================================================================
// PID
// ================================================================
float compute_pid(float setpoint, float measured) {
    float error = setpoint - measured;

    // Deadzone
    if (fabsf(error) < ANGLE_DEADZONE) {
        pid_integral    = 0.0f;
        pid_error_prev  = 0.0f;
        return 0.0f;
    }

    // Intégrale avec anti-windup
    pid_integral += error * dt;
    pid_integral  = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // Dérivée
    float derivative = (error - pid_error_prev) / dt;
    pid_error_prev   = error;

    return (PID_KP * error) + (PID_KI * pid_integral) + (PID_KD * derivative);
}

// ================================================================
// ESC - Conversion output PID → µs PWM
// ================================================================
void set_motor(float pid_out) {
    // pid_out est typiquement dans [-500, +500]
    // On mappe vers [1000, 2000] µs (ESC bidirectionnel : 1500 = neutre)
    int pwm_us = ESC_MID_US + (int)pid_out;
    pwm_us = constrain(pwm_us, ESC_MIN_US, ESC_MAX_US);
    esc.writeMicroseconds(pwm_us);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== CUBLI TÉTRAÈDRE - Démarrage ===");

    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);  // 400 kHz Fast Mode

    // MPU9250
    mpu_init();
    mpu_calibrate();

    // ESC - Armement
    Serial.println("[ESC] Armement...");
    esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
    esc.writeMicroseconds(ESC_MID_US);  // Neutre bidi
    delay(ESC_ARM_DELAY);
    Serial.println("[ESC] Armé !");

    // Init angle
    for (int i = 0; i < 100; i++) {
        update_angle();
        delay(5);
    }

    last_loop_us = micros();
    Serial.println("[OK] Boucle de contrôle démarrée");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // Timing précis
    unsigned long now = micros();
    dt = (now - last_loop_us) / 1e6f;
    if (dt < 1.0f / LOOP_HZ) return;  // Attendre prochain cycle
    last_loop_us = now;

    // 1. Lire capteur + mettre à jour angle
    update_angle();

    // 2. Vérifier si trop penché → abandon
    if (fabsf(angle) > ANGLE_FALLOVER) {
        if (!is_fallen) {
            Serial.println("[!] Chute détectée - Moteur arrêté");
            is_fallen = true;
        }
        esc.writeMicroseconds(ESC_MID_US);
        return;
    }
    is_fallen = false;

    // 3. Calcul PID
    pid_output = compute_pid(ANGLE_SETPOINT, angle);

    // 4. Commande moteur
    set_motor(pid_output);

    // 5. Debug série (toutes les 50ms)
    static unsigned long last_print = 0;
    if (millis() - last_print > 50) {
        last_print = millis();
        Serial.printf("Angle: %6.2f° | PID: %7.2f | PWM: %d µs\n",
                      angle,
                      pid_output,
                      ESC_MID_US + (int)constrain(pid_output, -500, 500));
    }
}
