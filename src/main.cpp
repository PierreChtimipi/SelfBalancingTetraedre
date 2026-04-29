#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// =========================================================
// Configuration Matérielle
// =========================================================
#define PIN_SDA 21
#define PIN_SCL 22
const int ESC_PIN = 18;
#define MPU_ADDR 0x68

Servo esc;

// =========================================================
// Paramètres
// =========================================================
const float Kp = 0.74f;
const float Kd = 0.06f;
const float Kw = 0.005f;
const float Kt = 0.0095f;   
const float IQ_MAX = 12.0f; 

const unsigned long DT_EST_US = 5000;  // 200 Hz
const unsigned long DT_CTL_US = 10000; // 100 Hz

// =========================================================
// Variables Globales
// =========================================================
float theta_est = 0.0f;
float theta_dot_est = 0.0f;
float theta = 0.0f;
float theta_dot = 0.0f;
float omega_w = 0.0f;

float vertical_zero_offset_rad = 0.0f; 
float gx_offset = 0, gy_offset = 0, gz_offset = 0; // On ne calibre que le Gyro pour éviter les bugs de gravité

unsigned long last_est_us = 0;
unsigned long last_ctl_us = 0;

enum RobotState {
    STATE_LYING_FLAT, 
    STATE_SPIN_UP,    
    STATE_IMPULSE,    
    STATE_BALANCING   
};

RobotState currentState = STATE_LYING_FLAT;
unsigned long state_timer = 0;

// =========================================================
// Outils
// =========================================================
float clamp(float value, float vmin, float vmax) {
    if (value < vmin) return vmin;
    if (value > vmax) return vmax;
    return value;
}

// =========================================================
// MPU-6500 : Lecture I2C
// =========================================================
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void mpu_read_accel_gyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    
    int16_t raw_ax = Wire.read()<<8 | Wire.read();
    int16_t raw_ay = Wire.read()<<8 | Wire.read();
    int16_t raw_az = Wire.read()<<8 | Wire.read();
    Wire.read(); Wire.read(); // Température ignorée
    int16_t raw_gx = Wire.read()<<8 | Wire.read();
    int16_t raw_gy = Wire.read()<<8 | Wire.read();
    int16_t raw_gz = Wire.read()<<8 | Wire.read();

    // Conversion en g (sans offset pour l'accéléromètre pour préserver la vraie norme)
    ax = raw_ax / 16384.0f;
    ay = raw_ay / 16384.0f;
    az = raw_az / 16384.0f;
    
    // Gyro avec offsets
    gx = ((raw_gx - gx_offset) / 65.5f) * (M_PI / 180.0f);
    gy = ((raw_gy - gy_offset) / 65.5f) * (M_PI / 180.0f);
    gz = ((raw_gz - gz_offset) / 65.5f) * (M_PI / 180.0f);
}

void init_angle_from_acc(float ax, float ay, float az) {
    theta_est = atan2f(ax, ay); // Axe Y supposé vertical
}

void estimate_angle(float ax, float ay, float az, float gyro_z_rad_s, float dt_s, float alpha=0.96f) {
    float theta_gyro = theta_est + gyro_z_rad_s * dt_s;
    float theta_acc = atan2f(ax, ay);
    
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    // Tolérance large pour s'assurer que le filtre corrige bien la dérive
    if (norm > 0.5f && norm < 1.5f) {
        theta_est = alpha * theta_gyro + (1.0f - alpha) * theta_acc;
    } else {
        theta_est = theta_gyro;
    }
    theta_dot_est = gyro_z_rad_s;
}

// =========================================================
// Encodeur Virtuel
// =========================================================
float estimated_omega_w = 0.0f; 
void update_virtual_encoder(float commanded_torque, float dt) {
    estimated_omega_w += (commanded_torque / 0.001f) * dt;
    estimated_omega_w *= 0.99f;
    estimated_omega_w = clamp(estimated_omega_w, -500.0f, 500.0f);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    mpu_write(0x6B, 0x01); 
    mpu_write(0x1B, 0x08); 
    mpu_write(0x1C, 0x00); 
    mpu_write(0x1A, 0x03); 

    ESP32PWM::allocateTimer(0);
    esc.setPeriodHertz(250); 
    esc.attach(ESC_PIN, 1000, 2000);
    esc.writeMicroseconds(1000); // ARMEMEMT A 0 GAZ
    
    Serial.println("ÉTAPE 1 : NE BOUGEZ PAS (Posé à plat ou debout) ! Calibration du Gyro...");
    long sgx=0, sgy=0, sgz=0;
    for (int i = 0; i < 200; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43); // Registre direct du gyro
        Wire.endTransmission(false);
        Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
        sgx += (int16_t)(Wire.read()<<8 | Wire.read());
        sgy += (int16_t)(Wire.read()<<8 | Wire.read());
        sgz += (int16_t)(Wire.read()<<8 | Wire.read());
        delay(5);
    }
    gx_offset = sgx/200.0f; gy_offset = sgy/200.0f; gz_offset = sgz/200.0f;

    Serial.println("\nÉTAPE 2 : CALIBRATION DU ZERO VERTICAL !");
    Serial.println("-> MAINTENEZ LE ROBOT DEBOUT EN PARFAIT ÉQUILIBRE PENDANT 2 SECONDES...");
    delay(2000);
    
    float sum_angle = 0;
    for(int i=0; i<50; i++) {
        float ax, ay, az, gx, gy, gz;
        mpu_read_accel_gyro(ax, ay, az, gx, gy, gz);
        sum_angle += atan2f(ax, ay);
        delay(10);
    }
    vertical_zero_offset_rad = sum_angle / 50.0f;
    
    float ax, ay, az, gx, gy, gz;
    mpu_read_accel_gyro(ax, ay, az, gx, gy, gz);
    init_angle_from_acc(ax, ay, az);

    Serial.println("\n✅ PRÊT ! Posez le robot sur la table. L'ESC a dû biper son armement.");
    last_est_us = micros();
    last_ctl_us = last_est_us;
}

// =========================================================
// LOOP Principale
// =========================================================
void loop() {
    unsigned long now_us = micros();
    int current_pwm = 1000; 

    // --- Boucle IMU (200 Hz) ---
    if ((now_us - last_est_us) >= DT_EST_US) {
        float dt_est = (now_us - last_est_us) / 1000000.0f;
        last_est_us = now_us;

        float ax, ay, az, gx, gy, gz;
        mpu_read_accel_gyro(ax, ay, az, gx, gy, gz);
        estimate_angle(ax, ay, az, gz, dt_est, 0.96f);
        
        theta = theta_est - vertical_zero_offset_rad;
        theta_dot = theta_dot_est;
    }

    // --- Boucle Commande (100 Hz) ---
    if ((now_us - last_ctl_us) >= DT_CTL_US) {
        float dt_ctl = (now_us - last_ctl_us) / 1000000.0f; 
        last_ctl_us = now_us;
        float angle_deg = theta * (180.0f / M_PI);

        switch (currentState) {
            
            case STATE_LYING_FLAT:
                current_pwm = 1000; // Arrêt
                // S'il est couché (angle > 30°)
                if (fabs(angle_deg) > 30.0f) { 
                    if (millis() - state_timer > 2000) {
                        currentState = STATE_SPIN_UP;
                        state_timer = millis();
                        Serial.println("⚡ SPIN-UP : Accélération progressive...");
                    }
                } else {
                    state_timer = millis(); 
                }
                break;

            case STATE_SPIN_UP:
                {
                    unsigned long elapsed = millis() - state_timer;
                    if (elapsed < 1500) {
                        // Accélération en douceur (Soft-Start) pour ne pas bloquer l'ESC
                        // On passe de 1000 à 1800 sur 1.5 secondes
                        current_pwm = 1000 + (int)((elapsed * 800) / 1500);
                    } else {
                        currentState = STATE_IMPULSE;
                        state_timer = millis();
                        Serial.println("💥 KICK ! Freinage brutal !");
                    }
                }
                break;

            case STATE_IMPULSE:
                current_pwm = 1000; // Coupure des gaz

                // On laisse l'inertie lever le robot. S'il se redresse un peu, on passe en équilibre.
                if (millis() - state_timer > 200 || fabs(angle_deg) < 25.0f) {
                    currentState = STATE_BALANCING;
                    Serial.println("🎯 TENTATIVE D'ÉQUILIBRE !");
                }
                break;

            case STATE_BALANCING:
                if (fabs(angle_deg) > 40.0f) {
                    currentState = STATE_LYING_FLAT;
                    state_timer = millis();
                    Serial.println("❌ CRASH : Retour au sol.");
                    break;
                }

                float tau_cmd = -Kp * theta - Kd * theta_dot - Kw * estimated_omega_w;
                update_virtual_encoder(tau_cmd, dt_ctl);

                float iq_cmd = clamp(tau_cmd / Kt, -IQ_MAX, IQ_MAX);
                // Le moteur ne tourne qu'en marche avant (1000 à 2000)
                current_pwm = 1000 + (int)((iq_cmd / IQ_MAX) * 1000.0f);
                current_pwm = clamp(current_pwm, 1000, 2000); 
                break;
        }

        esc.writeMicroseconds(current_pwm);
    }
    
    // --- Affichage (10 Hz) ---
    static unsigned long last_print = 0;
    if (millis() - last_print > 100) {
        last_print = millis();
        float angle_deg = theta * (180.0f / M_PI);
        String etat_str = (currentState == STATE_LYING_FLAT) ? "COUCHÉ" : 
                          (currentState == STATE_SPIN_UP) ? "ÉLAN" : 
                          (currentState == STATE_IMPULSE) ? "KICK" : "ÉQUILIBRE";
        Serial.printf("[%s] Angle: %+6.1f° | PWM: %4d\n", etat_str.c_str(), angle_deg, esc.readMicroseconds());
    }
}