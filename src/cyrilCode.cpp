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
// Paramètres physiques et commande
// =========================================================
const float Kp = 0.74f;
const float Kd = 0.06f;
const float Kw = 0.005f;

const float Kt = 0.0095f;   // N.m/A, moteur 1000KV approx
const float IQ_MAX = 12.0f; // Limite en Ampères

// Fréquences en microsecondes
const unsigned long DT_EST_US = 5000;  // 200 Hz
const unsigned long DT_CTL_US = 10000; // 100 Hz

// =========================================================
// État estimateur et capteur
// =========================================================
float theta_est = 0.0f;
float theta_dot_est = 0.0f;

float theta = 0.0f;
float theta_dot = 0.0f;
float omega_w = 0.0f;

// Offsets MPU
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Timers
unsigned long last_est_us = 0;
unsigned long last_ctl_us = 0;

// =========================================================
// Outils
// =========================================================
float clamp(float value, float vmin, float vmax) {
    if (value < vmin) return vmin;
    if (value > vmax) return vmax;
    return value;
}

// =========================================================
// MPU-9250 : Lecture I2C
// =========================================================
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void mpu_read_accel_gyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    uint8_t buf[14];
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    for (uint8_t i = 0; i < 14; i++) buf[i] = Wire.read();

    int16_t ax_raw = (buf[0] << 8 | buf[1]);
    int16_t ay_raw = (buf[2] << 8 | buf[3]);
    int16_t az_raw = (buf[4] << 8 | buf[5]);
    int16_t gx_raw = (buf[8] << 8 | buf[9]);
    int16_t gy_raw = (buf[10] << 8 | buf[11]);
    int16_t gz_raw = (buf[12] << 8 | buf[13]);

    ax = (ax_raw - ax_offset) / 16384.0f;
    ay = (ay_raw - ay_offset) / 16384.0f;
    az = (az_raw - az_offset) / 16384.0f;
    
    // En °/s puis conversion en rad/s
    gx = ((gx_raw - gx_offset) / 65.5f) * (M_PI / 180.0f);
    gy = ((gy_raw - gy_offset) / 65.5f) * (M_PI / 180.0f);
    gz = ((gz_raw - gz_offset) / 65.5f) * (M_PI / 180.0f);
}

// =========================================================
// Estimation de l'angle
// =========================================================
void init_angle_from_acc(float ax, float ay, float az) {
    theta_est = atan2f(ax, az);
}

void estimate_angle(float ax, float ay, float az, float gyro_y_rad_s, float dt_s, float alpha=0.98f, float g_ref=1.0f, float g_tol=0.15f) {
    // Prédiction gyro
    float theta_gyro = theta_est + gyro_y_rad_s * dt_s;

    // Angle accéléro
    float theta_acc = atan2f(ax, az);

    // Norme accéléro
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    // Fusion si l'accélération ne subit pas trop de chocs (entre 0.85g et 1.15g)
    if (norm >= (g_ref - g_tol) && norm <= (g_ref + g_tol)) {
        theta_est = alpha * theta_gyro + (1.0f - alpha) * theta_acc;
    } else {
        theta_est = theta_gyro;
    }

    theta_dot_est = gyro_y_rad_s;
}

// =========================================================
// Encodeur Virtuel (Estimateur de vitesse)
// =========================================================

// On crée une variable globale pour mémoriser la vitesse de la roue
float estimated_omega_w = 0.0f; 

// Paramètres de l'estimateur
const float INERTIE_ROUE = 0.001f; // Constante d'inertie (à ajuster selon le poids de ta roue)
const float FRICTION_MOTEUR = 0.99f; // Le moteur ralentit un peu tout seul à cause des roulements

float get_encoder_speed() {
    // Au lieu d'un vrai capteur, on renvoie notre estimation mathématique
    return estimated_omega_w; 
}

// ---> À AJOUTER DANS TA BOUCLE DE COMMANDE (100Hz) <---
// Juste APRÈS avoir calculé "tau_cmd" et "iq_cmd", on met à jour notre encodeur virtuel :
void update_virtual_encoder(float commanded_torque, float dt) {
    // La physique : Accélération = Couple / Inertie
    float acceleration_theorique = commanded_torque / INERTIE_ROUE;
    
    // Vitesse = Ancienne vitesse + (Accélération * Temps)
    estimated_omega_w = estimated_omega_w + (acceleration_theorique * dt);
    
    // On applique une petite friction naturelle pour que la vitesse ne monte pas à l'infini dans le code
    estimated_omega_w = estimated_omega_w * FRICTION_MOTEUR;
    
    // On bride la vitesse virtuelle à une valeur réaliste (ex: 500 rad/s)
    estimated_omega_w = clamp(estimated_omega_w, -500.0f, 500.0f);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // Init MPU
    mpu_write(0x6B, 0x01); // PWR_MGMT_1
    mpu_write(0x1B, 0x08); // GYRO_CONFIG (500 dps)
    mpu_write(0x1C, 0x00); // ACCEL_CONFIG (2g)

    // Init ESC
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    esc.setPeriodHertz(50);
    esc.attach(ESC_PIN, 1000, 2000);
    esc.writeMicroseconds(1500); // 1500 = Arrêt pour un ESC bidirectionnel
    
    Serial.println("Calibration des offsets...");
    long sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
    for (int i = 0; i < 200; i++) {
        uint8_t buf[14];
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
        for (uint8_t j = 0; j < 14; j++) buf[j] = Wire.read();
        
        sax += (int16_t)(buf[0]<<8|buf[1]);
        say += (int16_t)(buf[2]<<8|buf[3]);
        saz += (int16_t)(buf[4]<<8|buf[5]);
        sgx += (int16_t)(buf[8]<<8|buf[9]);
        sgy += (int16_t)(buf[10]<<8|buf[11]);
        sgz += (int16_t)(buf[12]<<8|buf[13]);
        delay(5);
    }
    ax_offset = sax/200.0f; ay_offset = say/200.0f; az_offset = (saz/200.0f) - 16384.0f;
    gx_offset = sgx/200.0f; gy_offset = sgy/200.0f; gz_offset = sgz/200.0f;

    // Initialisation angle
    float ax, ay, az, gx, gy, gz;
    mpu_read_accel_gyro(ax, ay, az, gx, gy, gz);
    init_angle_from_acc(ax, ay, az);

    Serial.println("Prêt !");
    last_est_us = micros();
    last_ctl_us = last_est_us;
}

// =========================================================
// LOOP Principale
// =========================================================
void loop() {
    unsigned long now_us = micros();

    // -----------------------------------------------------
    // 1) Boucle estimation IMU à 200 Hz
    // -----------------------------------------------------
    if ((now_us - last_est_us) >= DT_EST_US) {
        float dt_est = (now_us - last_est_us) / 1000000.0f;
        last_est_us = now_us;

        float ax, ay, az, gx, gy, gz;
        mpu_read_accel_gyro(ax, ay, az, gx, gy, gz);

        // Attention : Vérifier quel axe du gyroscope correspond au tangage (gy_rad_s ici)
        estimate_angle(ax, ay, az, gy, dt_est, 0.98f);
        
        theta = theta_est;
        theta_dot = theta_dot_est;
    }

    // -----------------------------------------------------
    // 2) Boucle commande à 100 Hz
    // -----------------------------------------------------
    if ((now_us - last_ctl_us) >= DT_CTL_US) {
        last_ctl_us = now_us;

        // 1. On lit notre vitesse virtuelle
        omega_w = get_encoder_speed();

        // 2. Loi de commande (Maintenant Kw sert à quelque chose !)
        float tau_cmd = -Kp * theta - Kd * theta_dot - Kw * omega_w;

        // 3. On met à jour notre encodeur pour le prochain tour de boucle
        float dt_ctl = (now_us - last_ctl_us) / 1000000.0f; // Temps écoulé (~0.01s)
        update_virtual_encoder(tau_cmd, dt_ctl);
        // Loi de commande PD
        float tau_cmd = -Kp * theta - Kd * theta_dot - Kw * omega_w;

        // Conversion couple -> courant
        float iq_cmd = tau_cmd / Kt;
        iq_cmd = clamp(iq_cmd, -IQ_MAX, IQ_MAX);

        // Mapping Courant [-12A, 12A] -> PWM [1000, 2000] pour ESC Bidirectionnel
        int pwm_cmd = 1500; 

        // Sécurité anti-chute (0.6 rad = ~34 degrés)
        if (fabs(theta) > 0.6f) {
            pwm_cmd = 1500; // Force l'arrêt
        } else {
            // Mapping linéaire proportionnel
            pwm_cmd = 1500 + (int)((iq_cmd / IQ_MAX) * 500.0f);
            pwm_cmd = clamp(pwm_cmd, 1000, 2000);
        }

        // Envoi au moteur
        esc.writeMicroseconds(pwm_cmd);
    }
}