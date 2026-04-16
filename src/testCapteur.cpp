#include <Arduino.h> // OBLIGATOIRE sous PlatformIO
#include <Wire.h>
#include <math.h>

// ================================================================
// TEST MPU9250 - Lecture et affichage des données
// ESP32 + MPU9250 (I2C)
// ================================================================

// PINS À CONFIGURER (Validé pour ton câblage)
#define PIN_SDA     21
#define PIN_SCL     22

// ----------------------------------------------------------------
// MPU9250 - Registres
// ----------------------------------------------------------------
// Adresse I2C : 0x68 (Si AD0 est relié au GND) ou 0x69 (Si AD0 est au 3.3V)
#define MPU9250_ADDR        0x68 
#define REG_WHO_AM_I        0x75 // Registre d'identification
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_CONFIG          0x1A
#define REG_SMPLRT_DIV      0x19

// ----------------------------------------------------------------
// Variables
// ----------------------------------------------------------------
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

float ax_offset=0, ay_offset=0, az_offset=0;
float gx_offset=0, gy_offset=0, gz_offset=0;

float angle_x = 0.0f;  // Filtre complémentaire axe X
float angle_y = 0.0f;  // Filtre complémentaire axe Y

unsigned long last_us = 0;
float dt = 0.01f;

#define COMP_ALPHA  0.98f

// ================================================================
// MPU9250 - I2C
// ================================================================
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void mpu_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU9250_ADDR, (uint8_t)len, (uint8_t)true);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
}

void mpu_init() {
    // 1. Vérification de la présence du capteur (WHO AM I)
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU9250_ADDR, (uint8_t)1, (uint8_t)true);
    uint8_t whoami = Wire.read();
    
    // On accepte 0x71 (MPU9250), 0x73 (MPU9255) ET 0x70 (MPU6500)
    if (whoami != 0x71 && whoami != 0x73 && whoami != 0x70) {
        Serial.println("❌ ERREUR CRITIQUE : MPU9250 INTROUVABLE !");
        Serial.print("Code d'erreur WHO_AM_I reçu : 0x");
        Serial.println(whoami, HEX);
        Serial.println("-> Vérifiez vos câbles SDA/SCL et l'alimentation 3.3V.");
        while(1); // Bloque le programme ici pour éviter le crash
    }
    
    Serial.println("✅ Capteur MPU9250 détecté !");

    // 2. Configuration
    mpu_write(REG_PWR_MGMT_1, 0x00);   delay(100);
    mpu_write(REG_PWR_MGMT_1, 0x01);
    mpu_write(REG_SMPLRT_DIV, 0x04);   // 200 Hz
    mpu_write(REG_CONFIG,     0x03);   // DLPF 42 Hz
    mpu_write(REG_GYRO_CONFIG,  0x08); // ±500°/s
    mpu_write(REG_ACCEL_CONFIG, 0x00); // ±2g
    Serial.println("✅ MPU9250 Initialisé avec succès.");
}

void mpu_read_all() {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);
    ax_raw = (int16_t)(buf[0]  << 8 | buf[1]);
    ay_raw = (int16_t)(buf[2]  << 8 | buf[3]);
    az_raw = (int16_t)(buf[4]  << 8 | buf[5]);
    gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
    gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
    gz_raw = (int16_t)(buf[12] << 8 | buf[13]);
}

// ================================================================
// Calibration
// ================================================================
void mpu_calibrate() {
    Serial.println("\n[CAL] Posez le capteur à plat sur la table et NE BOUGEZ PLUS...");
    delay(3000);
    Serial.println("[CAL] Calibration en cours (Ne touchez pas la table)...");

    const int N = 500;
    long sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
    for (int i = 0; i < N; i++) {
        mpu_read_all();
        sax+=ax_raw; say+=ay_raw; saz+=az_raw;
        sgx+=gx_raw; sgy+=gy_raw; sgz+=gz_raw;
        delay(5);
    }
    ax_offset = sax/(float)N;
    ay_offset = say/(float)N;
    az_offset = saz/(float)N - 16384.0f; // Retire 1g sur l'axe Z (gravité)
    gx_offset = sgx/(float)N;
    gy_offset = sgy/(float)N;
    gz_offset = sgz/(float)N;

    Serial.println("[CAL] Terminée ! Vous pouvez prendre le capteur en main.");
    Serial.println("─────────────────────────────────────────────────");
    delay(2000);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Laisse le temps au port série de s'ouvrir
    Serial.println("\n=== TEST D'ÉQUILIBRE MPU9250 ===");

    // Initialisation I2C pour ESP32
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); // I2C en mode rapide (400kHz)

    mpu_init();
    mpu_calibrate();

    last_us = micros();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // Timing précis pour le filtre
    unsigned long now = micros();
    dt = (now - last_us) / 1000000.0f;
    if (dt < 0.005f) return; // Limite à 200 Hz max
    last_us = now;

    // Lecture brute
    mpu_read_all();

    // Conversion en valeurs physiques (g et °/s)
    float ax = (ax_raw - ax_offset) / 16384.0f; 
    float ay = (ay_raw - ay_offset) / 16384.0f;
    float az = (az_raw - az_offset) / 16384.0f;

    float gx = (gx_raw - gx_offset) / 65.5f;   
    float gy = (gy_raw - gy_offset) / 65.5f;
    float gz = (gz_raw - gz_offset) / 65.5f;

    // Angles bruts depuis l'accéléromètre (Trigonométrie)
    float accel_angle_x = atan2f(ay, sqrtf(ax*ax + az*az)) * (180.0f / M_PI);
    float accel_angle_y = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / M_PI);

    // Filtre complémentaire (Fusion Accéléromètre + Gyroscope)
    angle_x = COMP_ALPHA * (angle_x + gx * dt) + (1.0f - COMP_ALPHA) * accel_angle_x;
    angle_y = COMP_ALPHA * (angle_y + gy * dt) + (1.0f - COMP_ALPHA) * accel_angle_y;

    // Affichage propre dans la console (Toutes les 100ms)
    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();

        // Affichage adapté pour le traceur série de l'IDE
        Serial.printf("AngleX:%.2f AngleY:%.2f\n", angle_x, angle_y);
    }
}