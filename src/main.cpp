#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ================================================================
// CONFIGURATION MATÉRIELLE
// ================================================================
#define PIN_SDA 21
#define PIN_SCL 22
const int ESC_PIN = 18;

// ================================================================
// PARAMÈTRES DU ROBOT
// ================================================================
const int PWM_MIN = 1000;     // Arrêt total du moteur
const int PWM_MAX = 1250;     // BRIDAGE SÉCURITÉ (environ 25% de puissance)
const float ANGLE_MAX = 45.0; // Angle à partir duquel le moteur est à fond

// --- ÉTATS D'ERREUR ---
enum ErrorState { NO_ERROR = 0, ERR_MPU = 1, ERR_ESC = 2 };
ErrorState currentError = NO_ERROR;

// ================================================================
// VARIABLES CAPTEUR (MPU-9250 / MPU-6500)
// ================================================================
#define MPU_ADDR 0x68
float ax_offset=0, ay_offset=0, az_offset=0;
float gx_offset=0, gy_offset=0, gz_offset=0;
float angle_x = 0.0f;
unsigned long last_us = 0;
#define COMP_ALPHA 0.98f

Servo esc;

// ================================================================
// FONCTIONS I2C & MPU
// ================================================================
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void mpu_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)len, (uint8_t)true);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== DÉMARRAGE DU ROBOT TETRACUBLI ===");
    
    // 1. Initialisation I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // 2. Vérification du Capteur
    Serial.println("Recherche du capteur MPU...");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75); // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)1);
    uint8_t identity = Wire.read();
    
    // On accepte 0x71 (MPU9250), 0x73 (MPU9255) et 0x70 (MPU6500)
    if (identity != 0x71 && identity != 0x70 && identity != 0x73) {
        currentError = ERR_MPU;
        Serial.printf("❌ ERREUR FATALE : Capteur I2C introuvable (Code: 0x%X)\n", identity);
    } else {
        Serial.printf("✅ Capteur trouvé ! (ID: 0x%X)\n", identity);
    }

    // 3. Initialisation ESC
    Serial.println("Armement de l'ESC...");
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    esc.setPeriodHertz(50);
    esc.attach(ESC_PIN, 1000, 2000);
    esc.writeMicroseconds(1000); // Envoi du signal "zéro" pour l'armement

    // 4. Calibration (Seulement si le capteur est présent)
    if (currentError == NO_ERROR) {
        mpu_write(0x6B, 0x01); // PWR_MGMT_1 : Réveil
        mpu_write(0x1B, 0x08); // GYRO_CONFIG : ±500°/s
        mpu_write(0x1C, 0x00); // ACCEL_CONFIG : ±2g

        Serial.println("⏱️ NE BOUGEZ PAS LE CAPTEUR - Calibration en cours...");
        long sax=0, say=0, saz=0, sgx=0;
        
        // On prend 200 mesures pour faire une moyenne (dure environ 1 seconde)
        for (int i = 0; i < 200; i++) {
            uint8_t buf[14];
            mpu_read_burst(0x3B, buf, 14);
            sax += (int16_t)(buf[0]<<8|buf[1]);
            say += (int16_t)(buf[2]<<8|buf[3]);
            saz += (int16_t)(buf[4]<<8|buf[5]);
            sgx += (int16_t)(buf[8]<<8|buf[9]);
            delay(5);
        }
        ax_offset = sax/200.0f; 
        ay_offset = say/200.0f; 
        az_offset = saz/200.0f - 16384.0f; // On retire la gravité sur l'axe Z
        gx_offset = sgx/200.0f;

        // On laisse 3 secondes de plus pour être sûr que l'ESC a fini sa musique d'armement
        Serial.println("Attente de la fin des bips de l'ESC...");
        delay(3000); 

        Serial.println("🚀 SYSTÈME ARMÉ ET PRÊT !");
    }
    
    last_us = micros();
}

// ================================================================
// LOOP (BOUCLE PRINCIPALE)
// ================================================================
void loop() {
    // --- MODE ERREUR : LE HACK DE LA LED TX ---
    if (currentError != NO_ERROR) {
        esc.writeMicroseconds(1000); // SÉCURITÉ ABSOLUE : Coupe le moteur

        int blinks = (currentError == ERR_MPU) ? 3 : 2;
        
        // Fait flasher la petite LED TX physiquement soudée sur la carte !
        for (int i = 0; i < blinks; i++) {
            Serial.println("████████████████████████████████████████████████████████████");
            delay(100); 
            delay(200); 
        }
        
        Serial.println(); // Pause visuelle dans la console
        delay(1500);      // Pause avant de répéter le code d'erreur
        
        return; // Stoppe la boucle ici
    }

    // --- MODE NORMAL (Équilibre) ---

    // 1. Timing précis pour le calcul d'angle (200Hz)
    unsigned long now = micros();
    float dt = (now - last_us) / 1000000.0f;
    if (dt < 0.005f) return;
    last_us = now;

    // 2. Lecture I2C ultra rapide
    uint8_t buf[14];
    mpu_read_burst(0x3B, buf, 14);
    
    // 3. Conversion en valeurs physiques
    float ax = ((int16_t)(buf[0]<<8|buf[1]) - ax_offset) / 16384.0f;
    float ay = ((int16_t)(buf[2]<<8|buf[3]) - ay_offset) / 16384.0f;
    float az = ((int16_t)(buf[4]<<8|buf[5]) - az_offset) / 16384.0f;
    float gx = ((int16_t)(buf[8]<<8|buf[9]) - gx_offset) / 65.5f;

    // 4. Calcul de l'angle (Filtre Complémentaire)
    float accel_angle_x = atan2f(ay, sqrtf(ax*ax + az*az)) * (180.0f / M_PI);
    angle_x = COMP_ALPHA * (angle_x + gx * dt) + (1.0f - COMP_ALPHA) * accel_angle_x;

    // 5. Commande du moteur (Correcteur Proportionnel)
    float inclinaison = fmin(fabs(angle_x), ANGLE_MAX); // On bloque le calcul à 45° max
    int pwr = map((long)inclinaison, 0, (long)ANGLE_MAX, PWM_MIN, PWM_MAX);
    
    esc.writeMicroseconds(pwr);

    // 6. Affichage Console (10 fois par seconde)
    static unsigned long lp = 0;
    if (millis() - lp > 100) {
        lp = millis();
        // Ce print fait scintiller doucement la LED TX en vol normal ("cœur qui bat")
        Serial.printf("Angle: %6.1f° | Puissance ESC: %4d us\n", angle_x, pwr);
    }
}