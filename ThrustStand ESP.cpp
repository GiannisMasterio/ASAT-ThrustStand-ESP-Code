#include <WiFi.h>
#include <PubSubClient.h>
#include <HX711_ADC.h>
#include "driver/ledc.h"
#include <Wire.h>

const int NUM_NETWORKS = 2;

const char* SSIDs[NUM_NETWORKS]     = {"Masterio", "Yiannis"};
const char* PASSWORDS[NUM_NETWORKS] = {"masterakhs95", "14052003"};
const char* MQTT_SERVERS[NUM_NETWORKS] = {"10.122.194.210", "192.168.231.124"};

const char* current_mqtt_server = MQTT_SERVERS[0];

const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENTID = "ESP32Controller";
const char* MQTT_TOPIC_SUB = "thrust-toesp";
const char* MQTT_TOPIC_PUB = "thrust-fromesp";

#define PIN_DOUT 18
#define PIN_SCK 19
#define ESC_PIN 23          // ESC PWM output pin
#define RPM_PIN 14          // RPM pulse input pin (hall/optical sensor)
#define I2C_SDA 21 //μπλε
#define I2C_SCL 22 // κιτρινο

// INA228 / INA-like I2C

#define I2C_ADDR       0x45
#define I2C_FREQ 400000
#define REG_VBUS       0x05
#define REG_CURRENT    0x07

// ------------------ ESC PWM ------------------

#define ESC_CHANNEL   LEDC_CHANNEL_0
#define ESC_TIMER     LEDC_TIMER_0
#define ESC_SPEEDMODE LEDC_LOW_SPEED_MODE

// ------------------ Globals ------------------

WiFiClient espClient;
PubSubClient client(espClient);
HX711_ADC scale(PIN_DOUT, PIN_SCK);

// ------------------ Scaling & constants ------------------

const float INA228_VBUS_LSB   = 0.0001953125f;
const float PM02D_VBUS_SCALE  = (1.0f / 16.0f);
const float CURRENT_LSB_USER  = 0.00003688f; //0.00004465f

float ZERO_OFFSET = 38468.909;     // The raw value when vertical & empty
float THRUST_CALIBRATION_FACTOR = 0.4485; // The "steps per gram"
float gravity = 9.81;

bool use_lowpass_filter = false;
float percentage_old_measurement = 0.8f;

const float MIN_THROTTLE = 1100.0f;
const float MAX_THROTTLE = 2000.0f;

const int PUBLISH_INTERVAL_MS = 100;//oldrpm
// --- Global Declarations ---
const int POLE_PAIRS = 7;
volatile unsigned long last_pulse_us = 0;
volatile unsigned long pulse_period_us = 0;

// Interrupt Service Routine: captures microseconds between pulses
void IRAM_ATTR onRpmPulse() {
    unsigned long now_us = micros();
    pulse_period_us = now_us - last_pulse_us;
    last_pulse_us = now_us;
}
const float CAL_FACTOR = 220.0f;
const int SAMPLE_SIZE = 5;
const float TICKS_PER_US = 65536.0 / 20000.0;

struct Telemetry {
  float newton = 0.0F;
  float rpm = 0.0f;
  float current = 0.0f;
  float voltage = 0.0f;
  float torque = 0.0f;
};

volatile unsigned long pulse_count = 0;
unsigned long last_read_time = 0;
unsigned long lastPublishTime = 0;

float loadcellAccum = 0.0f;
uint8_t loadcellCount = 0;

float current_throttle = MIN_THROTTLE;
bool escArmed = false;
bool scale_is_ok = false;
float stand_offset = 37500;
float DIVIDER = 5.0505;

int wait_for_network_ms = 6000; // perimenoyme 6 seconds gia neo connection 

Telemetry telem; // σε αυτο το instance θα κρατιουνται ολες οι τιμες της τηλεμετριας

void connectToWiFiandMQTT() {

    for (int i = 0; i < NUM_NETWORKS; i++) {

        Serial.print("\nTrying to connect to: ");

        Serial.println(SSIDs[i]);
        WiFi.disconnect(true); // Πλήρης αποσύνδεση και καθαρισμός
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.begin(SSIDs[i], PASSWORDS[i]);

        unsigned long startAttemptTime = millis();

        // Περιμένουμε μέχρι να συνδεθεί ή να περάσει ο χρόνος

        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wait_for_network_ms) {

            delay(500);
            Serial.print(".");

        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Connected!");
            current_mqtt_server = MQTT_SERVERS[i];
            client.setServer(current_mqtt_server, MQTT_PORT);
            return;

        } else {
            Serial.println("\nTimeout or failed.");
        }

    }

}

// ------------------ I2C Helpers ------------------

uint32_t readRegister20bit(uint8_t reg){ //  ΤΑΣΗ 20BIT

    Wire.beginTransmission(I2C_ADDR); // εναρξη επικοινωνιας I2C με βαση το I2C ADDRESS
    Wire.write(reg); // γραφω απο ποιο reg θελω να διαβασω -- address oxi δατα
    if (Wire.endTransmission(false) != 0) { 
        return 0; // Bus error or device NACK -- μη επιτυχες request
    }
    
    Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)3); // ζητησε απτο I2C ADDRESS ΑΥΤΟ 3 BYTES -- kathws exw 20bit register

    uint32_t raw = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read(); // 32 bit integer -> shift left 16bit to prwto byte 
    return raw & 0xFFFFF; // krataw ta pio useless bits -> kanw and me ayto to mask -> dld exw krathsei ta 20 pio deksia

}

int32_t readRegister24bitSigned(uint8_t reg){ // ΡΕΥΜΑ ΕΙΝΑΙ 24BIT ΓΤ ΜΠΟΡΕΙ ΝΑ ΕΙΝΑΙ ΑΡΝΗΤΙΚΟ -> GIAYTO KANW RETURN SIGNED

    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { 
        return 0; // Bus error or device NACK -- μη επιτυχες request
    }
    Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)3);
    int32_t raw = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    if (raw & 0x800000) raw |= 0xFF000000;
    return raw;

}

void updateVoltageAndCurrent(){

    uint32_t rawV = readRegister20bit(REG_VBUS); // απλα προσδιοριζω ποιο reg θελω για την ταση και το περναω
    telem.voltage = rawV * INA228_VBUS_LSB * PM02D_VBUS_SCALE; // πως απο καθαρη μετρηση παω στην πραγματικη τιμη τασης

    long sumC = readRegister24bitSigned(REG_CURRENT); // παιρνει μεσο ορο απο 2 καθαρες μετρησεις
    sumC += readRegister24bitSigned(REG_CURRENT);
    float rawC_avg = (float)sumC / 2.0f;
    telem.current = (rawC_avg * CURRENT_LSB_USER ); 

}

// ------------------ Pulse ISR ------------------

void IRAM_ATTR count_pulses() {
    pulse_count++;
 // IRAM_ATTR είναι attribute του ESP32 που ζητά να τοποθετηθεί ο κώδικας της συνάρτησης στη μνήμη IRAM, ώστε να μπορεί να εκτελείται κατάλληλα μέσα σε interrupt.
 // ετσι πετυχαινουμε πιο γρηγορη αναγνωση των παλμων του RPM SENSOR
}

// ------------------ ESC Control ------------------

void setMotorPWM(float microseconds) {

    uint32_t maxDuty = (1 << 16) - 1;
    uint32_t duty = (uint32_t)((microseconds / 20000.0f) * maxDuty);
    if (duty > maxDuty) duty = maxDuty;
    ledc_set_duty(ESC_SPEEDMODE, ESC_CHANNEL, duty);    
    ledc_update_duty(ESC_SPEEDMODE, ESC_CHANNEL);

}

// ------------------ MQTT ------------------

const char* buildPayload(Telemetry* telem, float current_throttle) {

    static char packet[120];
    snprintf(packet, sizeof(packet),"Thrust:%.2f,Current:%.4f,Voltage:%.2f,Throttle:%.0f,RPM:%.0f,Torque:%.2f", telem->newton, telem->current, telem->voltage, current_throttle, telem->rpm, telem->torque);
    return packet;

}

void callback(char* topic, byte* payload, unsigned int length) {

    char message_str[64];
    if (length >= sizeof(message_str)) length = sizeof(message_str)-1;
    memcpy(message_str, payload, length);
    message_str[length] = '\0';
    float new_throttle;

    if (sscanf(message_str, "Throttle:%f", &new_throttle) == 1) {
        new_throttle = constrain(new_throttle, MIN_THROTTLE, MAX_THROTTLE);
        current_throttle = new_throttle;
        if (escArmed) setMotorPWM(current_throttle);
    }

}

void reconnect() {

    while (!client.connected()) { // Αν χάθηκε το WiFi, ξαναδοκίμασε τη λίστα

        if (WiFi.status() != WL_CONNECTED) {
            connectToWiFiandMQTT();
        }

        Serial.print("Attempting MQTT connection to ");
        Serial.print(current_mqtt_server);
        Serial.print("...");

        if (client.connect(MQTT_CLIENTID)) {
            Serial.println("connected");
            client.subscribe(MQTT_TOPIC_SUB);

        } else {

            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(3000);

        }

    }

}

void esc_initialize(){
    setMotorPWM(MIN_THROTTLE);
    delay(3000);
    escArmed = true;
}

void setup() {

    Serial.begin(115200);
    delay(200);
    
    // RPM pin
    pinMode(RPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), count_pulses, RISING); // κατα τη θετικη ανοδο αυξανω τον μετρητη παλμων
    
    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);

    // PWM for ESC
 
    ledc_timer_config_t timerConfig = { // Δημιουργουμε μια δομή ρυθμίσεων για τον LEDC timer του ESP32. Ο LEDC είναι το περιφερειακό που παράγει το PWM hardware.

        .speed_mode       = ESC_SPEEDMODE,
        .duty_resolution  = LEDC_TIMER_16_BIT,
        .timer_num        = ESC_TIMER,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK

    };

    ledc_timer_config(&timerConfig);
    ledc_channel_config_t channelConfig = {

        .gpio_num       = ESC_PIN,
        .speed_mode     = ESC_SPEEDMODE,
        .channel        = ESC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = ESC_TIMER,
        .duty           = 0,
        .hpoint         = 0

    };
    ledc_channel_config(&channelConfig);

    esc_initialize();

    connectToWiFiandMQTT();

    scale.begin();
    unsigned long start = millis();
    while (!scale.update() && millis() - start < 3000) delay(5);

    if (millis() - start < 3000) {

        scale_is_ok = true;
        scale.setCalFactor(CAL_FACTOR);

    } else scale_is_ok = false;

    client.setCallback(callback);
    last_read_time = millis();

}

void updateThrust() {
    if (loadcellCount<=0) return;

    float raw_average = loadcellAccum / loadcellCount;
    float grams = - (raw_average - ZERO_OFFSET) / THRUST_CALIBRATION_FACTOR;
    telem.newton = (grams / 1000.0) * gravity;
    loadcellAccum = 0.0f;
    loadcellCount = 0;
}

void loop() {

    if (!client.connected()) reconnect(); 
    client.loop();

    if (scale_is_ok && scale.update()) {

        float v = scale.getData();
        loadcellAccum += v;
        loadcellCount++;
    }
    unsigned long now = millis();//new rpm

    if (now - lastPublishTime >= PUBLISH_INTERVAL_MS) {
        lastPublishTime = now; // remember to advance publish timer

        // 1. Safely read period from interrupt
        noInterrupts();
        unsigned long period_us = pulse_period_us;
        unsigned long last_pulse = last_pulse_us;
        interrupts();

        // 2. Check if motor stopped (no pulse in the last 200ms)
        float current_rpm = 0.0f;
        if (micros() - last_pulse < 200000UL && period_us > 0) {
            
            current_rpm = 60000000.0f / ((float)period_us * POLE_PAIRS); // (60 sec * 1,000,000 us) / (period * pole_pairs)
        }

        // 3. Low-pass filter (optional, but now continuous)
        if (use_lowpass_filter) {
            telem.rpm = (telem.rpm * percentage_old_measurement) + 
                        (current_rpm * (1.0f - percentage_old_measurement));
        } else {
            telem.rpm = current_rpm;
        }

        updateThrust();
        updateVoltageAndCurrent();

        const char* payload = buildPayload(&telem, current_throttle);
        client.publish(MQTT_TOPIC_PUB, payload);
    }

}