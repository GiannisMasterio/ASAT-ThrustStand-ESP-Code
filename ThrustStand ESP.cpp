#include <WiFi.h>
#include <PubSubClient.h>
#include "HX711-multi.h"
#include "driver/ledc.h"
#include <Wire.h>

const int NUM_NETWORKS = 3;

const char* SSIDs[NUM_NETWORKS]        = {"Masterio", "Yiannis", "AndroidAP" };
const char* PASSWORDS[NUM_NETWORKS]    = {"masterakhs95", "14052003", "password1" };
const char* MQTT_SERVERS[NUM_NETWORKS] = {"172.22.128.124", "10.171.118.124", "10.101.124.135" };

const char* current_mqtt_server = MQTT_SERVERS[0];

const uint16_t MQTT_PORT  = 1883;
const char* MQTT_CLIENTID = "ESP32Controller";
const char* MQTT_TOPIC_SUB = "thrust-toesp";
const char* MQTT_TOPIC_PUB = "thrust-fromesp";

// ----- 3 LOAD CELLS ΜΕ ΚΟΙΝΟ CLOCK (HX711Multi) -----
#define PIN_SCK 19
#define PIN_DOUT_F1 18   // Thrust
#define PIN_DOUT_T1 20   // Torque 1
#define PIN_DOUT_T2 21   // Torque 2

byte DOUTS[3] = {PIN_DOUT_F1, PIN_DOUT_T1, PIN_DOUT_T2};
HX711MULTI scales(3, DOUTS, PIN_SCK);

#define ESC_PIN 23
#define RPM_PIN 4
#define I2C_SDA 6
#define I2C_SCL 7

// ----- INA228 / I2C -----
#define I2C_ADDR       0x45
#define I2C_FREQ       400000 
#define REG_VBUS       0x05
#define REG_CURRENT    0x07

// ----- ESC PWM -----
#define ESC_CHANNEL   LEDC_CHANNEL_0
#define ESC_TIMER     LEDC_TIMER_0
#define ESC_SPEEDMODE LEDC_LOW_SPEED_MODE

const float INA228_VBUS_LSB   = 0.0001953125f;
const float PM02D_VBUS_SCALE  = (1.0f / 16.0f);
const float CURRENT_LSB_USER  = 0.00003688f; 

bool use_lowpass_filter = false;
float percentage_old_measurement = 0.8f;

const float MIN_THROTTLE = 1130.0f;
const float MAX_THROTTLE = 2000.0f;
const int PUBLISH_INTERVAL_MS = 100; 
const int POLE_PAIRS = 7;
const int wait_for_network_ms = 5000; 

volatile unsigned long last_pulse_us = 0;
volatile unsigned long pulse_period_us = 0;
unsigned long lastPublishTime = 0;
bool escArmed = false;

struct Telemetry {
    float newton = 0.0f;  
    float rpm = 0.0f;
    float current = 0.0f;
    float voltage = 0.0f;
    float torque = 0.0f;  
    float throttle = MIN_THROTTLE;
};

WiFiClient espClient;
PubSubClient client(espClient);
Telemetry telem; 

// ------------------ RPM ISR ------------------
void IRAM_ATTR onRpmPulse() {
    unsigned long now_us = micros();
    unsigned long diff = now_us - last_pulse_us;
    if (diff > 500) { 
        pulse_period_us = diff;
        last_pulse_us = now_us;
    }
}

// ------------------ WiFi & MQTT ------------------
void connectToWiFiandMQTT() {
    for (size_t i = 0; i < NUM_NETWORKS; i++) {
        Serial.print("\nTrying to connect to: ");
        Serial.println(SSIDs[i]);
        WiFi.disconnect(true); 
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.begin(SSIDs[i], PASSWORDS[i]);

        unsigned long startAttemptTime = millis();
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

void reconnect() {
    while (!client.connected()) { 
        if (WiFi.status() != WL_CONNECTED) connectToWiFiandMQTT();
        
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

// ------------------ I2C Helpers ------------------
uint32_t readRegister20bit(uint8_t reg){ 
    Wire.beginTransmission(I2C_ADDR); 
    Wire.write(reg); 
    if (Wire.endTransmission(false) != 0) return 0; 
    
    Wire.requestFrom(static_cast<uint8_t>(I2C_ADDR), static_cast<uint8_t>(3)); 
    uint32_t raw = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read(); 
    return raw & 0xFFFFF; 
}

int32_t readRegister24bitSigned(uint8_t reg){ 
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0; 

    Wire.requestFrom(static_cast<uint8_t>(I2C_ADDR), static_cast<uint8_t>(3));
    int32_t raw = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    if (raw & 0x800000) raw |= 0xFF000000;
    return raw;
}

void updateVoltageAndCurrent(){
    uint32_t rawV = readRegister20bit(REG_VBUS); 
    telem.voltage = rawV * INA228_VBUS_LSB * PM02D_VBUS_SCALE; 

    long sumC = readRegister24bitSigned(REG_CURRENT); 
    sumC += readRegister24bitSigned(REG_CURRENT);
    
    float rawC_avg = static_cast<float>(sumC) / 2.0f;
    telem.current = (rawC_avg * CURRENT_LSB_USER); 
}

// ------------------ ESC Control ------------------
void setMotorPWM(float microseconds) {
    uint32_t maxDuty = (1 << 16) - 1;
    uint32_t duty = static_cast<uint32_t>((microseconds / 20000.0f) * maxDuty);
    if (duty > maxDuty) duty = maxDuty;
    ledc_set_duty(ESC_SPEEDMODE, ESC_CHANNEL, duty);    
    ledc_update_duty(ESC_SPEEDMODE, ESC_CHANNEL);
}

void esc_initialize(){
    setMotorPWM(MIN_THROTTLE);
    delay(3000);
    escArmed = true;
}

// ------------------ MQTT Callbacks ------------------
const char* buildPayload() {
    static char packet[120];
    snprintf(packet, sizeof(packet),"Thrust:%.2f,Current:%.4f,Voltage:%.2f,Throttle:%.0f,RPM:%.0f,Torque:%.2f", 
             telem.newton, telem.current, telem.voltage, telem.throttle, telem.rpm, telem.torque);
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
        telem.throttle = new_throttle;
        if (escArmed) setMotorPWM(telem.throttle);
    }
}

// ------------------ SETUP ------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    
    pinMode(RPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), onRpmPulse, RISING); 
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);

    ledc_timer_config_t timerConfig = { 
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
    client.setCallback(callback);
    
    Serial.println("Initializing HX711Multi in RAW mode...");
}

// ------------------ LOOP ------------------
void loop() {
    if (!client.connected()) reconnect(); 
    client.loop();

    // 1. Ταυτόχρονη ανάγνωση των 3 Load Cells
    if (scales.is_ready()) {
        long raw_values[3];
        scales.readRaw(raw_values); // Διαβάζει ταυτόχρονα και τα 3 DOUTs
        
        telem.newton = (float)raw_values[0];                                // Thrust RAW
        telem.torque = ((float)raw_values[1] + (float)raw_values[2]) / 2.0f; // Torque RAW Average
    }

    unsigned long now = millis(); 
    if (now - lastPublishTime >= PUBLISH_INTERVAL_MS) {
        
        noInterrupts();
        unsigned long period_us = pulse_period_us;
        unsigned long last_pulse = last_pulse_us;
        interrupts();

        updateVoltageAndCurrent();

        float current_rpm = 0.0f;
        if (micros() - last_pulse < 250000UL && period_us > 0) {
            current_rpm = 60000000.0f / (static_cast<float>(period_us) * POLE_PAIRS);
        }

        if (use_lowpass_filter) {
            if (current_rpm == 0.0f) telem.rpm = 0.0f; 
            else telem.rpm = (telem.rpm * percentage_old_measurement) + (current_rpm * (1.0f - percentage_old_measurement)); 
        } else {
            telem.rpm = current_rpm;
        }

        const char* payload = buildPayload();
        client.publish(MQTT_TOPIC_PUB, payload);
        lastPublishTime = now; 
    }
}