#include <Arduino.h>
#include <ArduinoJson.h>

// =============================================================================
// SLAVE ESP32 — SOIL MOISTURE 1-4
// Reads soil moisture sensors 1-4 and sends values to the main ESP via UART.
//
// Wiring:
//   Slave GPIO17 (TX2) ────> Main GPIO16 (RX2)
//   Slave GPIO16 (RX2) <──── Main GPIO17 (TX2)   [reserved, unused]
//   GND                ────  GND
// =============================================================================

// ========== SHARED CONSTANTS ==========
// Must match values in main_esp.cpp
#define UART_BAUD   115200
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define ADC_DRY     2662
#define ADC_WET     1269

// ========== PIN CONFIGURATION ==========
// ADC1 pins only — safe, no WiFi interference
#define SOIL1_PIN 32
#define SOIL2_PIN 33
#define SOIL3_PIN 34
#define SOIL4_PIN 35

// ========== TIMING ==========
#define SEND_INTERVAL 1000  // ms between transmissions

#define SLAVE_SERIAL Serial2

// ========== ADC HELPERS ==========
int readADC(int pin)
{
    long sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += analogRead(pin);
        delay(5);
    }
    return (int)(sum / 10);
}

int readSoilPercent(int pin)
{
    int raw = readADC(pin);
    raw = constrain(raw, ADC_WET, ADC_DRY);
    return map(raw, ADC_DRY, ADC_WET, 0, 50);
}

// ========== SETUP ==========
void setup()
{
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);
    analogSetPinAttenuation(SOIL1_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL2_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL3_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL4_PIN, ADC_11db);

    pinMode(SOIL1_PIN, INPUT);
    pinMode(SOIL2_PIN, INPUT);
    pinMode(SOIL3_PIN, INPUT);
    pinMode(SOIL4_PIN, INPUT);

    // UART to main ESP — slave only transmits (TX2)
    SLAVE_SERIAL.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.println("Slave ESP32 ready - sending soil moisture 1-4");
}

// ========== LOOP ==========
void loop()
{
    static unsigned long lastSend = 0;

    if (millis() - lastSend >= SEND_INTERVAL)
    {
        lastSend = millis();

        int s1 = readSoilPercent(SOIL1_PIN);
        int s2 = readSoilPercent(SOIL2_PIN);
        int s3 = readSoilPercent(SOIL3_PIN);
        int s4 = readSoilPercent(SOIL4_PIN);

        // Send compact JSON line terminated with \n — main ESP parses this
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "{\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d}\n",
                 s1, s2, s3, s4);
        SLAVE_SERIAL.print(buf);

        Serial.print("Sent: ");
        Serial.print(buf);
    }
}