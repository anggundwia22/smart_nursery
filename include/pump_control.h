#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <Arduino.h>
#include <RTClib.h>

#define RELAY_PIN 26
#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define COOLDOWN_TIME 300000UL

enum PumpState
{
    PUMP_IDLE,
    PUMP_RUNNING,
    PUMP_COOLDOWN,
    PUMP_ERROR
};

struct PumpControl
{
    PumpState state = PUMP_IDLE;
    unsigned long startTime = 0;
    unsigned long cooldownStart = 0;
    bool irrigationDone[2] = {false, false};
};

struct Config
{
    int threshold = 60;
    int dry = 2662;
    int wet = 1269;
    int pumpDuration = 60000;
    int measurementInterval = 3600000;
    int dataLogInterval = 3600000;
    int irrigationHour1 = 7;
    int irrigationMinute1 = 0;
    int irrigationSecond1 = 0;
    int irrigationHour2 = 16;
    int irrigationMinute2 = 0;
    int irrigationSecond2 = 0;
};

struct SensorData
{
    float temperature = 0.0f;
    float humidity = 0.0f;
    int soilMoisture1 = 0;
    int soilMoisture2 = 0;
    int soilMoisture3 = 0;
    int soilMoisture4 = 0;
    float lux = 0.0f;
    unsigned long lastMeasurement = 0;
    unsigned long lastDataLog = 0;
};

extern Config config;
extern PumpControl pumpControl;
extern SensorData data;
extern void serialPrintln(const char *message);
extern void logToFile(const char *message);

void controlPump(DateTime &currentTime);

#endif
