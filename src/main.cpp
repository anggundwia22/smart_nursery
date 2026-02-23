#include <Arduino.h>
#include <DHT.h>
#include <RTClib.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <BH1750.h>

// ========== PIN CONFIGURATION ==========
#define DHTPIN 4
#define DHTTYPE DHT22
#define SOIL1_MOISTURE_PIN 14
#define SOIL2_MOISTURE_PIN 25
#define SOIL3_MOISTURE_PIN 26
#define SOIL4_MOISTURE_PIN 27
#define SOIL5_MOISTURE_PIN 32
#define SOIL6_MOISTURE_PIN 33
#define SOIL7_MOISTURE_PIN 34
#define SOIL8_MOISTURE_PIN 35
#define SOIL9_MOISTURE_PIN 36
#define SOIL10_MOISTURE_PIN 37
#define RELAY_PIN 19

// ========== RELAY CONTROL ==========
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// ========== SYSTEM CONFIGURATION ==========
#define CONFIG_FILE "/config.json"
#define SERIAL_BUFFER_SIZE 100
#define WDT_TIMEOUT 180 // 3 minutes watchdog timeout
#define MINIMUM_INTERVAL 1000UL
#define COOLDOWN_TIME 300000UL // 5 minutes
#define DATA_LOG_INTERVAL 3600000
#define DATA_LOG_FILE "/data_log.csv"

// ========== GLOBAL OBJECTS ==========
DHT dht(DHTPIN, DHTTYPE);
RTC_DS3231 rtc;
BH1750 lightMeter;
WebServer server(80);

// ========== SENSOR DATA ==========
struct SensorData
{
    float temperature = 0.0f;
    float humidity = 0.0f;
    float lux = 0.0f;
    int soilMoisture1 = 0;
    int soilMoisture2 = 0;
    int soilMoisture3 = 0;
    int soilMoisture4 = 0;
    int soilMoisture5 = 0;
    int soilMoisture6 = 0;
    int soilMoisture7 = 0;
    int soilMoisture8 = 0;
    int soilMoisture9 = 0;
    int soilMoisture10 = 0;
    unsigned long lastMeasurement = 0;
    unsigned long lastDataLog = 0;
} data;

// ========== CONFIGURATION ==========
struct Config
{
    int threshold = 60;
    int dry = 2662;
    int wet = 1269;
    int pumpDuration = 60000;
    int measurementInterval = 3600000;
    int dataLogInterval = 3600000;
    int irrigationHour1 = 7;   // Jadwal penyiraman 1 - jam
    int irrigationMinute1 = 0; // Jadwal penyiraman 1 - menit
    int irrigationSecond1 = 0; // Jadwal penyiraman 1 - detik
    int irrigationHour2 = 16;  // Jadwal penyiraman 2 - jam
    int irrigationMinute2 = 0; // Jadwal penyiraman 2 - menit
    int irrigationSecond2 = 0; // Jadwal penyiraman 2 - detik
} config;

// ========== LOG BUFFER ==========
struct LogMessage
{
    unsigned long timestamp;
    char message[80];
};

LogMessage serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
int totalMessages = 0;

// ========== PUMP STATE MACHINE ==========
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
} pumpControl;

// ========= STATUS SYSTEM ==========
struct SystemStatus
{
    bool rtcInitialized = false;
    bool bh1750OK = false;
} status;

String getDataLogFilename()
{
    return String("/data_log_") + ".csv";
}

// ========== FUNCTION PROTOTYPES ==========
void serialPrintln(const char *message); // untuk menampilkan pesan di Serial Monitor dan menyimpan log ke buffer
void logToFile(const char *message);     // untuk menyimpan log ke file dengan timestamp dari RTC
void initRTC();                          // untuk inisialisasi RTC dan penyesuaian waktu jika diperlukan
void initWatchdog();                     // untuk inisialisasi watchdog timer
void resetWatchdog();                    // untuk mereset watchdog timer agar mencegah reset sistem
bool setupLittleFS();                    // untuk inisialisasi LittleFS dan memastikan filesystem siap digunakan
void createDefaultConfig();              // untuk membuat file konfigurasi default jika belum ada
bool loadConfig();                       // untuk memuat konfigurasi dari file dan mengisi struktur Config
bool saveConfig();                       // untuk menyimpan konfigurasi saat ini ke file dalam format JSON
void validateMeasurementInterval();      // untuk memastikan interval pengukuran tidak kurang dari batas minimum

void readDHT22(); // untuk membaca data suhu dan kelembapan dari sensor DHT22, serta menyimpan hasilnya ke struktur SensorData
// void readSoilMoistureSensor();
int readSoilPercent(int pin); // untuk membaca data kelembaban tanah dari sensor menggunakan ADC, kemudian mengkonversi nilai tersebut menjadi persentase berdasarkan kalibrasi ADC_DRY dan ADC_WET
void readSoilMoisture();      // untuk membaca kelembaban tanah dari semua sensor yang terhubung dan menyimpan hasilnya ke struktur SensorData
void initLuxMeter();          // untuk inisialisasi sensor cahaya BH1750
void readLuxMeter();          // untuk membaca data cahaya dari sensor BH1750 dan menampilkan hasilnya di Serial Monitor serta menyimpan log

void setupWiFi();          // untuk menghubungkan ESP32 ke jaringan WiFi menggunakan SSID dan password yang telah ditentukan
void setupWebServer();     // untuk menginisialisasi web server, mendefinisikan rute HTTP, dan memulai server untuk menerima permintaan dari klien
void handleRoot();         // untuk menangani permintaan HTTP ke rute root ("/"), biasanya digunakan untuk menampilkan halaman utama dengan informasi status sistem
void handleStatus();       // untuk menangani permintaan HTTP ke rute "/status", biasanya digunakan untuk mengirimkan data sensor dalam format JSON sebagai respons
void handleConfig();       // untuk menangani permintaan HTTP ke rute "/config", biasanya digunakan untuk menerima data konfigurasi baru dari klien, memperbarui struktur Config, dan menyimpan perubahan ke file
void handleSettings();     // untuk menangani permintaan HTTP ke rute "/settings", biasanya digunakan untuk menerima data konfigurasi baru dari klien, memperbarui struktur Config, dan menyimpan perubahan ke file
void handleRestart();      // untuk menangani permintaan HTTP ke rute "/restart", biasanya digunakan untuk mereset sistem secara manual melalui antarmuka web
void handleLogs();         // untuk menangani permintaan HTTP ke rute "/logs", biasanya digunakan untuk mengirimkan log pesan yang disimpan dalam buffer sebagai respons dalam format JSON
void handleTime();         // untuk menangani permintaan HTTP ke rute "/time", biasanya digunakan untuk mengirimkan waktu saat ini dari RTC dalam format JSON sebagai respons
void handleDateTime();     // untuk menangani permintaan HTTP ke rute "/datetime", biasanya digunakan untuk menerima data tanggal dan waktu baru dari klien, memperbarui RTC dengan nilai tersebut, dan mengirimkan respons status kepada klien
void handleDataDownload(); // untuk menangani permintaan HTTP ke rute "/data/download", biasanya digunakan untuk mengirimkan file data log dalam format CSV sebagai respons untuk diunduh oleh klien
void handleDataDelete();   // untuk menangani permintaan HTTP ke rute "/data/delete", biasanya digunakan untuk menghapus file data log yang ada dan mengirimkan respons status kepada klien
void handleDataInfo();     // untuk menangani permintaan HTTP ke rute "/data/info", biasanya digunakan untuk mengirimkan informasi tentang file data log yang ada, seperti ukuran dan tanggal terakhir diubah, dalam format JSON sebagai respons
void initDataLog();        // untuk menginisialisasi file data log, memastikan file tersebut ada dan memiliki header yang benar jika baru dibuat
void saveDataRecord();     // untuk menyimpan rekaman data sensor saat ini ke file data log dalam format CSV dengan timestamp dari RTC

void resetDailyIrrigation(DateTime &currentTime);
void controlPump(DateTime &currentTime);

// ========== LOGGING FUNCTIONS ==========
void serialPrintln(const char *message)
{
    Serial.println(message);

    serialBuffer[serialBufferIndex].timestamp = millis();
    strncpy(serialBuffer[serialBufferIndex].message, message,
            sizeof(serialBuffer[0].message) - 1);
    serialBuffer[serialBufferIndex].message[sizeof(serialBuffer[0].message) - 1] = '\0';

    serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
    if (totalMessages < SERIAL_BUFFER_SIZE)
        totalMessages++;
}

void logToFile(const char *message)
{
    if (!status.rtcInitialized)
        return;

    DateTime now = rtc.now();
    char logBuffer[32];
    snprintf(logBuffer, sizeof(logBuffer), "/log_%04d%02d%02d.txt",
             now.year(), now.month(), now.day());

    File file = LittleFS.open(logBuffer, "a");
    if (!file)
        return;

    char timestamp[20];
    snprintf(timestamp, sizeof(timestamp), "[%02d:%02d:%02d] ",
             now.hour(), now.minute(), now.second());
    file.print(timestamp);
    file.println(message);
    file.close();
}

// ========== INITIALIZATION FUNCTIONS ==========
// void initRTC()
// {
//     Wire.begin();
//     if (!rtc.begin())
//     {
//         serialPrintln("RTC not detected");
//         return;
//     }

//     sensorData.rtcInitialized = true;
//     if (rtc.lostPower())
//     {
//         rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//         serialPrintln("RTC time adjusted");
//     }
//     serialPrintln("RTC initialized successfully");
// }

// void initRTC()
// {
//     Wire.begin();
//     if (!rtc.begin())
//     {
//         serialPrintln("RTC not found");
//         sensorData.rtcInitialized = false;
//     }
//     else
//     {
//         sensorData.rtcInitialized = true;
//         if (rtc.lostPower())
//         {
//             serialPrintln("RTC lost power, setting time!");
//             rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//         }
//         serialPrintln("RTC initialized successfully");
//     }
// }

void initRTC()
{
    Wire.begin();

    status.rtcInitialized = rtc.begin();

    if (!status.rtcInitialized)
    {
        serialPrintln("RTC not detected");
        return;
    }

    if (rtc.lostPower())
    {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    serialPrintln("RTC initialized successfully");
}

void initWatchdog()
{
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    serialPrintln("Watchdog initialized");
}

void resetWatchdog()
{
    esp_task_wdt_reset();
}

bool setupLittleFS()
{
    if (!LittleFS.begin())
    {
        serialPrintln("LittleFS Mount Failed");
        return false;
    }
    serialPrintln("LittleFS mounted successfully");
    return true;
}

// ========== CONFIGURATION FUNCTIONS ==========
void createDefaultConfig()
{
    JsonDocument doc;
    doc["threshold"] = 60;
    doc["dry"] = 2662;
    doc["wet"] = 1269;
    doc["pumpDuration"] = 60000;
    doc["measurementInterval"] = 3600000;
    doc["dataLogInterval"] = 3600000;
    doc["irrigationHour1"] = 7;
    doc["irrigationMinute1"] = 0;
    doc["irrigationSecond1"] = 0;
    doc["irrigationHour2"] = 16;
    doc["irrigationMinute2"] = 0;
    doc["irrigationSecond2"] = 0;

    File file = LittleFS.open(CONFIG_FILE, "w");
    if (!file)
    {
        serialPrintln("Failed to create default config file");
        return;
    }

    if (serializeJson(doc, file) == 0)
    {
        serialPrintln("Failed to write default config");
    }

    file.close();
    serialPrintln("Default config.json created successfully");
}

bool loadConfig()
{
    if (!LittleFS.exists(CONFIG_FILE))
    {
        return false;
    }

    File file = LittleFS.open(CONFIG_FILE, "r");
    if (!file)
        return false;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error)
        return false;

    config.threshold = doc["threshold"] | config.threshold;
    config.dry = doc["dry"] | config.dry;
    config.wet = doc["wet"] | config.wet;
    config.pumpDuration = doc["pumpDuration"] | config.pumpDuration;
    config.measurementInterval = doc["measurementInterval"] | config.measurementInterval;
    config.dataLogInterval = doc["dataLogInterval"] | config.dataLogInterval;
    config.irrigationHour1 = doc["irrigationHour1"] | config.irrigationHour1;
    config.irrigationMinute1 = doc["irrigationMinute1"] | config.irrigationMinute1;
    config.irrigationSecond1 = doc["irrigationSecond1"] | config.irrigationSecond1;
    config.irrigationHour2 = doc["irrigationHour2"] | config.irrigationHour2;
    config.irrigationMinute2 = doc["irrigationMinute2"] | config.irrigationMinute2;
    config.irrigationSecond2 = doc["irrigationSecond2"] | config.irrigationSecond2;

    // Log loaded irrigation schedule
    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer),
             "Irrigation Schedule: %02d:%02d:%02d and %02d:%02d:%02d",
             config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1,
             config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
    serialPrintln(logBuffer);

    return true;
}

bool saveConfig()
{
    File file = LittleFS.open(CONFIG_FILE, "w");
    if (!file)
        return false;

    JsonDocument doc;
    doc["threshold"] = config.threshold;
    doc["dry"] = config.dry;
    doc["wet"] = config.wet;
    doc["pumpDuration"] = config.pumpDuration;
    doc["measurementInterval"] = config.measurementInterval;
    doc["dataLogInterval"] = config.dataLogInterval;
    doc["irrigationHour1"] = config.irrigationHour1;
    doc["irrigationMinute1"] = config.irrigationMinute1;
    doc["irrigationSecond1"] = config.irrigationSecond1;
    doc["irrigationHour2"] = config.irrigationHour2;
    doc["irrigationMinute2"] = config.irrigationMinute2;
    doc["irrigationSecond2"] = config.irrigationSecond2;

    size_t bytesWritten = serializeJson(doc, file);
    file.close();

    if (bytesWritten > 0)
    {
        char logBuffer[80];
        snprintf(logBuffer, sizeof(logBuffer),
                 "Config saved - Schedule: %02d:%02d:%02d & %02d:%02d:%02d",
                 config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1,
                 config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
        serialPrintln(logBuffer);
    }

    return bytesWritten > 0;
}

void validateMeasurementInterval()
{
    if (config.measurementInterval < MINIMUM_INTERVAL)
    {
        config.measurementInterval = MINIMUM_INTERVAL;
        saveConfig();
    }
}

// ========== SENSOR READING FUNCTIONS ==========
void readDHT22()
{
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h))
    {
        data.temperature = t;
        data.humidity = h;

        char logBuffer[64];
        snprintf(logBuffer, sizeof(logBuffer),
                 "Temp: %.2f°C | Humidity: %.2f%%",
                 data.temperature, data.humidity);
        serialPrintln("DHT22 sensor read successfully");
        logToFile(logBuffer);
    }
    else
    {
        serialPrintln("Failed to read DHT22 sensor");
    }
}

// void readSoilMoistureSensor()
// {
//     int rawValue = analogRead(SOIL_MOISTURE_PIN);
//     sensorData.soilMoisture = map(rawValue, ADC_DRY, ADC_WET, 0, 100);
//     sensorData.soilMoisture = constrain(sensorData.soilMoisture, 0, 100);

//     char logBuffer[64];
//     snprintf(logBuffer, sizeof(logBuffer),
//              "Soil Moisture: %d%% (ADC: %d)",
//              sensorData.soilMoisture, rawValue);
//     serialPrintln(logBuffer);
//     logToFile(logBuffer);
// }

// int readSoilPercent(int pin)
// {
//     int raw = analogRead(pin);
//     raw = constrain(raw, 0, 4095); // sesuaikan hasil kalibrasi
//     return map(raw, 4095, 0, 0, 100);
// }

int readSoilPercent(int pin)
{
    int raw = analogRead(pin);
    raw = constrain(raw, config.dry, config.wet); // sesuaikan hasil kalibrasi
    return map(raw, config.wet, config.dry, 0, 100);
}

// int readSoilPercent(int pin)
// {
//     int raw = analogRead(pin);

//     // cek sensor terbaca atau tidak
//     if (raw < 500 || raw > 4000)
//     {
//         char logBuffer[80];
//         snprintf(logBuffer, sizeof(logBuffer),
//                  "Sensor tanah pada pin %d TIDAK Terdeteksi! Raw: %d", pin, raw);
//         serialPrintln(logBuffer);
//         return -1; // kode error
//     }

//     raw = constrain(raw, 1269, 2662);
//     int percent = map(raw, 2662, 1269, 0, 100);

//     char logBuffer[80];
//     snprintf(logBuffer, sizeof(logBuffer),
//              "Sensor tanah pada pin %d OK. Raw: %d Percent: %d%%", pin, raw, percent);
//     serialPrintln(logBuffer);

//     return percent;
// }

// int readSoilPercent(int pin)
// {
//     int raw = analogRead(pin);
//     raw = constrain(raw, 0, 4095); // sesuaikan hasil kalibrasi
//     return map(raw, 2662, 1269, 0, 100);
// }

// int readSoilPercent(int pin)
// {
//     int raw = analogRead(pin);
//     return raw;
// }

void readSoilMoisture()
{
    data.soilMoisture1 = readSoilPercent(SOIL1_MOISTURE_PIN);
    data.soilMoisture2 = readSoilPercent(SOIL2_MOISTURE_PIN);
    data.soilMoisture3 = readSoilPercent(SOIL3_MOISTURE_PIN);
    data.soilMoisture4 = readSoilPercent(SOIL4_MOISTURE_PIN);
    data.soilMoisture5 = readSoilPercent(SOIL5_MOISTURE_PIN);
    data.soilMoisture6 = readSoilPercent(SOIL6_MOISTURE_PIN);
    data.soilMoisture7 = readSoilPercent(SOIL7_MOISTURE_PIN);
    data.soilMoisture8 = readSoilPercent(SOIL8_MOISTURE_PIN);
    data.soilMoisture9 = readSoilPercent(SOIL9_MOISTURE_PIN);
    data.soilMoisture10 = readSoilPercent(SOIL10_MOISTURE_PIN);
}

void initLuxMeter()
{
    Wire.begin();

    status.bh1750OK = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

    if (status.bh1750OK)
        serialPrintln("BH1750 initialized");
    else
        serialPrintln("Error initializing BH1750");
}

void readLuxMeter()
{
    if (!status.bh1750OK)
    {
        Serial.println("BH1750 not available");
        return;
    }

    data.lux = lightMeter.readLightLevel();
    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer), "Light: %.2f lux", data.lux);
    logToFile(logBuffer);
}

// ========== IRRIGATION CONTROL ==========
void resetDailyIrrigation(DateTime &currentTime)
{
    static int lastDay = -1;

    if (currentTime.day() != lastDay)
    {
        lastDay = currentTime.day();
        pumpControl.irrigationDone[0] = false;
        pumpControl.irrigationDone[1] = false;

        char logBuffer[80];
        snprintf(logBuffer, sizeof(logBuffer),
                 "New day - Schedule reset (%02d:%02d:%02d & %02d:%02d:%02d)",
                 config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1,
                 config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
        serialPrintln(logBuffer);
        logToFile("Daily irrigation schedule reset");
    }
}

void startPump(int scheduleIndex, int hour, int minute, int second)
{
    pumpControl.state = PUMP_RUNNING;
    pumpControl.startTime = millis();

    digitalWrite(RELAY_PIN, RELAY_ON);

    pumpControl.irrigationDone[scheduleIndex] = true;

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer),
             //  "Pump START (%02d:%02d:%02d, Moisture: %d%%)",
             "Pump START (%02d:%02d:%02d)",
             hour, minute, second);

    serialPrintln(logBuffer);
    logToFile(logBuffer);
}

void controlPump(DateTime &currentTime)
{
    int currentHour = currentTime.hour();
    int currentMinute = currentTime.minute();
    int currentSecond = currentTime.second();

    // ==========================
    // SCHEDULE 1 - 07:00:00
    // ==========================

    if (currentHour == config.irrigationHour1 &&
        currentMinute == config.irrigationMinute1 &&
        currentSecond == 0 &&
        !pumpControl.irrigationDone[0] &&
        pumpControl.state == PUMP_IDLE)
    {
        startPump(0, config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1);
    }

    // ==========================
    // SCHEDULE 2 - 16:00:00
    // ==========================

    if (currentHour == config.irrigationHour2 &&
        currentMinute == config.irrigationMinute2 &&
        currentSecond == 0 &&
        !pumpControl.irrigationDone[1] &&
        pumpControl.state == PUMP_IDLE)
    {
        startPump(1, config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
    }

    // ==========================
    // STATE MACHINE
    // ==========================

    switch (pumpControl.state)
    {

    case PUMP_IDLE:
        break;

    case PUMP_RUNNING:

        if (millis() - pumpControl.startTime >= config.pumpDuration)
        {
            digitalWrite(RELAY_PIN, RELAY_OFF);

            pumpControl.state = PUMP_COOLDOWN;
            pumpControl.cooldownStart = millis();

            serialPrintln("Pump STOP");
            logToFile("Pump stopped");
        }

        break;

    case PUMP_COOLDOWN:

        if (millis() - pumpControl.cooldownStart >= COOLDOWN_TIME)
        {
            pumpControl.state = PUMP_IDLE;

            serialPrintln("Pump READY");
        }

        break;

    case PUMP_ERROR:

        digitalWrite(RELAY_PIN, RELAY_OFF);

        break;
    }
}

// ========== WEB SERVER HANDLERS ==========
void handleRoot()
{
    File file = LittleFS.open("/index.html", "r");
    if (!file)
    {
        server.send(404, "text/plain", "File not found");
        return;
    }

    server.streamFile(file, "text/html");
    file.close();
}

void handleStatus()
{
    JsonDocument doc;
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["soilMoisture1"] = data.soilMoisture1;
    doc["soilMoisture2"] = data.soilMoisture2;
    doc["soilMoisture3"] = data.soilMoisture3;
    doc["soilMoisture4"] = data.soilMoisture4;
    doc["pumpState"] = pumpControl.state;
    doc["threshold"] = config.threshold;
    doc["irrigationHour1"] = config.irrigationHour1;
    doc["irrigationMinute1"] = config.irrigationMinute1;
    doc["irrigationHour2"] = config.irrigationHour2;
    doc["irrigationMinute2"] = config.irrigationMinute2;

    if (status.rtcInitialized)
    {
        DateTime now = rtc.now();
        char timeStr[20];
        snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
        doc["timestamp"] = timeStr;
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleConfig()
{
    JsonDocument doc;
    doc["threshold"] = config.threshold;
    doc["dry"] = config.dry;
    doc["wet"] = config.wet;
    doc["pumpDuration"] = config.pumpDuration;
    doc["measurementInterval"] = config.measurementInterval;
    doc["dataLogInterval"] = config.dataLogInterval;
    doc["irrigationHour1"] = config.irrigationHour1;
    doc["irrigationMinute1"] = config.irrigationMinute1;
    doc["irrigationSecond1"] = config.irrigationSecond1;
    doc["irrigationHour2"] = config.irrigationHour2;
    doc["irrigationMinute2"] = config.irrigationMinute2;
    doc["irrigationSecond2"] = config.irrigationSecond2;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleSettings()
{
    if (server.hasArg("threshold"))
    {
        config.threshold = server.arg("threshold").toInt();
    }
    if (server.hasArg("dry"))
    {
        config.dry = server.arg("dry").toInt();
    }
    if (server.hasArg("wet"))
    {
        config.wet = server.arg("wet").toInt();
    }
    if (server.hasArg("pumpDuration"))
    {
        config.pumpDuration = server.arg("pumpDuration").toInt();
    }
    if (server.hasArg("measurementInterval"))
    {
        unsigned long seconds = server.arg("measurementInterval").toInt();
        config.measurementInterval = max(MINIMUM_INTERVAL, seconds * 1000UL);
    }
    if (server.hasArg("dataLogInterval"))
    {
        unsigned long seconds = server.arg("dataLogInterval").toInt();
        config.dataLogInterval = seconds * 1000UL;
    }
    if (server.hasArg("irrigationHour1"))
    {
        int hour = server.arg("irrigationHour1").toInt();
        if (hour >= 0 && hour <= 23)
        {
            config.irrigationHour1 = hour;
        }
    }
    if (server.hasArg("irrigationMinute1"))
    {
        int minute = server.arg("irrigationMinute1").toInt();
        if (minute >= 0 && minute <= 59)
        {
            config.irrigationMinute1 = minute;
        }
    }
    if (server.hasArg("irrigationSecond1"))
    {
        int second = server.arg("irrigationSecond1").toInt();
        if (second >= 0 && second <= 59)
        {
            config.irrigationSecond1 = second;
        }
    }
    if (server.hasArg("irrigationHour2"))
    {
        int hour = server.arg("irrigationHour2").toInt();
        if (hour >= 0 && hour <= 23)
        {
            config.irrigationHour2 = hour;
        }
    }
    if (server.hasArg("irrigationMinute2"))
    {
        int minute = server.arg("irrigationMinute2").toInt();
        if (minute >= 0 && minute <= 59)
        {
            config.irrigationMinute2 = minute;
        }
    }
    if (server.hasArg("irrigationSecond2"))
    {
        int second = server.arg("irrigationSecond2").toInt();
        if (second >= 0 && second <= 59)
        {
            config.irrigationSecond2 = second;
        }
    }

    if (saveConfig())
    {
        server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Settings saved\"}");
        serialPrintln("Settings updated successfully");
    }
    else
    {
        server.send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to save\"}");
    }
}

void handleRestart()
{
    server.send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
}

void handleLogs()
{
    JsonDocument doc;
    JsonArray logs = doc["logs"].to<JsonArray>();

    int start = (totalMessages < SERIAL_BUFFER_SIZE) ? 0 : serialBufferIndex;
    int count = min(totalMessages, SERIAL_BUFFER_SIZE);

    for (int i = 0; i < count; i++)
    {
        int idx = (start + i) % SERIAL_BUFFER_SIZE;
        JsonObject logEntry = logs.add<JsonObject>();
        logEntry["timestamp"] = serialBuffer[idx].timestamp;
        logEntry["message"] = serialBuffer[idx].message;
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

// ========== WIFI SETUP ==========
void setupWiFi()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Smart Nursery", "12345678");
    IPAddress ip = WiFi.softAPIP();

    char logBuffer[32];
    snprintf(logBuffer, sizeof(logBuffer), "AP IP: %d.%d.%d.%d",
             ip[0], ip[1], ip[2], ip[3]);
    serialPrintln(logBuffer);
}

// ========== DATA MANAGEMENT FUNCTIONS ==========
void initDataLog()
{
    if (!LittleFS.exists(DATA_LOG_FILE))
    {
        File file = LittleFS.open(DATA_LOG_FILE, "w");
        if (!file)
        {
            serialPrintln("Failed to create data log file");
            return;
        }
        file.println("DateTime,Temperature(C),Humidity(%),Lux,SoilMoisture1(%),SoilMoisture2(%),SoilMoisture3(%),SoilMoisture4(%),SoilMoisture5(%),SoilMoisture6(%),SoilMoisture7(%),SoilMoisture8(%),SoilMoisture9(%),SoilMoisture10(%)");
        file.close();
        serialPrintln("Data log file created with header");
    }
    else
    {
        serialPrintln("Data log file already exists");
    }
}

void saveDataRecord()
{
    if (!status.rtcInitialized)
    {
        serialPrintln("Cannot save data - RTC not initialized");
        return;
    }

    File file = LittleFS.open(DATA_LOG_FILE, "a");
    if (!file)
    {
        serialPrintln("Failed to open data log file");
        return;
    }

    DateTime now = rtc.now();
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d,%.2f,%.2f,%d,%d,%d,%d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second(),
             data.temperature,
             data.humidity,
             data.lux,
             data.soilMoisture1,
             data.soilMoisture2,
             data.soilMoisture3,
             data.soilMoisture4,
             data.soilMoisture5,
             data.soilMoisture6,
             data.soilMoisture7,
             data.soilMoisture8,
             data.soilMoisture9,
             data.soilMoisture10);

    file.println(buffer);
    file.close();

    // char logMsg[120];
    // snprintf(logMsg, sizeof(logMsg), "Data saved: T=%.2f°C H=%.2f%% SM1=%d%% SM2=%d%% SM3=%d%% SM4=%d%%",
    //          data.temperature, data.humidity, data.soilMoisture1, data.soilMoisture2, data.soilMoisture3, data.soilMoisture4);
    // serialPrintln(logMsg);

    char logMsg[120];
    snprintf(logMsg, sizeof(logMsg), "Data saved: T=%.2f°C H=%.2f%% SM1=%d%%",
             data.temperature, data.humidity, data.soilMoisture1);
    serialPrintln(logMsg);
}

void handleDataDownload()
{
    if (!LittleFS.exists(DATA_LOG_FILE))
    {
        server.send(404, "text/plain", "No data available");
        return;
    }

    File file = LittleFS.open(DATA_LOG_FILE, "r");
    if (!file)
    {
        server.send(500, "text/plain", "Failed to open data file");
        return;
    }

    server.sendHeader("Content-Type", "text/csv");
    server.sendHeader("Content-Disposition", "attachment; filename=sensor_data.csv");
    server.streamFile(file, "text/csv");
    file.close();

    serialPrintln("Data downloaded by user");
}

void handleDataDelete()
{
    if (LittleFS.exists(DATA_LOG_FILE))
    {
        LittleFS.remove(DATA_LOG_FILE);
        initDataLog(); // Recreate with header
        server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Data deleted\"}");
        serialPrintln("Data deleted and file reset");
    }
    else
    {
        server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"No data file found\"}");
    }
}

void handleDataInfo()
{
    JsonDocument doc;

    if (!LittleFS.exists(DATA_LOG_FILE))
    {
        doc["exists"] = false;
        doc["records"] = 0;
        doc["size"] = 0;
    }
    else
    {
        File file = LittleFS.open(DATA_LOG_FILE, "r");
        if (!file)
        {
            doc["exists"] = false;
            doc["records"] = 0;
            doc["size"] = 0;
        }
        else
        {
            doc["exists"] = true;
            doc["size"] = file.size();

            int lineCount = 0;
            while (file.available())
            {
                String line = file.readStringUntil('\n');
                lineCount++;
            }
            doc["records"] = lineCount > 0 ? lineCount - 1 : 0; // -1 for header
            file.close();
        }
    }

    // Calculate next log time
    if (data.lastDataLog > 0)
    {
        unsigned long timeSinceLog = millis() - data.lastDataLog;
        if (timeSinceLog < config.dataLogInterval)
        {
            doc["nextLogSeconds"] = (config.dataLogInterval - timeSinceLog) / 1000;
        }
        else
        {
            doc["nextLogSeconds"] = 0;
        }
    }
    else
    {
        doc["nextLogSeconds"] = config.dataLogInterval / 1000; // 1800 seconds = 30 minutes
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void setupWebServer()
{
    server.enableCORS(true);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/config", HTTP_GET, handleConfig);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/logs", HTTP_GET, handleLogs);
    server.on("/time", HTTP_GET, handleTime);
    server.on("/datetime", HTTP_GET, handleDateTime);
    // server.on("/serial", HTTP_GET, handleSerial);

    // ← TAMBAH 3 BARIS INI:
    server.on("/data/download", HTTP_GET, handleDataDownload);
    server.on("/data/delete", HTTP_POST, handleDataDelete);
    server.on("/data/info", HTTP_GET, handleDataInfo);

    server.begin();
    serialPrintln("Web server started");
}

// void handleSerial()
// {
//     String logs;
//     logs.reserve(SERIAL_BUFFER_SIZE * 100); // Pre-allocate space

//     int start = (serialBufferIndex - totalMessages + SERIAL_BUFFER_SIZE) % SERIAL_BUFFER_SIZE;
//     for (int i = 0; i < totalMessages; i++)
//     {
//         int index = (start + i) % SERIAL_BUFFER_SIZE;
//         logs += String(serialBuffer[index].timestamp);
//         logs += ": ";
//         logs += serialBuffer[index].message;
//         logs += "\n";
//     }

//     server.send(200, "text/plain", logs);
// }

void handleDateTime()
{
    if (status.rtcInitialized)
    {
        DateTime now = rtc.now();
        String json = "{";
        json += "\"date\":\"" + String(now.timestamp(DateTime::TIMESTAMP_DATE)) + "\",";
        json += "\"time\":\"" + String(now.timestamp(DateTime::TIMESTAMP_TIME)) + "\"";
        json += "}";

        server.send(200, "application/json", json);
    }
    else
    {
        server.send(500, "application/json", "{\"error\":\"RTC not initialized\"}");
    }
}

void handleTime()
{
    if (status.rtcInitialized)
    {
        DateTime now = rtc.now();
        String json = "{";
        json += "\"time\":\"" + String(now.timestamp(DateTime::TIMESTAMP_TIME)) + "\",";
        json += "}";

        server.send(200, "application/json", json);
    }
    else
    {
        server.send(500, "application/json", "{\"error\":\"RTC not initialized\"}");
    }
}

// ========== SETUP ==========
void setup()
{
    Serial.begin(115200);
    delay(1000);

    serialPrintln("Starting Smart Nursery System...");

    analogReadResolution(12);                              // Atur resolusi ADC menjadi 12 bit (0-4095)
    analogSetPinAttenuation(SOIL1_MOISTURE_PIN, ADC_11db); // Atur attenuasi untuk rentang pengukuran yang lebih luas (0-3.3V)
    analogSetPinAttenuation(SOIL2_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL3_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL4_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL5_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL6_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL7_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL8_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL9_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL10_MOISTURE_PIN, ADC_11db);

    pinMode(SOIL1_MOISTURE_PIN, INPUT);
    pinMode(SOIL2_MOISTURE_PIN, INPUT);
    pinMode(SOIL3_MOISTURE_PIN, INPUT);
    pinMode(SOIL4_MOISTURE_PIN, INPUT);
    pinMode(SOIL5_MOISTURE_PIN, INPUT);
    pinMode(SOIL6_MOISTURE_PIN, INPUT);
    pinMode(SOIL7_MOISTURE_PIN, INPUT);
    pinMode(SOIL8_MOISTURE_PIN, INPUT);
    pinMode(SOIL9_MOISTURE_PIN, INPUT);
    pinMode(SOIL10_MOISTURE_PIN, INPUT);

    // Initialize sensors
    dht.begin();
    readDHT22();
    readSoilMoisture();
    initLuxMeter();
    readLuxMeter();

    initRTC();
    initWatchdog();

    // Initialize filesystem
    if (!setupLittleFS())
    {
        serialPrintln("LittleFS setup failed. Restarting...");
        delay(2000);
        ESP.restart();
    }

    // Load configuration
    if (!loadConfig())
    {
        serialPrintln("Using default configuration");
        createDefaultConfig();
        loadConfig();
    }
    validateMeasurementInterval();

    // ← TAMBAH 2 BARIS INI:
    initDataLog();
    serialPrintln("Data logging initialized");

    // Initialize pump control
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);

    // Setup network and web server
    setupWiFi();
    setupWebServer();

    serialPrintln("Setup complete");
}

// ========== MAIN LOOP ==========
void loop()
{
    server.handleClient();
    resetWatchdog();

    unsigned long now = millis();
    if (now - data.lastMeasurement >= config.measurementInterval)
    {
        data.lastMeasurement = now;

        readDHT22();
        readSoilMoisture();
        readLuxMeter();
        // readSoilMoistureSensor();

        if (now - data.lastDataLog >= config.dataLogInterval)
        {
            data.lastDataLog = now;
            saveDataRecord();
        }

        if (status.rtcInitialized)
        {
            DateTime currentTime = rtc.now();
            resetDailyIrrigation(currentTime);
            controlPump(currentTime);

            char logBuffer[64];
            snprintf(logBuffer, sizeof(logBuffer),
                     "Time: %02d-%02d-%04d %02d:%02d:%02d",
                     currentTime.day(), currentTime.month(), currentTime.year(),
                     currentTime.hour(), currentTime.minute(), currentTime.second());
            serialPrintln(logBuffer);
        }
    }
}
