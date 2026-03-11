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
// Soil 1-4 are read by the slave ESP32 via UART - no local pins needed
#define SOIL5_MOISTURE_PIN 32
#define SOIL6_MOISTURE_PIN 33
#define SOIL7_MOISTURE_PIN 34
#define SOIL8_MOISTURE_PIN 35
#define SOIL9_MOISTURE_PIN 36
#define SOIL10_MOISTURE_PIN 39
#define PUMP_PIN 19
#define SOLENOID_PIN 18

// ========== SLAVE ESP UART ==========//
// Wiring: Main GPIO16 (RX2) <--- Slave GPIO17 (TX2)
//         Main GPIO17 (TX2) ---> Slave GPIO16 (RX2)
//         GND --- GND
#define SLAVE_SERIAL Serial2
#define SLAVE_BAUD 115200
#define SLAVE_RX_PIN 16
#define SLAVE_TX_PIN 17
#define SLAVE_TIMEOUT 5000UL // ms - mark soil 1 - 4 if no data received

// ========== RELAY CONTROL ==========
#define PUMP_ON LOW
#define PUMP_OFF HIGH
#define SOLENOID_OPEN LOW
#define SOLENOID_CLOSED HIGH

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

// ========== SLAVE UART STATE ==========//
unsigned long lastSlaveReceived = 0; // timestamp of last valid packet from slave
String slaveBuffer = "";             // accumulates incoming chars untul '\n'

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

// ========== WATERING MODE ==========
enum WateringMode
{
    MODE_SCHEDULE = 0, // only run on schedule (07:00 & 16:00)
    MODE_MOISTURE = 1, // only run based on soil moisture threshold
    MODE_BOTH = 2      // moisture first, schedule as fallback
};

// ========== CONFIGURATION ==========
struct Config
{
    // Moisture threshold (%) for automatic watering mode
    int threshold = 30;
    // Active watering strategy (see WateringMode)
    int wateringMode = MODE_BOTH;
    int dry = 2662;
    int wet = 1269;
    int pumpDuration = 60000;
    int measurementInterval = 60000;
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

// Control source: who is actively controlling the pump (priority hierarchy)
enum ControlSource
{
    CONTROL_NONE = 0,
    MANUAL_OVERRIDE,    // Manual ON/OFF button (highest)
    SOIL_AUTOMATION,    // Average soil moisture below threshold (Settings)
    SCHEDULE_AUTOMATION // Scheduled irrigation (lowest)
};

// Debounce: require avg < threshold for N consecutive checks before starting (prevents rapid cycling)
#define MOISTURE_DEBOUNCE_COUNT 5

struct PumpControl
{
    PumpState state = PUMP_IDLE;
    unsigned long startTime = 0;
    unsigned long cooldownStart = 0;
    bool irrigationDone[2] = {false, false};
    bool manualOverride = false;
    ControlSource controlSource = CONTROL_NONE;
    uint8_t moistureStableCount = 0; // debounce for moisture-based start
    int pumpRunsToday = 0;           // jumlah penyiraman hari ini, reset tiap ganti hari
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

void pollSlaveUART();         // untuk memeriksa data dari slave ESP32 melalui UART, memperbarui kelembaban tanah 1-4, dan menandai sensor sebagai tidak valid jika timeout tercapai
void readDHT22();             // untuk membaca data suhu dan kelembapan dari sensor DHT22, serta menyimpan hasilnya ke struktur SensorData
int readADC(int pin);         // untuk membaca nilai mentah dari pin ADC, melakukan pembacaan berulang untuk stabilisasi, dan mengembalikan rata-rata
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
void handlePumpControl();  // untuk menangani permintaan HTTP POST ke rute "/pump", mengontrol pompa ON/OFF secara manual
void handleLogs();         // untuk menangani permintaan HTTP ke rute "/logs", biasanya digunakan untuk mengirimkan log pesan yang disimpan dalam buffer sebagai respons dalam format JSON
void handleLogsClear();    // untuk mengosongkan buffer log (POST /logs/clear)
void handleSetDateTime();  // untuk menangani permintaan HTTP ke rute "/setdatetime", biasanya digunakan untuk menerima data tanggal dan waktu baru dari klien, memperbarui RTC dengan nilai tersebut, dan mengirimkan respons status kepada klien
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
    unsigned long ms = millis();
    Serial.print("[");
    Serial.print(ms);
    Serial.print("] ");
    Serial.println(message);

    serialBuffer[serialBufferIndex].timestamp = ms;
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
void initRTC()
{
    Wire.begin();
    if (!rtc.begin())
    {
        serialPrintln("RTC not found");
        status.rtcInitialized = false;
    }
    else
    {
        status.rtcInitialized = true;
        if (rtc.lostPower())
        {
            serialPrintln("RTC lost power, setting time!");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        serialPrintln("RTC initialized successfully");
    }
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
    // Default moisture threshold (%) for automatic watering
    doc["threshold"] = 30;
    // Default watering mode: 2 = both (moisture first, then schedule)
    doc["wateringMode"] = 2;
    doc["dry"] = 2662;
    doc["wet"] = 1269;
    doc["pumpDuration"] = 60000;
    doc["measurementInterval"] = 60000;
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
    config.wateringMode = doc["wateringMode"] | config.wateringMode;
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
    doc["wateringMode"] = config.wateringMode;
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
        // serialPrintln("DHT22 sensor read successfully");
        logToFile(logBuffer);
    }
    else
    {
        serialPrintln("Failed to read DHT22 sensor");
    }
}

// int readSoilPercent(int pin)
// {
//     int raw = analogRead(pin);
//     raw = constrain(raw, config.dry, config.wet); // sesuaikan hasil kalibrasi
//     return map(raw, config.wet, config.dry, 0, 100);
// }

int readADC(int pin)
{
    long sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += analogRead(pin);
        delay(10);
    }
    return sum / 10;
}

int readSoilPercent(int pin)
{
    int raw = readADC(pin);
    raw = constrain(raw, config.wet, config.dry); // sesuaikan hasil kalibrasi
    return map(raw, config.dry, config.wet, 0, 50);
}

// Called every loop() -reads any incoming bytes from slave and parses complete lines
// void pollSlaveUART()
// {
//     static unsigned long lastDataTime = 0;
//     while (SLAVE_SERIAL.available())
//     {
//         String line = SLAVE_SERIAL.readStringUntil('\n');
//         line.trim();
//         if (line.startsWith("SOIL:"))
//         {
//             // Expected format: SOIL:val1,val2,val3,val4
//             int vals[4] = {0};
//             sscanf(line.c_str(), "SOIL:%d,%d,%d,%d", &vals[0], &vals[1], &vals[2], &vals[3]);
//             data.soilMoisture1 = vals[0];
//             data.soilMoisture2 = vals[1];
//             data.soilMoisture3 = vals[2];
//             data.soilMoisture4 = vals[3];
//             lastDataTime = millis();
//         }
//     }

//     // If no data received for a while, mark soil 1-4 as invalid
//     if (millis() - lastDataTime > SLAVE_TIMEOUT)
//     {
//         data.soilMoisture1 = -1;
//         data.soilMoisture2 = -1;
//         data.soilMoisture3 = -1;
//         data.soilMoisture4 = -1;
//     }
// }

void pollSlaveUART()
{
    while (SLAVE_SERIAL.available())
    {
        char c = (char)SLAVE_SERIAL.read();
        if (c == '\n')
        {
            // Complete line received - parse JSON
            // Expected format: {"s1":42, "s2":38, "s3":55, "s4":61}
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, slaveBuffer);
            if (!error)
            {
                data.soilMoisture1 = doc["s1"] | data.soilMoisture1;
                data.soilMoisture2 = doc["s2"] | data.soilMoisture2;
                data.soilMoisture3 = doc["s3"] | data.soilMoisture3;
                data.soilMoisture4 = doc["s4"] | data.soilMoisture4;
                lastSlaveReceived = millis();
            }
            else
            {
                serialPrintln("Slave parse error");
            }
            slaveBuffer = "";
        }
        else
        {
            if (slaveBuffer.length() < 128) // guard againts runway buffer
                slaveBuffer += c;
        }
    }

    // If slave has been silent too long, flag soil 1-4 as unavailable
    if (lastSlaveReceived > 0 && millis() - lastSlaveReceived > SLAVE_TIMEOUT) 
    {
        data.soilMoisture1 = -1;
        data.soilMoisture2 = -1;
        data.soilMoisture3 = -1;
        data.soilMoisture4 = -1;
        serialPrintln("WARNING: Slave ESP timeout - soil 1-4 unavailable");
        lastSlaveReceived = millis(); // suppress repeat log for another SLAVE_TIMEOUT ms
    }
}


void readSoilMoisture()
{
    // Soil 1-4: received from slave ESP via UART (see pollSlaveUART)
    // Soil 5-10:  read locally
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
        serialPrintln("BH1750 not available");
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
        pumpControl.pumpRunsToday = 0;

        char logBuffer[80];
        snprintf(logBuffer, sizeof(logBuffer),
                 "Schedule reset (%02d:%02d:%02d & %02d:%02d:%02d)",
                 config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1,
                 config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
        serialPrintln(logBuffer);
        logToFile(logBuffer);
    }
}

// Returns average soil moisture (0-100) or -1 if no valid sensor data
int getAverageSoilMoisture()
{
    int vals[10] = {
        data.soilMoisture1, data.soilMoisture2, data.soilMoisture3, data.soilMoisture4,
        data.soilMoisture5, data.soilMoisture6, data.soilMoisture7, data.soilMoisture8,
        data.soilMoisture9, data.soilMoisture10};
    int sum = 0, count = 0;
    for (int i = 0; i < 10; i++)
    {
        if (vals[i] >= 0 && vals[i] <= 100)
        {
            sum += vals[i];
            count++;
        }
    }
    if (count == 0)
        return -1;
    return sum / count;
}

void startPump(int scheduleIndex, int hour, int minute, int second)
{
    pumpControl.state = PUMP_RUNNING;
    pumpControl.startTime = millis();
    pumpControl.pumpRunsToday++;

    digitalWrite(SOLENOID_PIN, SOLENOID_OPEN);
    delay(500);

    digitalWrite(PUMP_PIN, PUMP_ON);

    if (scheduleIndex >= 0 && scheduleIndex <= 1)
        pumpControl.irrigationDone[scheduleIndex] = true;

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer),
             "Pump START (%02d:%02d:%02d)",
             hour, minute, second);

    serialPrintln(logBuffer);
    logToFile(logBuffer);
}

void controlPump(DateTime &currentTime)
{
    int avgSoil = getAverageSoilMoisture();

    // ==========================
    // STATE MACHINE: RUNNING -> duration expires -> COOLDOWN -> IDLE
    // Pump turns OFF after configured duration; cooldown prevents overlapping cycles.
    // ==========================

    switch (pumpControl.state)
    {
    case PUMP_IDLE:
        break;

    case PUMP_RUNNING:
        if (millis() - pumpControl.startTime >= config.pumpDuration)
        {
            digitalWrite(PUMP_PIN, PUMP_OFF);
            delay(500);
            digitalWrite(SOLENOID_PIN, SOLENOID_CLOSED);
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
        digitalWrite(PUMP_PIN, PUMP_OFF);
        break;
    }

    // ==========================
    // PRIORITY 1: Manual override — skip all automation
    // ==========================
    if (pumpControl.manualOverride)
        return;

    if (pumpControl.state != PUMP_IDLE)
        return;

    // ==========================
    // PRIORITY 2: Moisture-based (Average Soil Moisture < threshold from Settings)
    // Uses config.threshold (%) and config.pumpDuration (ms). Cooldown between activations.
    // ==========================
    bool allowMoisture =
        (config.wateringMode == MODE_MOISTURE || config.wateringMode == MODE_BOTH);
    bool allowSchedule =
        (config.wateringMode == MODE_SCHEDULE || config.wateringMode == MODE_BOTH);

    if (allowMoisture && avgSoil >= 0 && avgSoil < config.threshold)
    {
        pumpControl.moistureStableCount++;
        if (pumpControl.moistureStableCount >= MOISTURE_DEBOUNCE_COUNT)
        {
            pumpControl.moistureStableCount = 0;
            startPump(-1, 0, 0, 0);
            pumpControl.controlSource = SOIL_AUTOMATION;
            char buf[80];
            snprintf(buf, sizeof(buf), "Pump START (Moisture Auto, avg %d%% < threshold %d%%)", avgSoil, config.threshold);
            serialPrintln(buf);
            logToFile(buf);
        }
        return;
    }
    pumpControl.moistureStableCount = 0;

    // ==========================
    // PRIORITY 3: Scheduled irrigation (only if manual off and moisture not triggering)
    // ==========================
    if (!allowSchedule)
        return;

    int currentHour = currentTime.hour();
    int currentMinute = currentTime.minute();
    int currentSecond = currentTime.second();

    if (currentHour == config.irrigationHour1 &&
        currentMinute == config.irrigationMinute1 &&
        currentSecond >= config.irrigationSecond1 &&
        !pumpControl.irrigationDone[0])
    {
        startPump(0, config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1);
        pumpControl.controlSource = SCHEDULE_AUTOMATION;
        serialPrintln("Pump START (Schedule)");
        logToFile("Pump started by schedule");
        return;
    }

    if (currentHour == config.irrigationHour2 &&
        currentMinute == config.irrigationMinute2 &&
        currentSecond >= config.irrigationSecond2 &&
        !pumpControl.irrigationDone[1])
    {
        startPump(1, config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
        pumpControl.controlSource = SCHEDULE_AUTOMATION;
        serialPrintln("Pump START (Schedule)");
        logToFile("Pump started by schedule");
        return;
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
    server.sendHeader("Cache-Control", "private, max-age=120");
    server.streamFile(file, "text/html");
    file.close();
}

void handleStatus()
{
    JsonDocument doc;
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["lux"] = data.lux;
    doc["soilMoisture1"] = data.soilMoisture1;
    doc["soilMoisture2"] = data.soilMoisture2;
    doc["soilMoisture3"] = data.soilMoisture3;
    doc["soilMoisture4"] = data.soilMoisture4;
    doc["soilMoisture5"] = data.soilMoisture5;
    doc["soilMoisture6"] = data.soilMoisture6;
    doc["soilMoisture7"] = data.soilMoisture7;
    doc["soilMoisture8"] = data.soilMoisture8;
    doc["soilMoisture9"] = data.soilMoisture9;
    doc["soilMoisture10"] = data.soilMoisture10;
    doc["pumpState"] = pumpControl.state;
    doc["controlSource"] = (int)pumpControl.controlSource;
    doc["manualOverride"] = pumpControl.manualOverride;
    doc["threshold"] = config.threshold;
    doc["wateringMode"] = config.wateringMode;
    doc["irrigationHour1"] = config.irrigationHour1;
    doc["irrigationMinute1"] = config.irrigationMinute1;
    doc["irrigationSecond1"] = config.irrigationSecond1;
    doc["irrigationHour2"] = config.irrigationHour2;
    doc["irrigationMinute2"] = config.irrigationMinute2;
    doc["irrigationSecond2"] = config.irrigationSecond2;

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
    doc["wateringMode"] = config.wateringMode;
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
    // Simpan nilai lama untuk bandingkan — hanya log yang benar-benar berubah
    int oldThreshold = config.threshold;
    int oldDry = config.dry;
    int oldWet = config.wet;
    unsigned long oldPumpDuration = config.pumpDuration;
    unsigned long oldMeasurementInterval = config.measurementInterval;
    unsigned long oldDataLogInterval = config.dataLogInterval;
    int oldWateringMode = config.wateringMode;
    int oldH1 = config.irrigationHour1, oldM1 = config.irrigationMinute1, oldS1 = config.irrigationSecond1;
    int oldH2 = config.irrigationHour2, oldM2 = config.irrigationMinute2, oldS2 = config.irrigationSecond2;

    bool logSchedule = false;
    bool logModeWatering = false;
    bool logThreshold = false;
    bool logPumpDuration = false;
    bool logMeasurementInterval = false;
    bool logDataLogInterval = false;
    bool logCalibration = false;

    if (server.hasArg("threshold"))
    {
        int v = server.arg("threshold").toInt();
        if (v != oldThreshold)
            logThreshold = true;
        config.threshold = v;
    }
    if (server.hasArg("dry"))
    {
        int v = server.arg("dry").toInt();
        if (v != oldDry)
            logCalibration = true;
        config.dry = v;
    }
    if (server.hasArg("wet"))
    {
        int v = server.arg("wet").toInt();
        if (v != oldWet)
            logCalibration = true;
        config.wet = v;
    }
    if (server.hasArg("pumpDuration"))
    {
        unsigned long v = (unsigned long)server.arg("pumpDuration").toInt();
        if (v != oldPumpDuration)
            logPumpDuration = true;
        config.pumpDuration = v;
    }
    if (server.hasArg("measurementInterval"))
    {
        unsigned long seconds = server.arg("measurementInterval").toInt();
        unsigned long v = max(MINIMUM_INTERVAL, seconds * 1000UL);
        if (v != oldMeasurementInterval)
            logMeasurementInterval = true;
        config.measurementInterval = v;
    }
    if (server.hasArg("dataLogInterval"))
    {
        unsigned long seconds = server.arg("dataLogInterval").toInt();
        unsigned long v = seconds * 1000UL;
        if (v != oldDataLogInterval)
            logDataLogInterval = true;
        config.dataLogInterval = v;
    }
    if (server.hasArg("wateringMode"))
    {
        int mode = server.arg("wateringMode").toInt();
        if (mode >= MODE_SCHEDULE && mode <= MODE_BOTH)
        {
            if (mode != oldWateringMode)
                logModeWatering = true;
            config.wateringMode = mode;
        }
    }
    if (server.hasArg("irrigationHour1"))
    {
        int hour = server.arg("irrigationHour1").toInt();
        if (hour >= 0 && hour <= 23)
            config.irrigationHour1 = hour;
    }
    if (server.hasArg("irrigationMinute1"))
    {
        int minute = server.arg("irrigationMinute1").toInt();
        if (minute >= 0 && minute <= 59)
            config.irrigationMinute1 = minute;
    }
    if (server.hasArg("irrigationSecond1"))
    {
        int second = server.arg("irrigationSecond1").toInt();
        if (second >= 0 && second <= 59)
            config.irrigationSecond1 = second;
    }
    if (server.hasArg("irrigationHour2"))
    {
        int hour = server.arg("irrigationHour2").toInt();
        if (hour >= 0 && hour <= 23)
            config.irrigationHour2 = hour;
    }
    if (server.hasArg("irrigationMinute2"))
    {
        int minute = server.arg("irrigationMinute2").toInt();
        if (minute >= 0 && minute <= 59)
            config.irrigationMinute2 = minute;
    }
    if (server.hasArg("irrigationSecond2"))
    {
        int second = server.arg("irrigationSecond2").toInt();
        if (second >= 0 && second <= 59)
            config.irrigationSecond2 = second;
    }

    if (config.irrigationHour1 != oldH1 || config.irrigationMinute1 != oldM1 || config.irrigationSecond1 != oldS1 ||
        config.irrigationHour2 != oldH2 || config.irrigationMinute2 != oldM2 || config.irrigationSecond2 != oldS2)
        logSchedule = true;

    if (saveConfig())
    {
        server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Settings saved\"}");

        char logBuf[128];
        if (logSchedule)
        {
            snprintf(logBuf, sizeof(logBuf),
                     "Update schedule ('%02d:%02d:%02d & %02d:%02d:%02d')",
                     config.irrigationHour1, config.irrigationMinute1, config.irrigationSecond1,
                     config.irrigationHour2, config.irrigationMinute2, config.irrigationSecond2);
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logModeWatering)
        {
            const char *modeStr = config.wateringMode == MODE_SCHEDULE ? "Schedule" : config.wateringMode == MODE_MOISTURE ? "Moisture"
                                                                                                                           : "Both";
            snprintf(logBuf, sizeof(logBuf), "Update mode watering ('%s')", modeStr);
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logThreshold)
        {
            snprintf(logBuf, sizeof(logBuf), "Update threshold ('%d%%')", config.threshold);
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logPumpDuration)
        {
            snprintf(logBuf, sizeof(logBuf), "Update pump duration ('%lus')",
                     (unsigned long)(config.pumpDuration / 1000));
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logMeasurementInterval)
        {
            snprintf(logBuf, sizeof(logBuf), "Update measurement interval ('%lus')",
                     (unsigned long)(config.measurementInterval / 1000));
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logDataLogInterval)
        {
            snprintf(logBuf, sizeof(logBuf), "Update data log interval ('%lus')",
                     (unsigned long)(config.dataLogInterval / 1000));
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (logCalibration)
        {
            snprintf(logBuf, sizeof(logBuf), "Update calibration ('dry %d, wet %d')",
                     config.dry, config.wet);
            serialPrintln(logBuf);
            logToFile(logBuf);
        }
        if (!logSchedule && !logModeWatering && !logThreshold && !logPumpDuration &&
            !logMeasurementInterval && !logDataLogInterval && !logCalibration)
        {
            serialPrintln("Settings saved (no changes)");
        }
    }
    else
    {
        server.send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to save\"}");
        serialPrintln("Settings save failed");
    }
}

void handlePumpControl()
{
    if (!server.hasArg("state"))
    {
        server.send(400, "application/json", "{\"status\":\"error\",\"error\":\"Missing state\"}");
        return;
    }
    String state = server.arg("state");
    state.toLowerCase();

    if (state == "on")
    {
        pumpControl.manualOverride = true;
        pumpControl.controlSource = MANUAL_OVERRIDE;
        pumpControl.state = PUMP_RUNNING;
        pumpControl.startTime = millis();
        pumpControl.pumpRunsToday++;
        digitalWrite(SOLENOID_PIN, SOLENOID_OPEN);
        delay(500);
        digitalWrite(PUMP_PIN, PUMP_ON);
        serialPrintln("Pump ON (manual)");
        logToFile("Pump ON (manual)");
        server.send(200, "application/json", "{\"status\":\"success\",\"pump\":\"on\"}");
    }
    else if (state == "off")
    {
        pumpControl.manualOverride = true;
        pumpControl.controlSource = MANUAL_OVERRIDE;
        pumpControl.state = PUMP_IDLE;
        digitalWrite(PUMP_PIN, PUMP_OFF);
        digitalWrite(SOLENOID_PIN, SOLENOID_CLOSED);
        serialPrintln("Pump OFF (manual)");
        logToFile("Pump OFF (manual)");
        server.send(200, "application/json", "{\"status\":\"success\",\"pump\":\"off\"}");
    }
    else if (state == "auto")
    {
        pumpControl.manualOverride = false;
        pumpControl.controlSource = CONTROL_NONE;
        pumpControl.state = PUMP_IDLE;

        serialPrintln("Pump AUTO mode");
        logToFile("Pump AUTO mode");

        server.send(200, "application/json", "{\"status\":\"success\",\"mode\":\"auto\"}");
    }
    else
    {
        server.send(400, "application/json", "{\"status\":\"error\",\"error\":\"Invalid state\"}");
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

void handleLogsClear()
{
    serialBufferIndex = 0;
    totalMessages = 0;
    serialPrintln("Log buffer cleared");
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Log cleared\"}");
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
        file.println("DateTime,Temperature(C),Humidity(%),Lux,SoilMoisture1(%),SoilMoisture2(%),SoilMoisture3(%),SoilMoisture4(%),SoilMoisture5(%),SoilMoisture6(%),SoilMoisture7(%),SoilMoisture8(%),SoilMoisture9(%),SoilMoisture10(%),WateringCountToday");
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
    int avgSoil = getAverageSoilMoisture();
    char buffer[220];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
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
             data.soilMoisture10,
             pumpControl.pumpRunsToday);

    file.println(buffer);
    file.close();
    char logMsg[120];
    snprintf(logMsg, sizeof(logMsg), "Data saved: Temperature=%.2f°C Humidity=%.2f%% Lux=%.2f AvgSoil=%d%%",
             data.temperature, data.humidity, data.lux, avgSoil);
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

void handleSetDateTime()
{
    if (!status.rtcInitialized)
    {
        server.send(500, "application/json", "{\"error\":\"RTC not initialized\"}");
        return;
    }

    // Expects POST args: year, month, day, hour, minute, second
    if (!server.hasArg("year") || !server.hasArg("month") || !server.hasArg("day") ||
        !server.hasArg("hour") || !server.hasArg("minute") || !server.hasArg("second"))
    {
        server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        return;
    }

    int y = server.arg("year").toInt();
    int mo = server.arg("month").toInt();
    int d = server.arg("day").toInt();
    int h = server.arg("hour").toInt();
    int mi = server.arg("minute").toInt();
    int s = server.arg("second").toInt();

    if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59)
    {
        server.send(400, "application/json", "{\"error\":\"Invalid date/time values\"}");
        return;
    }

    rtc.adjust(DateTime(y, mo, d, h, mi, s));

    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer),
             "RTC updated: %04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
    serialPrintln(logBuffer);

    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"RTC updated\"}");
}

void setupWebServer()
{
    server.enableCORS(true);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/config", HTTP_GET, handleConfig);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/pump", HTTP_POST, handlePumpControl);
    server.on("/logs", HTTP_GET, handleLogs);
    server.on("/logs/clear", HTTP_POST, handleLogsClear);
    server.on("/time", HTTP_GET, handleTime);
    // server.on("/datetime", HTTP_GET, handleDateTime);
    server.on("/datetime", HTTP_ANY, []()
              {
        if (server.method() == HTTP_POST)
            handleSetDateTime();
        else
            handleDateTime(); });

    // server.on("/serial", HTTP_GET, handleSerial);

    // ← TAMBAH 3 BARIS INI:
    server.on("/data/download", HTTP_GET, handleDataDownload);
    server.on("/data/delete", HTTP_POST, handleDataDelete);
    server.on("/data/info", HTTP_GET, handleDataInfo);

    server.begin();
    serialPrintln("Web server started");
}

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
        json += "\"time\":\"" + String(now.timestamp(DateTime::TIMESTAMP_TIME)) + "\"";
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
    // Soil 1-4 handle by slave ESP via UART - no local adc setup needed
    analogSetPinAttenuation(SOIL5_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL6_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL7_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL8_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL9_MOISTURE_PIN, ADC_11db);
    analogSetPinAttenuation(SOIL10_MOISTURE_PIN, ADC_11db);

    // Soil 1-4 handled by slave ESP via UART - no local pinMode needed
    pinMode(SOIL5_MOISTURE_PIN, INPUT);
    pinMode(SOIL6_MOISTURE_PIN, INPUT);
    pinMode(SOIL7_MOISTURE_PIN, INPUT);
    pinMode(SOIL8_MOISTURE_PIN, INPUT);
    pinMode(SOIL9_MOISTURE_PIN, INPUT);
    pinMode(SOIL10_MOISTURE_PIN, INPUT);

    // Initialize UART link to slave ESP (soil moisture 1-4)
    SLAVE_SERIAL.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);
    serialPrintln("Slave UART initialized (Serial2)");

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
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, PUMP_OFF);
    digitalWrite(SOLENOID_PIN, SOLENOID_CLOSED);

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
    pollSlaveUART(); // continuosly drain incoming UART bytes from slave ESP

    unsigned long now = millis();
    if (now - data.lastMeasurement >= config.measurementInterval)
    {
        data.lastMeasurement = now;

        readDHT22();
        readSoilMoisture();
        readLuxMeter();

        if (now - data.lastDataLog >= config.dataLogInterval)
        {
            data.lastDataLog = now;
            saveDataRecord();
        }

        // if (status.rtcInitialized)
        // {
        //     DateTime currentTime = rtc.now();
        //     // resetDailyIrrigation(currentTime);
        //     // controlPump(currentTime);

        //     char logBuffer[64];
        //     snprintf(logBuffer, sizeof(logBuffer),
        //              "Time: %02d-%02d-%04d %02d:%02d:%02d",
        //              currentTime.day(), currentTime.month(), currentTime.year(),
        //              currentTime.hour(), currentTime.minute(), currentTime.second());
        //     // serialPrintln(logBuffer);
        // }
    }

    // Pump control runs every second — not gated by measurementInterval
    static unsigned long lastPumpCheck = 0;
    if (status.rtcInitialized && millis() - lastPumpCheck >= 1000)
    {
        lastPumpCheck = millis();
        DateTime currentTime = rtc.now();
        resetDailyIrrigation(currentTime);
        controlPump(currentTime);
    }
}
