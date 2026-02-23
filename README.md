# ESP32 Sensor Monitoring Dashboard

Program monitoring sensor DHT22 (Suhu & Kelembaban Udara) dan Soil Moisture Sensor menggunakan ESP32 dengan web dashboard yang responsif dan modern.

## 📋 Fitur

- ✅ Monitoring suhu dari DHT22
- ✅ Monitoring kelembaban udara dari DHT22
- ✅ Monitoring kelembaban tanah dari Soil Moisture Sensor
- ✅ Web dashboard dengan UI modern dan responsif
- ✅ Auto-refresh data setiap 2 detik
- ✅ Progress bar untuk kelembaban tanah
- ✅ Tampilan real-time tanpa perlu refresh manual

## 🔌 Koneksi Hardware

### DHT22 Sensor
- **VCC** → 3.3V ESP32
- **GND** → GND ESP32
- **DATA** → GPIO 4

### Soil Moisture Sensor
- **VCC** → 3.3V atau 5V ESP32
- **GND** → GND ESP32
- **A0/OUT** → GPIO 34 (ADC1_CH6)

**Catatan:** GPIO 34 adalah input-only pin yang cocok untuk ADC.

## ⚙️ Konfigurasi

1. **Edit WiFi Credentials**
   
   Buka `src/main.cpp` dan ubah:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
   
   Ganti dengan SSID dan password WiFi Anda.

2. **Sesuaikan Pin (Opsional)**
   
   Jika pin berbeda, ubah di bagian:
   ```cpp
   #define DHTPIN 4          // Pin untuk DHT22
   #define SOIL_MOISTURE_PIN 34  // Pin untuk Soil Moisture
   ```

3. **Kalibrasi Soil Moisture Sensor (Opsional)**
   
   Jika sensor Anda memiliki karakteristik berbeda, sesuaikan mapping di fungsi `readSensors()`:
   ```cpp
   soilMoisture = map(soilMoisture, 0, 4095, 100, 0);
   ```
   
   - Nilai pertama (0): nilai ADC minimum (tanah sangat basah)
   - Nilai kedua (4095): nilai ADC maksimum (tanah sangat kering)
   - Nilai ketiga (100): persentase maksimum
   - Nilai keempat (0): persentase minimum

## 🚀 Cara Menggunakan

1. **Install PlatformIO** (jika belum)
   - Download dari https://platformio.org/
   - Atau install extension di VS Code

2. **Upload Program**
   ```bash
   pio run --target upload
   ```

3. **Monitor Serial**
   ```bash
   pio device monitor
   ```
   
   Atau gunakan Serial Monitor di PlatformIO untuk melihat:
   - Status koneksi WiFi
   - IP Address ESP32
   - Data sensor real-time

4. **Akses Dashboard**
   
   Setelah ESP32 terhubung ke WiFi, buka browser dan akses:
   ```
   http://[IP_ADDRESS_ESP32]
   ```
   
   IP address akan ditampilkan di Serial Monitor.

## 📊 Dashboard Features

- **Card Suhu**: Menampilkan suhu dalam °C dengan border merah
- **Card Kelembaban Udara**: Menampilkan kelembaban dalam % dengan border cyan
- **Card Kelembaban Tanah**: Menampilkan kelembaban tanah dalam % dengan progress bar dan border biru
- **Auto-refresh**: Data diperbarui otomatis setiap 2 detik
- **Manual Refresh**: Tombol untuk refresh manual
- **Responsive Design**: Tampilan optimal di desktop dan mobile

## 🔧 Troubleshooting

### Sensor tidak terbaca
- Pastikan koneksi hardware benar
- Cek apakah pin yang digunakan sesuai
- Pastikan sensor mendapat power yang cukup

### WiFi tidak terhubung
- Pastikan SSID dan password benar
- Pastikan ESP32 dalam jangkauan WiFi
- Cek di Serial Monitor untuk pesan error

### Dashboard tidak muncul
- Pastikan ESP32 sudah terhubung ke WiFi
- Cek IP address di Serial Monitor
- Pastikan browser dan ESP32 dalam jaringan yang sama

### Nilai sensor tidak akurat
- Untuk DHT22: tunggu 2 detik antara pembacaan
- Untuk Soil Moisture: lakukan kalibrasi sesuai kondisi tanah Anda
- Pastikan sensor tidak rusak

## 📚 Library yang Digunakan

- **DHT Sensor Library** (Adafruit) - untuk membaca DHT22
- **Adafruit Unified Sensor** - dependency untuk DHT library
- **WiFi** (Built-in ESP32) - untuk koneksi WiFi
- **WebServer** (Built-in ESP32) - untuk web server

## 🎨 Customization

Anda dapat mengkustomisasi:
- Warna tema di CSS (gradient background, card colors)
- Interval update (ubah `updateInterval`)
- Layout dashboard (modifikasi HTML/CSS)
- Menambah sensor lain

## 📝 Catatan

- ESP32 ADC memiliki resolusi 12-bit (0-4095)
- DHT22 membutuhkan delay minimal 2 detik antara pembacaan
- GPIO 34, 35, 36, 39 adalah input-only (tidak bisa digunakan sebagai output)
- Pastikan power supply cukup untuk semua sensor

## 📄 License

Free to use and modify for your projects.

---

**Selamat menggunakan! 🌱**

