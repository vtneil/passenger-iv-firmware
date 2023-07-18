/*
 * Includes
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <IWatchdog.h>
#include "SdFat.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO08x.h"
#include "Adafruit_MS8607.h"
#include "Adafruit_MPRLS.h"
#include "DallasTemperature.h"
#include "vt_tools"
#include "vt_linalg"
#include "vt_kalman"
#include "vt_imu"
#include "vt_bme280"
#include "vt_lora"
#include "vt_serializer"
#include "psg_4_definitions.h"
#include "psg_4_sd_tools.h"
#include "psg_4_data_tools.h"

/**
 * Macros and Options
 */
//#define DEBUG_PRINT
#define ENABLE_IWDT
#define ENABLE_UPLINK
#define BLINK_STATUS
//#define CONFIG_LORA

/*
 * Namespaces
 */
using namespace vt;

/*
 * Constants
 */
constexpr uint32_t BNO085_UPDATE_INTERVAL_US = 10ul * 1000ul;  // 10,000 us
constexpr uint32_t BNO085_POLLING_INTERVAL = 10ul * 1000ul;    // ms * 1000 -> us

constexpr real_t dt = static_cast<real_t>(BNO085_POLLING_INTERVAL) * 0.001 * 0.001;
constexpr real_t d2t = integral_coefficient<1>() * (dt * dt);
constexpr real_t d3t = integral_coefficient<2>() * (dt * dt * dt);
constexpr real_t base_noise_value = 0.900;

constexpr numeric_matrix<4, 4> FCJ = make_numeric_matrix({{1, dt, d2t, d3t},
                                                          {0, 1,  dt,  d2t},
                                                          {0, 0,  1,   dt},
                                                          {0, 0,  0,   1}});
constexpr numeric_matrix<4, 1> BCJ;
constexpr numeric_matrix<1, 4> HCJ = make_numeric_matrix({{1, 0, 0, 0}});
constexpr numeric_matrix<4, 4> QCJ = numeric_matrix<4, 4>::diagonals(base_noise_value);
constexpr numeric_matrix<1, 1> RCJ = numeric_matrix<1, 1>::diagonals(base_noise_value);
constexpr numeric_vector<4> x0;

/*
 * System
 */
//#ifdef Serial
//#undef Serial
//#endif
//#define Serial Serial1
HardwareSerial Serial1(PA_10, PA_9);
HardwareSerial &SerialLoRa = Serial1;

SPIClass SPI_SD(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCLK1);
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;
SdFat32 sd1;
File32 sd1_file;
String sd1_filename;

OneWire one_wire_dallas(EXT_TEMP_PIN);
DeviceAddress one_wire_addresses;
arduino_iostream cout(Serial);

/*
 * Kalman Filters
 */
struct kf_s {
    struct {
        kalman_filter_t<4, 1, 1> x{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
        kalman_filter_t<4, 1, 1> y{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
        kalman_filter_t<4, 1, 1> z{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
    } accel;

    struct {
        kalman_filter_t<4, 1, 1> x{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
        kalman_filter_t<4, 1, 1> y{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
        kalman_filter_t<4, 1, 1> z{FCJ, BCJ, HCJ, QCJ, RCJ, x0};
    } euler;
} kf;

/*
 * MCU Data struct
 */
struct mcu0_data payload;
String payload_str;

struct peripherals {
    bool sd0;
    bool sd1;
    bool bme280;
    bool ms8607;
    bool ds18b20;
    bool mprls;
    bool bno085;
    bool m10s;
} valid;

lora_e22 lora(SerialLoRa, PIN_LORA_M0, PIN_LORA_M1);
SFE_UBLOX_GNSS m10s;                // Internal: SF u-blox M10S
bme280_t bme280;                    // Internal: BME280
Adafruit_MS8607 ms8607;             // External: MS8607
DallasTemperature ds18b20;          // External: DS18B20
Adafruit_MPRLS mprls;               // Probe   : MPRLS
Adafruit_BNO08x bno085;             // Internal: BNO08x
sh2_SensorValue_t bno085_values;    // (BNO085 data)

/*
 * Smart delayers and Scheduler
 */
task_scheduler<20> scheduler;

/*
 * Functions
 */
extern void bno085_set_reports();

extern void read_bno085();

extern void read_bme280();

extern void read_ms8607();

extern void read_ds18b20();

extern void read_mprls();

extern void read_m10s();

extern void construct_string();

extern void write_sd0();

extern void write_sd1();

extern void write_usb();

extern void write_comm();

extern void calculate_state();

extern void debug_prompt_handler(Stream &stream);

extern void i2cdetect(uint8_t first = 0, uint8_t last = 127, TwoWire &wire = Wire, Stream &stream = Serial);

void setup() {
#ifdef ENABLE_IWDT
    static_cast<void>(IWatchdog.isReset(true));
#endif

    // Enable onboard LED
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, !HIGH);

    // Enable external LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // USB CDC Serial
    Serial.begin();

    // Intentional delay for exactly 1000 ms
    delayMicroseconds(1000ul * 1000ul);

    // Config LoRa
#ifdef CONFIG_LORA
    Serial.println("Beginning configuration mode...");
    lora.config();
    Serial.println("Entered configuration mode!");
    lora.set_param(0xffff,
                    0,
                    115200u,
                    LoRaParity::PARITY_8N1,
                    2400u,
                    LORA_CHANNEL,
                    240u,
                    true);
    lora.query_param();
    Serial.println("Done config!");
    delay(100u);
#endif

    // SPI SD0 (Internal Flash)
    valid.sd0 = sd0.begin(SdSpiConfig(PIN_SPI_CS_SD_INT, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
    if (valid.sd0) {
        make_new_filename(sd0, sd0_filename, "mcu0_data_sd0_", ".csv");
        open_for_append(sd0, sd0_file, sd0_filename);
    }

    // SPI SD1 (External Card)
    valid.sd1 = sd1.begin(SdSpiConfig(PIN_SPI_CS_SD_EXT, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
    if (valid.sd1) {
        make_new_filename(sd1, sd1_filename, "mcu0_data_sd1_", ".csv");
        open_for_append(sd1, sd1_file, sd1_filename);
    }

    // Intentional delay for exactly 1000 ms
    delayMicroseconds(1000ul * 1000ul);

    // I2Cs
    Wire.setSCL(PIN_I2C_SCL1);
    Wire.setSDA(PIN_I2C_SDA1);
    Wire.setClock(I2C_CLOCK_STANDARD);
    Wire.begin();

    // BME280 (0x77)
    valid.bme280 = (0x60 == bme280.begin(BME280_ADDRESS));

    // MS8607 (0x76 and 0x40)
    valid.ms8607 = ms8607.begin();

    // BNO085 (0x4A)
    valid.bno085 = bno085.begin_I2C();
    if (valid.bno085) bno085_set_reports();

    // MAX-M10S (0x42)
    valid.m10s = m10s.begin(0x42, UBLOX_CUSTOM_MAX_WAIT);
    if (valid.m10s) {
        m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setDynamicModel(DYN_MODEL_AIRBORNE2g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
//        m10s.saveConfiguration(UBLOX_CUSTOM_MAX_WAIT);  // (For saving to module's flash memory, optional)
    }

    // MPRLS (0x18)
    valid.mprls = mprls.begin();

    // DS18B20
    ds18b20.setOneWire(&one_wire_dallas);
    ds18b20.setResolution(9);
    ds18b20.begin();
    valid.ds18b20 = ds18b20.getAddress(one_wire_addresses, 0);

    // Battery Voltage Readings (1)
    scheduler.add_task([]() -> void {
        payload.batt_volt = analogRead(PIN_VBATT);
    }, 1000ul, millis);

    // WDT reloader task (1)
#ifdef ENABLE_IWDT
    scheduler.add_task([]() -> void {
        IWatchdog.reload();
    }, WDT_INT_RELOAD, micros);
#endif

    // Sensor reading tasks (6)
    scheduler
            .add_task(read_bme280, 100ul, millis, valid.bme280)
            .add_task(read_ms8607, 200ul, millis, valid.ms8607)
            .add_task(read_mprls, 200ul, millis, valid.mprls)
            .add_task(read_m10s, 500ul, millis, valid.m10s)
            .add_task(read_bno085, 100ul * 1000ul, micros, valid.bno085)
            .add_task(read_ds18b20, 1000ul, millis, valid.ds18b20);

    // Data construction and communication tasks (5)
    scheduler
            .add_task(construct_string, 100ul, millis)
            .add_task(write_comm, 2000ul, millis)
            .add_task(write_usb, 1000ul, millis)
            .add_task(write_sd0, 500ul, millis, valid.sd0)
            .add_task(write_sd1, 100ul, millis, valid.sd1);

    // State calculation (1)
    scheduler.add_task(calculate_state, STATE_INT_UPDATE, millis);

    // Debug from Uplink and USB (2)
#ifdef ENABLE_UPLINK
    scheduler
            .add_task([]() -> void {
                debug_prompt_handler(Serial);
            }, 250ul, millis)
            .add_task([]() -> void {
                debug_prompt_handler(SerialLoRa);
            }, 250ul, millis);
#endif

    payload.band_id = 0;  // Set ID to main device
    payload_str.reserve(PAYLOAD_STR_MAX_LEN);

    // UARTs
    lora.begin(115200u);

    // Turn off all LEDs
    digitalWrite(PIN_BOARD_LED, !LOW);
    digitalWrite(PIN_LED, LOW);

    // LED Blink during operation (1)
#ifdef BLINK_STATUS
    scheduler.add_task([]() -> void {
        digitalToggle(PIN_BOARD_LED);
        digitalToggle(PIN_LED);
    }, 500ul, millis);
#endif

    // System: IWDG Timer
#ifdef ENABLE_IWDT
    IWatchdog.begin(WDT_INT_TIMEOUT);
#endif
}

void loop() { scheduler.exec(); }  // Run tasks only! Should not add anything. Long delay is prohibited.

void bno085_set_reports() {
    bno085.enableReport(SH2_ROTATION_VECTOR, BNO085_UPDATE_INTERVAL_US);
    bno085.enableReport(SH2_ACCELEROMETER, BNO085_UPDATE_INTERVAL_US);
}

void read_bno085() {
    if (bno085.wasReset()) bno085_set_reports();
    if (bno085.getSensorEvent(&bno085_values)) {
        switch (bno085_values.sensorId) {
            case SH2_ROTATION_VECTOR:
                imu_tools::quaternion_to_euler(make_numeric_vector(
                        {
                                bno085_values.un.rotationVector.real,
                                bno085_values.un.rotationVector.i,
                                bno085_values.un.rotationVector.j,
                                bno085_values.un.rotationVector.k
                        }
                )) >> payload.eul_x >> payload.eul_y >> payload.eul_z;
                break;
            case SH2_ACCELEROMETER:
                tie(
                        bno085_values.un.accelerometer.x,
                        bno085_values.un.accelerometer.y,
                        bno085_values.un.accelerometer.z
                ) >> payload.acc_x >> payload.acc_y >> payload.acc_z;
            default:
                break;
        }
    }

    // Kalman filter Predict-Update
    payload.acc_x = kf.accel.x.predict().update(payload.acc_x).state;
    payload.acc_y = kf.accel.y.predict().update(payload.acc_y).state;
    payload.acc_z = kf.accel.z.predict().update(payload.acc_z).state;
    payload.eul_x = kf.euler.x.predict().update(payload.eul_x).state;
    payload.eul_y = kf.euler.y.predict().update(payload.eul_y).state;
    payload.eul_z = kf.euler.z.predict().update(payload.eul_z).state;
}

void read_bme280() {
    payload.pressure_int = bme280.read_pressure();
    payload.humidity_int = bme280.read_humidity();
    payload.temperature_int = bme280.read_temperature_c();
}

void read_ms8607() {
    sensors_event_t sp;
    sensors_event_t st;
    sensors_event_t sh;
    ms8607.getEvent(&sp, &st, &sh);
    payload.pressure_ext = sp.pressure;
    payload.temperature_ext = st.temperature;
    payload.humidity_ext = sh.relative_humidity;

//    sensors_event_t sensors_event;
//    ms8607.getEvent(&sensors_event, nullptr, nullptr);
//    payload.pressure_ext = sensors_event.pressure;
//    ms8607.getEvent(nullptr, &sensors_event, nullptr);
//    payload.temperature_ext = sensors_event.temperature;
//    ms8607.getEvent(nullptr, nullptr, &sensors_event);
//    payload.humidity_ext = sensors_event.relative_humidity;

    payload.altitude_ext = static_cast<float>(calc_altitude_approx_slow(payload.pressure_ext));
}

void read_ds18b20() {
    ds18b20.requestTemperaturesByAddress(one_wire_addresses);
    payload.temperature_probe = ds18b20.getTempC(one_wire_addresses);
}

void read_mprls() {
    payload.pressure_probe = mprls.readPressure();
    payload.altitude_probe = static_cast<float>(calc_altitude_approx_slow(payload.pressure_probe));
}

void read_m10s() {
    if (m10s.getPVT()) {
        payload.gps_time = m10s.getUnixEpoch(payload.gps_time_us, UBLOX_CUSTOM_MAX_WAIT);
        payload.gps_siv = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
        payload.gps_latitude = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        payload.gps_longitude = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        payload.gps_altitude = static_cast<float>(m10s.getAltitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
    }
}

void construct_string() {
    payload.uptime = millis();
    build_string_to(payload_str,
                    payload.band_id,
                    payload.counter,
                    payload.state,
                    payload.uptime,
                    payload.gps_time,
                    payload.gps_time_us,
                    payload.gps_siv,
                    String(payload.gps_latitude, 8),
                    String(payload.gps_longitude, 8),
                    payload.gps_altitude,
                    payload.acc_x,
                    payload.acc_y,
                    payload.acc_z,
                    payload.eul_x,
                    payload.eul_y,
                    payload.eul_z,
                    payload.pressure_int,
                    payload.humidity_int,
                    payload.temperature_int,
                    payload.pressure_ext,
                    payload.humidity_ext,
                    payload.temperature_ext,
                    payload.altitude_ext,
                    payload.pressure_probe,
                    payload.temperature_probe,
                    payload.altitude_probe,
                    payload.batt_volt
    );
    ++payload.counter;
}

void write_sd0() {
    sd0_file.println(payload_str);
    sd0_file.flush();
}

void write_sd1() {
    sd1_file.println(payload_str);
    sd1_file.flush();
}

void write_usb() {
    String valid_str;
    valid_str.reserve(8);
    valid_str += STR_REPR(valid.sd0);
    valid_str += STR_REPR(valid.sd1);
    valid_str += STR_REPR(valid.bme280);
    valid_str += STR_REPR(valid.ms8607);
    valid_str += STR_REPR(valid.ds18b20);
    valid_str += STR_REPR(valid.mprls);
    valid_str += STR_REPR(valid.bno085);
    valid_str += STR_REPR(valid.m10s);

    cout << "Valid=" << valid_str
         << ", Size=" << sizeof(payload)
         << ", Length=" << payload_str.length()
         << ", Free Mem=" << free_memory()
         << ", Data=" << payload_str
         << "\r\n";
}

void write_comm() {
#ifndef SIMPLE_RXTX
    checksum_t checksum = calc_checksum<checksum_t>(payload);

    SerialLoRa.write(START_SEQ, sizeof(START_SEQ));
    SerialLoRa.write(reinterpret_cast<uint8_t *>(&checksum), sizeof(checksum));
    SerialLoRa.write(reinterpret_cast<uint8_t *>(&payload), sizeof(payload));
#else
    SerialLoRa.println(payload_str);
//    i2cdetect(0, 127, Wire, SerialLoRa);
#endif
}

void get_valid_altitude(float &altitude) {
    if (validate_altitude(payload.gps_altitude)) altitude = payload.gps_altitude;
    if (validate_altitude(payload.altitude_ext)) altitude = payload.altitude_ext;

    // Default altitude from pressure port sensor, inaccurate but wouldn't die.
    altitude = payload.altitude_probe;
}

void calculate_state() {
    static float curr_alt;
    get_valid_altitude(curr_alt);

    switch (static_cast<GlobalState>(payload.state)) {
        case GlobalState::PREPARATION:
            payload.state = static_cast<uint8_t>(GlobalState::AIRBORNE_READY);
            return;
        case GlobalState::AIRBORNE_READY:
            return;
        case GlobalState::ASCENT_SUB_HALF:
            return;
        case GlobalState::ASCENT_SUB_MAX:
            return;
        case GlobalState::DESCENT:
            return;
        case GlobalState::DESCENT_ENB_CELL:
            return;
        case GlobalState::LANDED_FIXED:
            return;
        case GlobalState::INVALIDATE:
            return;
    }
}

void debug_prompt_handler(Stream &stream) {
    if (!stream.available()) return;

    uint8_t rx_cnt = 1;

    // Read and wait on buffer
    char c = static_cast<char>(stream.read());
    delayMicroseconds(10ul * 1000ul);

    // Clear remaining on buffer
    while (stream.available()) {
        c = (c == static_cast<char>(stream.read())) ? c : 'x';
        ++rx_cnt;
    }

    // Cancel if rx_cnt is less than 5
    if (rx_cnt < 5) return;

    switch (c) {
        case 's': // fall through
        case 'S': // List SD card files
            if (valid.sd0) {
                stream.println("SD0 Files");
                list_files(sd0);
            }
            if (valid.sd1) {
                stream.println("SD1 Files");
                list_files(sd1);
            }
            break;

        case 'd': // fall through
        case 'D': // Delete all files in all SD Card (All files will be lost!)
            if (valid.sd0) delete_all_in(sd0);
            if (valid.sd1) delete_all_in(sd1);
            break;

        case 'i': // fall through
        case 'I': // List I2C devices on the bus
            i2cdetect(0, 127, Wire, stream);
            break;

        case 'l': // fall through
        case 'L': // Config LoRa module to the preset value
            lora.config();
            lora.set_param(0xffff,
                           0,
                           115200u,
                           LoRaParity::PARITY_8N1,
                           2400u,
                           LORA_CHANNEL,
                           240u,
                           true);
            delay(100u);
            lora.begin(115200u);
            break;

        case 'r': // fall through
        case 'R': // System Reset
            NVIC_SystemReset();

        case 'x':
        case 'X':
        default:
            break;
    }
}

void i2cdetect(uint8_t first, uint8_t last, TwoWire &wire, Stream &stream) {
    uint8_t resp;
    char buf[10];

    stream.print("   ");
    for (uint8_t i = 0; i < 16; i++) {
        sprintf(buf, "%3x", i);
        stream.print(buf);
    }

    for (uint8_t addr = 0; addr < 0x80; addr++) {
        if (addr % 16 == 0) {
            sprintf(buf, "\n%02x:", addr & 0xF0);
            stream.print(buf);
        }
        if (addr >= first && addr <= last) {
            wire.beginTransmission(addr);
            resp = wire.endTransmission();
            if (resp == 0) {
                // device found
                //stream.printf(" %02x", addr);
                sprintf(buf, " %02x", addr);
                stream.print(buf);
            } else if (resp == 4) {
                // other resp
                stream.print(" XX");
            } else {
                // resp = 2: received NACK on transmit of addr
                // resp = 3: received NACK on transmit of data
                stream.print(" --");
            }
        } else {
            // addr not scanned
            stream.print("   ");
        }
    }
    stream.println("\n");
}
