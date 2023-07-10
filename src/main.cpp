/*
 * Includes
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <IWatchdog.h>
#include "SdFat.h"
#include "SparkFun_u-blox_GNSS_v3.h"
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
#include "psg_4_definitions.h"

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
HardwareSerial SerialLoRa(USART1);
OneWire one_wire_dallas(EXT_TEMP_PIN);
DeviceAddress one_wire_addresses;

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
struct mcu0_data {
    uint8_t band_id;
    uint32_t counter;
    uint8_t state;

    uint32_t gps_time;
    uint32_t gps_time_us;
    uint8_t gps_siv;
    real_t gps_latitude;
    real_t gps_longitude;
    real_t gps_altitude;

    real_t acc_x;
    real_t acc_y;
    real_t acc_z;
    real_t eul_x;
    real_t eul_y;
    real_t eul_z;

    float pressure_int;
    float humidity_int;
    float temperature_int;

    float pressure_ext;
    float humidity_ext;
    float temperature_ext;

    float pressure_probe;
    float temperature_probe;

    float batt_volt;
} payload;
String payload_str;

struct sensors_list {
    bool bme280;
    bool ms8607;
    bool ds18b20;
    bool mprls;
    bool bno085;
    bool m10s;
} valid;

lora_e22 lora(SerialLoRa, PIN_M0, PIN_M1);
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
task_scheduler<10> scheduler;

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

extern void write_storage();

extern void write_comm();

void setup() {
    // System
    IWatchdog.begin(1000000ul);
    payload.band_id = 0;  // Set ID to main device
    payload_str.reserve(256);

    // Pins
    pinMode(PIN_BOARD_LED, OUTPUT);

    // UARTs
    Serial.begin(115200);

    // I2Cs
    Wire.setSCL(PIN_I2C_SCL1);
    Wire.setSDA(PIN_I2C_SDA1);

    // Sensors
    valid.m10s = m10s.begin();
    if (valid.m10s) {
        m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, 250u);
        m10s.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, 250u);
        m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, 250u);
        m10s.setDynamicModel(DYN_MODEL_AIRBORNE2g, VAL_LAYER_RAM_BBR, 250u);
//        m10s.saveConfiguration(250u);  // (For saving to module's flash memory)
    }

    bme280.begin(0x76);

    ds18b20.setOneWire(&one_wire_dallas);
    ds18b20.setResolution(9);
    ds18b20.begin();
    ds18b20.getAddress(one_wire_addresses, 0);

    valid.mprls = mprls.begin();
    valid.ms8607 = ms8607.begin();
    valid.bno085 = bno085.begin_I2C();
    if (valid.bno085) bno085_set_reports();

    // Add tasks
    scheduler.add_task(static_cast<void (*)()>([]() -> void { IWatchdog.reload(); }), 5000ul * 1000ul, micros);
    scheduler.add_task(read_bme280, 250ul, millis);
    if (valid.ms8607) scheduler.add_task(read_ms8607, 250ul, millis);
    scheduler.add_task(read_ds18b20, 250ul, millis);
    if (valid.mprls) scheduler.add_task(read_mprls, 250ul, millis);
    if (valid.m10s) scheduler.add_task(read_m10s, 500ul, millis);
    if (valid.bno085) scheduler.add_task(read_bno085, 50ul * 1000ul, micros);
    scheduler.add_task(construct_string, 1000ul, millis);
}

void loop() { scheduler.exec(); }

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
                payload.acc_x = bno085_values.un.accelerometer.x;
                payload.acc_y = bno085_values.un.accelerometer.y;
                payload.acc_z = bno085_values.un.accelerometer.z;
            default:
                break;
        }
    }

    // Kalman filter Predict-Update
    kf.accel.x.predict();
    kf.accel.y.predict();
    kf.accel.z.predict();
    kf.euler.x.predict();
    kf.euler.y.predict();
    kf.euler.z.predict();
    kf.accel.x.update(payload.acc_x);
    kf.accel.y.update(payload.acc_y);
    kf.accel.z.update(payload.acc_z);
    kf.euler.x.update(payload.eul_x);
    kf.euler.y.update(payload.eul_y);
    kf.euler.z.update(payload.eul_z);
}

void read_bme280() {
    payload.pressure_int = bme280.read_pressure();
    payload.humidity_int = bme280.read_humidity();
    payload.temperature_int = bme280.read_temperature_c();
}

void read_ms8607() {
    sensors_event_t e_p, e_h, e_t;
    ms8607.getEvent(&e_p, &e_t, &e_h);
    payload.pressure_ext = e_p.pressure;
    payload.humidity_ext = e_p.relative_humidity;
    payload.temperature_ext = e_p.temperature;
}

void read_ds18b20() {
    ds18b20.requestTemperaturesByAddress(one_wire_addresses);
    payload.temperature_probe = ds18b20.getTempC(one_wire_addresses);
}

void read_mprls() {
    payload.pressure_probe = mprls.readPressure();
}

void read_m10s() {
    if (m10s.getPVT()) {
        payload.gps_time = m10s.getUnixEpoch(payload.gps_time_us);
        payload.gps_siv = m10s.getSIV(250u);
        payload.gps_latitude = static_cast<real_t>(m10s.getLatitude(250u)) * 1.e-7;
        payload.gps_longitude = static_cast<real_t>(m10s.getLongitude(250u)) * 1.e-7;
        payload.gps_altitude = static_cast<real_t>(m10s.getAltitude(250u)) * 1.e-3;
    }
}

void construct_string() {
    build_string_to(payload_str,
                    payload.band_id,
                    payload.counter,
                    payload.state,
                    payload.gps_time,
                    payload.gps_time_us,
                    payload.gps_siv,
                    String(payload.gps_latitude, 6),
                    String(payload.gps_longitude, 6),
                    String(payload.gps_altitude, 6),
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
                    payload.pressure_probe,
                    payload.temperature_probe,
                    payload.batt_volt
    );
    Serial.print(payload_str.length());
    Serial.print(" ");
    Serial.println(payload_str);

    ++payload.counter;
}
