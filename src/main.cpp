/*
 * Includes
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BNO08x.h"
#include "vt_tools"
#include "vt_linalg"
#include "vt_kalman"
#include "vt_imu"
#include "vt_bme280"

/*
 * Namespaces
 */
using namespace vt;

/*
 * Constants
 */
constexpr uint32_t BNO085_UPDATE_INTERVAL_US = 10UL * 1000UL;  // 10,000 us
constexpr uint32_t SYSTEM_POLLING_INTERVAL = 10UL * 1000UL;   // ms * 1000 -> us

constexpr real_t dt = static_cast<real_t>(SYSTEM_POLLING_INTERVAL) * 0.001 * 0.001;
constexpr real_t d2t = integral_coefficient<1>() * (dt * dt);
constexpr real_t d3t = integral_coefficient<2>() * (dt * dt * dt);
constexpr real_t base_noise_value = 0.900;

constexpr numeric_matrix<4, 4> FCJ = make_numeric_matrix({{1, dt, d2t, d3t},
                                                          {0, 1,  dt,  d2t},
                                                          {0, 0,  1,   dt},
                                                          {0, 0,  0,   1}});
constexpr numeric_matrix<4, 1> B1D;
constexpr numeric_matrix<1, 4> H1D = make_numeric_matrix({{1, 0, 0, 0}});
constexpr numeric_matrix<4, 4> Q1D = numeric_matrix<4, 4>::diagonals(base_noise_value);
constexpr numeric_matrix<1, 1> R1D = numeric_matrix<1, 1>::diagonals(base_noise_value);
constexpr numeric_vector<4> x0;

/*
 * System
 */
HardwareSerial SerialLoRa(USART1);

/*
 * Kalman Filters
 */
struct kf_s {
    struct {
        kalman_filter_t<4, 1, 1> x{FCJ, B1D, H1D, Q1D, R1D, x0};
        kalman_filter_t<4, 1, 1> y{FCJ, B1D, H1D, Q1D, R1D, x0};
        kalman_filter_t<4, 1, 1> z{FCJ, B1D, H1D, Q1D, R1D, x0};
    } accel;

    struct {
        kalman_filter_t<4, 1, 1> x{FCJ, B1D, H1D, Q1D, R1D, x0};
        kalman_filter_t<4, 1, 1> y{FCJ, B1D, H1D, Q1D, R1D, x0};
        kalman_filter_t<4, 1, 1> z{FCJ, B1D, H1D, Q1D, R1D, x0};
    } euler;
} kf;

/*
 * MCU Data struct
 */
struct mcu_data {
    struct {
        float pressure{};
        float humidity{};
        float temperature{};
    } bme280;

    struct {
        numeric_vector<3> accel;
        numeric_vector<3> euler;
    } bno085;
} data;

bme280_t bme280;
Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085_values;

/*
 * Smart delayers
 */
smart_delay sd_bme280(500UL * 1000UL, micros);
smart_delay sd_bno085(SYSTEM_POLLING_INTERVAL, micros);

/*
 * Functions
 */
extern void bno085_set_reports();

extern void read_bno085();

extern void read_bme280();

void setup() {
    // Pins
    pinMode(PC13, OUTPUT);

    // UARTs
    Serial.begin(115200);

    // I2Cs
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);

    // Sensors
    bme280.begin();
    bno085.begin_I2C();
    bno085_set_reports();

    // Reset smart delayers
    sd_bno085.reset();
    sd_bme280.reset();
}

void loop() {
    if (sd_bno085) {
        digitalWrite(PC13, !digitalRead(PC13));

        kf.accel.x.predict();
        kf.accel.y.predict();
        kf.accel.z.predict();
        kf.euler.x.predict();
        kf.euler.y.predict();
        kf.euler.z.predict();
        read_bno085();
        kf.accel.x.update(data.bno085.accel[0]);
        kf.accel.y.update(data.bno085.accel[1]);
        kf.accel.z.update(data.bno085.accel[2]);
        kf.euler.x.update(data.bno085.euler[0]);
        kf.euler.y.update(data.bno085.euler[1]);
        kf.euler.z.update(data.bno085.euler[2]);

        Serial.println(build_string(
                data.bno085.accel[0],
                kf.accel.x.state,

                data.bno085.accel[1],
                kf.accel.y.state,

                data.bno085.accel[2],
                kf.accel.z.state,

                data.bno085.euler[0],
                kf.euler.x.state,

                data.bno085.euler[1],
                kf.euler.y.state,

                data.bno085.euler[2],
                kf.euler.z.state
        ));
    }

    if (sd_bme280) {
        read_bme280();
//        Serial.println(build_string(
//                data.bme280.pressure,
//                data.bme280.humidity,
//                data.bme280.temperature
//        ));
    }
}

void bno085_set_reports() {
    bno085.enableReport(SH2_ROTATION_VECTOR, BNO085_UPDATE_INTERVAL_US);
    bno085.enableReport(SH2_ACCELEROMETER, BNO085_UPDATE_INTERVAL_US);
}

void read_bno085() {
    if (bno085.wasReset()) bno085_set_reports();
    if (bno085.getSensorEvent(&bno085_values)) {
        switch (bno085_values.sensorId) {
            case SH2_ROTATION_VECTOR:
                data.bno085.euler = imu_tools::quaternion_to_euler(make_numeric_vector(
                        {
                                bno085_values.un.rotationVector.real,
                                bno085_values.un.rotationVector.i,
                                bno085_values.un.rotationVector.j,
                                bno085_values.un.rotationVector.k
                        }
                ));
                break;
            case SH2_ACCELEROMETER:
                data.bno085.accel = {
                        bno085_values.un.accelerometer.x,
                        bno085_values.un.accelerometer.y,
                        bno085_values.un.accelerometer.z
                };
            default:
                break;
        }
    }
}

void read_bme280() {
    data.bme280 = {
            bme280.read_pressure(),
            bme280.read_humidity(),
            bme280.read_temperature_c()
    };
}
