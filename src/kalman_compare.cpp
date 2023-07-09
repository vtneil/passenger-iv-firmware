/*
 * Includes
 */
#include <Arduino.h>
#include "vt_tools"
#include "vt_linalg"
#include "vt_kalman"
#include "vt_imu"
#include "Adafruit_BNO08x.h"

using namespace vt;

constexpr uint32_t BNO085_UPDATE_INTERVAL_US = 10UL * 1000UL;  // 10,000 us
constexpr uint32_t SYSTEM_POLLING_INTERVAL = 5UL * 1000UL;   // ms * 1000 -> us

constexpr real_t dt = static_cast<real_t>(SYSTEM_POLLING_INTERVAL) * 0.001 * 0.001;
constexpr real_t d2t = integral_coefficient<1>() * (dt * dt);
constexpr real_t d3t = integral_coefficient<2>() * (dt * dt * dt);
constexpr real_t base_noise_value = 0.08;

constexpr numeric_matrix<3, 3> FCA = make_numeric_matrix({{1, dt, d2t},
                                                          {0, 1,  dt},
                                                          {0, 0,  1}});
constexpr numeric_matrix<3, 1> BCA;
constexpr numeric_matrix<1, 3> HCA = make_numeric_matrix({{1, 0, 0}});
constexpr numeric_matrix<3, 3> QCA = numeric_matrix<3, 3>::diagonals(base_noise_value);
constexpr numeric_matrix<1, 1> RCA = numeric_matrix<1, 1>::diagonals(base_noise_value);
constexpr numeric_vector<3> xca;

constexpr numeric_matrix<4, 4> FCJ = make_numeric_matrix({{1, dt, d2t, d3t},
                                                          {0, 1,  dt,  d2t},
                                                          {0, 0,  1,   dt},
                                                          {0, 0,  0,   1}});
constexpr numeric_matrix<4, 1> BCJ;
constexpr numeric_matrix<1, 4> HCJ = make_numeric_matrix({{1, 0, 0, 0}});
constexpr numeric_matrix<4, 4> QCJ = numeric_matrix<4, 4>::diagonals(base_noise_value);
constexpr numeric_matrix<1, 1> RCJ = numeric_matrix<1, 1>::diagonals(base_noise_value);
constexpr numeric_vector<4> xcj;

struct kf_s {
    kalman_filter_t<3, 1, 1> x_ca{FCA, BCA, HCA, QCA, RCA, xca};
    kalman_filter_t<4, 1, 1> x_cj{FCJ, BCJ, HCJ, QCJ, RCJ, xcj};
} kf;

struct mcu_data {
    struct {
        numeric_vector<3> accel;
        numeric_vector<3> euler;
    } bno085;
} data;

Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085_values;

smart_delay sd_bno085(SYSTEM_POLLING_INTERVAL, micros);
//smart_delay_non_adaptive sd_bno085(SYSTEM_POLLING_INTERVAL, micros);

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

void setup() {
    // Pins
    pinMode(PC13, OUTPUT);

    // UARTs
    Serial.begin(115200);

    // I2Cs
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);

    // Sensors
    bno085.begin_I2C();
    bno085_set_reports();

    // Reset smart delayers
    sd_bno085.reset();
}

void loop() {
    if (sd_bno085) {
        digitalWrite(PC13, !digitalRead(PC13));

        kf.x_ca.predict();
        kf.x_cj.predict();
        read_bno085();
        kf.x_ca.update(data.bno085.accel[2]);
        kf.x_cj.update(data.bno085.accel[2]);

        Serial.println(build_string(
                sd_bno085.target_interval,
                sd_bno085.true_interval,
                data.bno085.accel[2],
                kf.x_ca.state,
                kf.x_cj.state
        ));
    }
}
