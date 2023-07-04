#include <Arduino.h>
#include "vt_tools"
#include "vt_kalman"
#include "vt_bme280"

using namespace vt;

constexpr uint32_t SENSOR_INTERVAL = 100; // ms
constexpr real_t dt = static_cast<real_t>(SENSOR_INTERVAL);
constexpr real_t hdts = 0.5 * dt * dt;
constexpr real_t base_noise_value = 0.001;

numeric_matrix<9, 9> Fm({{1, 0, 0, dt, 0,  0,  hdts, 0,    0},
                         {0, 1, 0, 0,  dt, 0,  0,    hdts, 0},
                         {0, 0, 1, 0,  0,  dt, 0,    0,    hdts},
                         {0, 0, 0, 1,  0,  0,  dt,   0,    0},
                         {0, 0, 0, 0,  1,  0,  0,    dt,   0},
                         {0, 0, 0, 0,  0,  1,  0,    0,    dt},
                         {0, 0, 0, 0,  0,  0,  1,    0,    0},
                         {0, 0, 0, 0,  0,  0,  0,    1,    0},
                         {0, 0, 0, 0,  0,  0,  0,    0,    1}});
numeric_matrix<9, 1> B;
numeric_matrix<3, 9> H({{1},
                        {0, 1},
                        {0, 0, 1}});
numeric_matrix<9, 9> Q = numeric_matrix<9, 9>::diagonals(base_noise_value);
numeric_matrix<3, 3> R = numeric_matrix<3, 3>::diagonals(base_noise_value);
numeric_vector<9> x0;

kalman_filter_t<9, 3, 1> kf_bme280(Fm, B, H, Q, R, x0);
bme280_t sensor;

float temp;
float hum;
float pres;

void setup() {
    pinMode(PC13, OUTPUT);
    Serial.begin(115200);

    delay(1000);
    Serial.println(F("Serial OK!"));
    Serial.println(sensor.begin());
}

void loop() {
    kf_bme280.predict({});

    static uint32_t counter = 0;
    temp = sensor.read_temperature_c();
    hum = sensor.read_humidity();
    pres = sensor.read_pressure();

    kf_bme280.update(make_numeric_vector({temp, hum, pres}));

    Serial.println(build_string(counter,
                                temp,
                                kf_bme280.state_vector()[0],
                                hum,
                                kf_bme280.state_vector()[1],
                                pres,
                                kf_bme280.state_vector()[2]));

    delay(1000);
}
