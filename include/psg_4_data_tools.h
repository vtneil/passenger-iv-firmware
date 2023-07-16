#include <math.h>

bool validate_latitude(double latitude) {
    return (-90. <= latitude) && (latitude <= 90.);
}

bool validate_longitude(double longitude) {
    return (0. <= longitude) && (longitude <= 180.);
}

bool validate_altitude(float altitude, float altitude_min = 0.01f, float altitude_max = 80000.f) {
    return (altitude_min <= altitude) && (altitude <= altitude_max);
}

bool validate_temperature(float temperature) {
    return (-85.f <= temperature) && (temperature <= 85.f);
}

double calc_altitude_approx_slow(double pressure_hpa) {
    static constexpr double s = 5.25588;
    static constexpr double k = 2.25577 * 1.e-5;
    static constexpr double k_inv = 1. / k;
    static constexpr double e_s_inv = 1.209567788;
    static constexpr double k_inv_e_s_inv_minus_ln_p0 = 4946.545908;
    return k_inv - k_inv_e_s_inv_minus_ln_p0 * pow(e_s_inv, log(pressure_hpa * 100.));
}

float calc_rate(const float &val_now, float &val_prev, size_t dt) {
    float rate = (val_now - val_prev) / static_cast<float>(dt);
    val_prev = val_now;
    return rate;
}
