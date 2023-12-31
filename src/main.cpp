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
#include <Servo.h>

/*
 * Namespaces
 */
using namespace vt;

/*
 * Constants
 */
constexpr uint32_t BNO085_UPDATE_INTERVAL_US = 10ul * 1000ul;  // 10,000 us
constexpr uint32_t BNO085_POLLING_INTERVAL = 10ul * 1000ul;    // ms * 1000 -> us

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

OneWire one_wire_ds18b20(PIN_DS18B20);
DeviceAddress dev_addr;
arduino_iostream cout(Serial);

/*
 * Kalman Filters
 */
struct kf_s {
    static constexpr real_t dt = static_cast<real_t>(BNO085_POLLING_INTERVAL) * 0.001 * 0.001;
    static constexpr real_t d2t = integral_coefficient<1>() * (dt * dt);
    static constexpr real_t d3t = integral_coefficient<2>() * (dt * dt * dt);
    static constexpr real_t base_noise_value = 0.900;

    static constexpr numeric_matrix<4, 4> FCJ = make_numeric_matrix({{1, dt, d2t, d3t},
                                                                     {0, 1,  dt,  d2t},
                                                                     {0, 0,  1,   dt},
                                                                     {0, 0,  0,   1}});
    static constexpr numeric_matrix<4, 1> BCJ = {};
    static constexpr numeric_matrix<1, 4> HCJ = make_numeric_matrix({{1, 0, 0, 0}});
    static constexpr numeric_matrix<4, 4> QCJ = numeric_matrix<4, 4>::diagonals(base_noise_value);
    static constexpr numeric_matrix<1, 1> RCJ = numeric_matrix<1, 1>::diagonals(base_noise_value);
    static constexpr numeric_vector<4> x0 = {};

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
checksum_t checksum;

struct peripherals {
    bool sd0;
    bool sd1;
    bool bme280;
    bool bno085;
    bool m10s;
    bool ms8607;
    bool mprls;
    bool ds18b20;
} valid;

/*
 * Peripherals
 */
lora_e22 lora(SerialLoRa, PIN_LORA_M0, PIN_LORA_M1);
SFE_UBLOX_GNSS m10s;                // Internal: SF u-blox M10S
bme280_t bme280;                    // Internal: BME280
//Adafruit_MS8607 ms8607;             // External: MS8607
//Adafruit_MPRLS mprls;               // Probe   : MPRLS
DallasTemperature ds18b20(&one_wire_ds18b20);          // External: DS18B20
Adafruit_BNO08x bno085;             // Internal: BNO08x

Servo servo;

/*
 * Smart delayers and Scheduler
 */
task_scheduler<20> scheduler;
smart_delay servo_delayer(TIME_SERVO_CUT, millis);
HardwareTimer timer_led = HardwareTimer(TIM1);

/*
 * Functions
 */
extern void blink_leds();

extern void config_lora();

extern void bno085_set_reports();

extern void read_bno085();

extern void read_bme280();

//extern void read_ms8607();

extern void read_ds18b20();

//extern void read_mprls();

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

    // Device Presets
    payload.band_id = 0;  // Set ID to main device
    payload.state = state_t::INVALID;
    payload.last_response = 'x';
    payload_str.reserve(PAYLOAD_STR_MAX_LEN);

    analogReadResolution(MY_ADC_RESOLUTION);
    analogWriteResolution(MY_ADC_RESOLUTION);

    // Enable onboard LED
    pinMode(PIN_BOARD_LED, OUTPUT);

    // Enable external LED
    pinMode(PIN_LED, OUTPUT);

    // Enable internal buzzer
    pinMode(PIN_BUZZER, OUTPUT);

    // Turn on LED
    LED_ON();

    // Setup PWM LED
    timer_led.setOverflow(480, MICROSEC_FORMAT);
    timer_led.attachInterrupt([]() -> void {
        static uint16_t pwm_val = 0;
        static bool direction = true;
        if (direction) ++pwm_val;
        else --pwm_val;
        if (pwm_val == MY_ADC_MAX_VALUE - 1) direction = false;
        else if (pwm_val == 0) direction = true;
        analogWrite(PIN_LED, pwm_val);
    });
    timer_led.resume();

    // USB CDC Serial
    Serial.begin();

    // Servo
    servo.attach(PIN_SERVO_CUT);

    // Intentional delay for exactly 1000 ms
    delayMicroseconds(1000ul * 1000ul);

    // Turn off the buzzer
    BUZZER_OFF();

    // Config LoRa
    if (!digitalRead(GCS_PIN_KEY)) {
        config_lora();
    }
    while (!digitalRead(GCS_PIN_KEY));

    // SPI SD0 (Internal Flash)
    valid.sd0 = sd0.begin(SdSpiConfig(PIN_SPI_CS_SD_INT, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
    if (valid.sd0) {
        make_new_filename(sd0, sd0_filename, F_NAME, F_EXT);
        open_for_append(sd0, sd0_file, sd0_filename);
    }

    // SPI SD1 (External Card)
    valid.sd1 = sd1.begin(SdSpiConfig(PIN_SPI_CS_SD_EXT, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
    if (valid.sd1) {
        make_new_filename(sd1, sd1_filename, "mcu0_data_sd1_", ".csv");
        open_for_append(sd1, sd1_file, sd1_filename);
    }

    // Intentional delay for exactly 200 ms
    delayMicroseconds(1000ul * 1000ul);

    // I2Cs
    Wire.setSCL(PIN_I2C_SCL1);
    Wire.setSDA(PIN_I2C_SDA1);
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.begin();

    // BME280 (0x77)
    valid.bme280 = (0x60 == bme280.begin(BME280_ADDRESS));

    // MS8607 (0x76 and 0x40)
//    valid.ms8607 = ms8607.begin();

    // BNO085 (0x4A)
    valid.bno085 = bno085.begin_I2C();
    if (valid.bno085) bno085_set_reports();

    // MAX-M10S (0x42)
    valid.m10s = m10s.begin(0x42, UBLOX_CUSTOM_MAX_WAIT);
    if (valid.m10s) {
        m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);

//        m10s.saveConfiguration(UBLOX_CUSTOM_MAX_WAIT);  // (For saving to module's flash memory, optional)
    }

    // MPRLS (0x18)
//    valid.mprls = mprls.begin();

    // DS18B20
    ds18b20.begin();
    valid.ds18b20 = ds18b20.getAddress(dev_addr, 0);
    ds18b20.setResolution(dev_addr, 9);

    // Servo (checker and disabler) (1)
    pinMode(PIN_SERVO_CUT, OUTPUT);

    scheduler.add_task([]() -> void {
        if (servo_delayer) {
            SERVO_OFF();
        }
    }, SET_INT(100ul), millis);

    // Battery Voltage Readings (1)
    pinMode(PIN_VBATT, INPUT_ANALOG);
    scheduler.add_task([]() -> void {
        payload.batt_volt = (analogRead(PIN_VBATT) * 100) / MY_ADC_MAX_VALUE;
    }, SET_INT(1000ul), millis);

    // Sensor reading tasks (6)
#ifdef ENABLE_SENSORS
    scheduler
            .add_task(read_bme280, SET_INT(100ul), millis, valid.bme280)
//            .add_task(read_ms8607, SET_INT(100ul), millis, valid.ms8607)
//            .add_task(read_mprls, SET_INT(200ul), millis, valid.mprls)
            .add_task(read_m10s, SET_INT(500ul), millis, valid.m10s)
            .add_task(read_bno085, SET_INT(100ul * 1000ul), micros, valid.bno085)
            .add_task(read_ds18b20, SET_INT(1000ul), millis, valid.ds18b20);
#endif

    // Data construction and communication tasks (5)
    scheduler
            .add_task(construct_string, SET_INT(100ul), millis)
            .add_task(write_comm, SET_INT(COMM_INT_LORA), millis)
            .add_task(write_usb, SET_INT(COMM_INT_USB), millis)
            .add_task(write_sd0, SET_INT(100ul), millis, valid.sd0)
            .add_task(write_sd1, SET_INT(100ul), millis, valid.sd1);

    // State calculation (1)
    scheduler.add_task(calculate_state, SET_INT(STATE_INT_UPDATE), millis);

    // Debug from Uplink and USB (2)
#ifdef ENABLE_UPLINK
    scheduler
            .add_task([]() -> void {
                debug_prompt_handler(Serial);
            }, SET_INT(100ul), millis)
            .add_task([]() -> void {
                debug_prompt_handler(SerialLoRa);
            }, SET_INT(100ul), millis);
#endif

    // UARTs
    lora.begin(115200u);

    // Turn off all LEDs
    LED_OFF();
    BUZZER_OFF();

    // LED Blink during operation (1)
#ifdef BLINK_STATUS
    scheduler.add_task(blink_leds, SET_INT(250ul), millis);
#endif

    // System: IWDG Timer (1)
#ifdef ENABLE_IWDT
    scheduler.add_task([]() -> void {
        IWatchdog.reload();
    }, SET_INT(WDT_INT_RELOAD), micros);
    IWatchdog.begin(SET_INT(WDT_INT_TIMEOUT));
#endif
}

void loop() { scheduler.exec(); }  // Run tasks only! Should not add anything. Long delay is prohibited.

void blink_leds() {
    LED_TOGGLE();
    BUZZER_TOGGLE();
}

void config_lora() {
    lora.config();
    lora.set_param(0xffff,
                   0,
                   115200u,
                   LoRaParity::PARITY_8N1,
                   2400u,
                   LORA_CHANNEL,
                   240u,
                   true);
    lora.query_param();
    delay(100u);
    lora.begin(115200u);
}

void bno085_set_reports() {
    bno085.enableReport(SH2_ROTATION_VECTOR, BNO085_UPDATE_INTERVAL_US);
    bno085.enableReport(SH2_ACCELEROMETER, BNO085_UPDATE_INTERVAL_US);
}

void read_bno085() {
    static sh2_SensorValue_t bno085_values;

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

//void read_ms8607() {
//    static sensors_event_t sp;
//    static sensors_event_t st;
//    static sensors_event_t sh;
//    ms8607.getEvent(&sp, &st, &sh);
//    payload.pressure_ext = sp.pressure;
//    payload.temperature_ext = st.temperature;
//    payload.humidity_ext = sh.relative_humidity;
//    payload.altitude_ext = static_cast<float>(calc_altitude_approx_slow(payload.pressure_ext));
//}

void read_ds18b20() {
    ds18b20.requestTemperaturesByAddress(dev_addr);
    payload.temperature_probe = ds18b20.getTempC(dev_addr);
}


//void read_mprls() {
//    payload.pressure_probe = mprls.readPressure();
//    payload.altitude_probe = static_cast<float>(calc_altitude_approx_slow(payload.pressure_probe));
//}

void read_m10s() {
    if (m10s.getPVT()) {
        payload.gps_time = m10s.getUnixEpoch(payload.gps_time_us, UBLOX_CUSTOM_MAX_WAIT);
        payload.gps_siv = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
        payload.gps_latitude = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        payload.gps_longitude = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
//        payload.gps_altitude = static_cast<float>(m10s.getAltitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;  // Ellipsoid
        payload.gps_altitude = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
    }
}

void construct_string() {
    payload.uptime_ms = millis();
    build_string_to(payload_str,
                    payload.band_id,
                    payload.counter,
                    payload.uptime_ms,
                    eval_state(payload.state),
                    payload.last_response,
                    payload.gps_siv,
                    payload.gps_time,
                    payload.gps_time_us,
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
    checksum_t checksum_tmp;
    char buf[2 + (2 * sizeof(checksum_t)) + 1] = "";
    String valid_str;
    valid_str.reserve(8);

    valid_str = "";
    valid_str += STR_REPR(valid.sd0);
    valid_str += STR_REPR(valid.sd1);
    valid_str += STR_REPR(valid.bme280);
    valid_str += STR_REPR(valid.bno085);
    valid_str += STR_REPR(valid.m10s);
    valid_str += STR_REPR(valid.ms8607);
    valid_str += STR_REPR(valid.mprls);
    valid_str += STR_REPR(valid.ds18b20);

//    cout << "Core, HCLK, APB1, APB2 = "
//         << SystemCoreClock << ' '
//         << HAL_RCC_GetHCLKFreq() << ' '
//         << HAL_RCC_GetPCLK1Freq() << ' '
//         << HAL_RCC_GetPCLK2Freq() << ' '
//         << "\r\n";

    calc_checksum(&payload, &checksum_tmp);

    switch (sizeof(checksum_t)) {
        case 1: // uint8_t
            snprintf(buf, sizeof(buf), "0x%02X", checksum_tmp);
            break;
        case 2: // uint16_t
            snprintf(buf, sizeof(buf), "0x%04hX", checksum_tmp);
            break;
        case 4: // uint32_t
            snprintf(buf, sizeof(buf), "0x%08lX", checksum_tmp);
            break;
        case 8: { // uint64_t
            snprintf(buf, sizeof(buf), "0x%08lX%08lX",
                     uint32_t(checksum_tmp >> 32),
                     uint32_t(checksum_tmp & 0x00000000FFFFFFFFull));
            break;
        }
    }

    cout << "VAL=" << valid_str
         << ", FRM=" << free_memory()
         << ", CHK=" << buf
         << ", DAT=" << payload_str
         << "\r\n";
}

void write_comm() {
#ifndef SIMPLE_RXTX
    calc_checksum(&payload, &checksum);
    SerialLoRa.write(START_SEQ_DAT, sizeof(START_SEQ_DAT));
    SerialLoRa.write(reinterpret_cast<uint8_t *>(&checksum), sizeof(checksum_t));
    SerialLoRa.write(reinterpret_cast<uint8_t *>(&payload), sizeof(struct mcu0_data));
#else
    SerialLoRa.println(payload_str);
#endif
}

void get_valid_altitude(float &altitude) {
    static in_place_buffer<float, 5> gps_alts;

    while (!gps_alts.push(payload.gps_altitude));

    if (gps_alts.average() != payload.gps_altitude && validate_altitude(payload.gps_altitude))
        altitude = payload.gps_altitude;
    if (validate_altitude(payload.altitude_ext))
        altitude = payload.altitude_ext;

    // Default altitude from pressure port sensor, inaccurate but wouldn't die, right?
    altitude = payload.altitude_probe;
}

void calculate_state() {
    static in_place_buffer<float, STATE_SAMPLE_CNT> alts;
    static float max_alt;
    get_valid_altitude(payload.altitude_valid);
    if (payload.altitude_valid > max_alt) max_alt = payload.altitude_valid;
    if (!alts.push(payload.altitude_valid)) return;

    float alt_avg = alts.average();

    switch (static_cast<state_t>(payload.state)) {
        case state_t::PREPARATION:  // 0
            // Always transfer to AIRBORNE_READY
            payload.state = state_t::AIRBORNE_READY;
            break;

        case state_t::AIRBORNE_READY:  // 1
            if (alt_avg > HALF_BURST_ALTITUDE)
                payload.state = state_t::ASCENT_TO_MAX;
            else if (alt_avg > FLOOR_ALTITUDE)
                payload.state = state_t::ASCENT_TO_HALF;
            break;

        case state_t::ASCENT_TO_HALF:  // 2
            if (alt_avg > HALF_BURST_ALTITUDE)
                payload.state = state_t::ASCENT_TO_MAX;
            break;

        case state_t::ASCENT_TO_MAX:  // 3
            if (alts.count_if(compare_max, max_alt) > STATE_SAMPLE_CNT * 7 / 10)
                payload.state = state_t::DESCENT;
            break;

        case state_t::DESCENT:  // 4
            if (alt_avg < 500)
                payload.state = state_t::DESCENT_ENB_CELL;
            break;

        case state_t::DESCENT_ENB_CELL:  // 5
            if (alts.count_if(compare_dv, alt_avg, 1.f) > STATE_SAMPLE_CNT / 2)
                payload.state = state_t::LANDED_FIXED;
            break;

        case state_t::LANDED_FIXED: { // 6
            // Accepting state, reset the MCU to restart the process
            static bool triggered = false;
            if (!triggered) {
                scheduler.get_task(write_comm) = task_t(write_comm, 2 * SET_INT(COMM_INT_LORA), millis);
                triggered = true;
            }
            break;
        }

        case state_t::INVALID:  // 255 (entry)
            if (alt_avg <= FLOOR_ALTITUDE)
                payload.state = state_t::PREPARATION;
            else if (alt_avg <= HALF_BURST_ALTITUDE)
                payload.state = state_t::ASCENT_TO_HALF;
            else
                payload.state = state_t::ASCENT_TO_MAX;
            break;
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
        if (c != static_cast<char>(stream.read())) {
            if (c != '\r' && c != '\n') return;  // Ignore CR and LF characters
        } else {
            ++rx_cnt;
        }
    }

    // Cancel if rx_cnt is less than 5
    if (rx_cnt < 5) return;

    payload.last_response = c;

    switch (c) {
        case '0': // Turn "off" LEDs toggling and LEDs
            scheduler.disable(blink_leds);
            LED_OFF();
            BUZZER_OFF();
            timer_led.pause();
            analogWrite(PIN_LED, 0);
            break;

        case '1': // Turn "on" LEDs toggling
            scheduler.enable(blink_leds);
            timer_led.resume();
            break;

        case '2': // Turn "on" static LEDs
            scheduler.disable(blink_leds);
            LED_ON();
            BUZZER_ON();
            timer_led.pause();
            analogWrite(PIN_LED, MY_ADC_MAX_VALUE - 1);
            break;

        case '3': // set LoRa interval at half interval
            scheduler.get_task(write_comm) = task_t(write_comm, SET_INT(COMM_INT_LORA / 2), millis);
            break;

        case '4': // set LoRa interval at full interval
            scheduler.get_task(write_comm) = task_t(write_comm, SET_INT(COMM_INT_LORA), millis);
            break;

        case '5': // set LoRa interval at double interval
            scheduler.get_task(write_comm) = task_t(write_comm, 2 * SET_INT(COMM_INT_LORA), millis);
            break;

        case 'c': // fall through
        case 'C': // Balloon separation command
            servo_delayer.reset();
            SERVO_ON();
            break;

        case 'u': // fall through
        case 'U': // CANCEL Balloon separation command
            SERVO_OFF();
            break;

        case 'a': // fall through
        case 'A': // Airborne 2G
            m10s.setDynamicModel(DYN_MODEL_AIRBORNE2g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
            break;

        case 'b': // fall through
        case 'B': // Airborne 4G
            m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
            break;

        case 'h': // fall through
        case 'H': // Hard reset GNSS module
            m10s.hardReset();
            break;

        case 's': // fall through
        case 'S': // List SD card files

#ifndef SIMPLE_RXTX
            stream.write(START_SEQ_CMD, sizeof(START_SEQ_DAT));
#endif

            if (valid.sd0) {
                stream.println("SD0 Files");
                list_files(sd0, "/", stream);
            }
            if (valid.sd1) {
                stream.println("SD1 Files");
                list_files(sd1, "/", stream);
            }
            break;

        case 'd': // fall through
        case 'D': // Delete all files in all SD Card (All files will be lost!)
            if (valid.sd0) {
                sd0_file.close();
                delete_all_in(sd0);
            }
            if (valid.sd1) {
                sd0_file.close();
                delete_all_in(sd1);
            }
            if (valid.sd0 || valid.sd1) {
                __NVIC_SystemReset();
            }
            break;

        case 'i': // fall through
        case 'I': // List I2C devices on the bus
            i2cdetect(0, 127, Wire, stream);
            break;

        case 'l': // fall through
        case 'L': // Config LoRa module to the preset value
            config_lora();
            break;

        case 'r': // fall through
        case 'R': // Perform System Reset
            if (valid.sd0) {
                sd0_file.close();
            }
            if (valid.sd1) {
                sd0_file.close();
            }
            __NVIC_SystemReset();


        case 'x': // fall through
        case 'X': // Do nothing and reset latest response
            break;

        default: // Invalid command
            payload.last_response = 'z';
            break;
    }
}

/*
 * Generates I2C devices list table (similar to i2cdetect command)
 */
void i2cdetect(uint8_t first, uint8_t last, TwoWire &wire, Stream &stream) {
    static uint8_t resp;
    static char buf[10];

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
