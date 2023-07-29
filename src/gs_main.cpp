#include <string.h>
#include <Arduino.h>
#include "vt_tools"
#include "vt_lora"
#include "psg_4_definitions.h"

//#define CONFIG_LORA

#define FORWARD_COMM
//#define PRINT_CHECKSUM

using namespace vt;

//HardwareSerial Serial2(PA_3, PA_2);
HardwareSerial &SerialLoRa = Serial2;
lora_e22 lora(SerialLoRa, GCS_PIN_LORA_M0, GCS_PIN_LORA_M1);

#ifndef SIMPLE_RXTX
uint8_t payload_buf[sizeof(checksum_t) + sizeof(struct mcu0_data)];
uint8_t compare_buf[sizeof(START_SEQ_DAT)];
String payload_str;
checksum_t checksum_dst;

enum class rx_mode_t : uint8_t {
    RX_WAIT = 0,
    RX_DAT,
    RX_CMD
};

extern void add_to_buf(uint8_t, uint8_t *, size_t);

extern void handle_data(const struct mcu0_data &, const checksum_t &);

#endif

task_scheduler<3> scheduler;

smart_delay sd100(100, millis);
smart_delay sd5000(5000, millis);
uint8_t btn_cnt;

void setup() {
    // Enable onboard LED
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, !HIGH);

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

    // LoRa
    lora.begin(115200u);

#ifndef SIMPLE_RXTX
    // String Reserve
    payload_str.reserve(PAYLOAD_STR_MAX_LEN);
#endif

    // Placeholder
    scheduler.add_task([]() -> void {}, 10000ul, millis);

    // Button
//    btn_cnt = 0;
//    scheduler.add_task([]() -> void {
//        if (1 <= btn_cnt && btn_cnt <= 5) {
//            SerialLoRa.print("CCCCC");
//            ++btn_cnt;
//        } else {
//            btn_cnt = 0;
//        }
//    }, 1000, millis);
//    scheduler.disable(1);

    // Blink
    scheduler.add_task([]() -> void {
        digitalToggle(PIN_BOARD_LED);
    }, SET_INT(250u), millis);
}

void loop() {
#ifndef SIMPLE_RXTX
    static size_t i = 0;
    static rx_mode_t rx_mode = rx_mode_t::RX_WAIT;
    static uint8_t b;
    static checksum_t checksum_src;
    static struct mcu0_data data_data;

    while (SerialLoRa.available()) {
        digitalToggle(PIN_BOARD_LED);

        b = SerialLoRa.read();

        switch (rx_mode) {
            case rx_mode_t::RX_WAIT: {
                // Watch for start sequence.
                if (b == START_SEQ_DAT[i]) {
                    ++i;
                    if (i == sizeof(START_SEQ_DAT)) {
                        // Reset Index, go to data receive mode
                        i = 0;
                        rx_mode = rx_mode_t::RX_DAT;
                    }
                } else if (b == START_SEQ_CMD[i]) {
                    ++i;
                    if (i == sizeof(START_SEQ_CMD)) {
                        // Reset Index, go to command receive mode
                        i = 0;
                        rx_mode = rx_mode_t::RX_CMD;
                    }
                } else {
                    i = 0; // Reset if sequence is broken.
                }
                break;
            }

            case rx_mode_t::RX_DAT: {
                // Start copying data into buffer if start sequence is found.
                // Now receiving mode: check for end sequence.
                if (i < sizeof(payload_buf)) {
                    // Receiving into buffer
                    payload_buf[i++] = b;

                    // Add to compare buffer
                    add_to_buf(b, compare_buf, sizeof(compare_buf));

                    // Compare latest data with start sequence and reset if found.
                    if (0 == memcmp(START_SEQ_DAT, compare_buf, sizeof(START_SEQ_DAT))) {
                        // Reset Index, go to start mode
                        i = 0;
                        rx_mode = rx_mode_t::RX_WAIT;
                    }

                    // Last index is met, use that data.
                    // Data might be invalid despite correct checksum.
                    if (i == sizeof(payload_buf)) {
                        // Checksum
                        checksum_src = *reinterpret_cast<checksum_t *>(payload_buf);
                        data_data = *reinterpret_cast<struct mcu0_data *>(payload_buf + sizeof(checksum_t));
                        calc_checksum(payload_buf + sizeof(checksum_t), &checksum_dst);

                        if (checksum_src == checksum_dst) {
                            handle_data(data_data, checksum_src);
                        }

                        // Reset Index, go to start mode
                        i = 0;
                        rx_mode = rx_mode_t::RX_WAIT;
                    }
                }
                break;
            }

            case rx_mode_t::RX_CMD: {
                delayMicroseconds(20ul * 1000ul);

                Serial.write(b);
                while (SerialLoRa.available()) Serial.write(SerialLoRa.read());

                i = 0;
                rx_mode = rx_mode_t::RX_WAIT;
                break;
            }
        }
    }
#else
    while (SerialLoRa.available()) Serial.write(SerialLoRa.read());
#endif

#ifdef FORWARD_COMM
    while (Serial.available()) SerialLoRa.write(Serial.read());
#endif

    scheduler.exec();

    // Button watcher
//    if (digitalRead(PIN_GCS_BTN)) {
//        if (btn_cnt == 0) {
//            sd5000.reset();
//            btn_cnt = 1;
//        } else if (btn_cnt == 1) {
//            if (sd5000) {
//                scheduler.enable(1);
//            }
//        }
//    } else {
//        scheduler.disable(1);
//    }
}

#ifndef SIMPLE_RXTX

void add_to_buf(uint8_t b, uint8_t *buf, size_t n) {
    for (size_t i = 0; i < n - 1; ++i) buf[i] = buf[i + 1];
    buf[n - 1] = b;
}

inline void handle_data(const struct mcu0_data &payload, const checksum_t &checksum_src) {
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

#ifdef PRINT_CHECKSUM
    static char buf_ref[2 + (2 * sizeof(checksum_t)) + 1] = "";
    static char buf_calc[sizeof(buf_ref)] = "";

    switch (sizeof(checksum_t)) {
        case 1: // uint8_t
            snprintf(buf_ref, sizeof(buf_ref), "0x%02X", checksum_src);
            snprintf(buf_calc, sizeof(buf_calc), "0x%02X", checksum_dst);
            break;
        case 2: // uint16_t
            snprintf(buf_ref, sizeof(buf_ref), "0x%04hX", checksum_src);
            snprintf(buf_calc, sizeof(buf_calc), "0x%04hX", checksum_dst);
            break;
        case 4: // uint32_t
            snprintf(buf_ref, sizeof(buf_ref), "0x%08lX", checksum_src);
            snprintf(buf_calc, sizeof(buf_calc), "0x%08lX", checksum_dst);
            break;
        case 8: { // uint64_t
            snprintf(buf_ref, sizeof(buf_ref), "0x%08lX%08lX",
                     uint32_t(checksum_src >> 32),
                     uint32_t(checksum_src & 0x00000000FFFFFFFFull));
            snprintf(buf_calc, sizeof(buf_calc), "0x%08lX%08lX",
                     uint32_t(checksum_dst >> 32),
                     uint32_t(checksum_dst & 0x00000000FFFFFFFFull));
            break;
        }
    }

    Serial.print(buf_ref);
    Serial.print(",");
    Serial.print(buf_calc);
    Serial.print(",");
#endif
    Serial.print(payload_str);
    Serial.print('\n');
}

#endif
