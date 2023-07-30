#include <string.h>
#include <Arduino.h>
#include "vt_tools"
#include "vt_lora"
#include "psg_4_definitions.h"

//#define CONFIG_LORA

#define MAX_NUM_TX              (10)
#define MAX_INT_TX              (500)
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

task_scheduler<5> scheduler;

smart_delay sd_btn(5000, millis);
smart_delay sd_key(2000, millis);
uint8_t btn_cnt;
uint8_t key_cnt;

void setup() {
    // Enable onboard LED
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, !LOW);

    // Buttons Pinout
    pinMode(GCS_PIN_BTN, INPUT);
    pinMode(GCS_PIN_KEY, INPUT);

    // USB CDC Serial
    Serial.begin();

    // Intentional delay for exactly 1000 ms
    delayMicroseconds(1000ul * 1000ul);

    // Config LoRa
    if (!digitalRead(GCS_PIN_KEY)) {
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
    }
    while (!digitalRead(GCS_PIN_KEY));

    // Prevent button accidents
    while (digitalRead(GCS_PIN_BTN));

    // LoRa
    lora.begin(115200u);

#ifndef SIMPLE_RXTX
    // String Reserve
    payload_str.reserve(PAYLOAD_STR_MAX_LEN);
#endif

    // Placeholder (blank)
    scheduler.add_task([]() -> void {}, 10000ul, millis);

    // Button
    btn_cnt = 0;
    // 1
    scheduler.add_task([]() -> void {
        if (1 <= btn_cnt && btn_cnt <= MAX_NUM_TX) {
            SerialLoRa.print("CCCCC");
            ++btn_cnt;
        }
    }, MAX_INT_TX, millis);
    scheduler.disable(1);

    // Board Button
    key_cnt = 0;
    // 2
    scheduler.add_task([]() -> void {
        if (1 <= key_cnt && key_cnt <= MAX_NUM_TX) {
            SerialLoRa.print("UUUUU");
            ++btn_cnt;
        }
    }, MAX_INT_TX, millis);
    scheduler.disable(2);

    // Blink Slow
    // 3
    scheduler.add_task([]() -> void {
        digitalToggle(PIN_BOARD_LED);
    }, SET_INT(250u), millis);

    // Blink Fast
    // 4
    scheduler.add_task([]() -> void {
        digitalToggle(PIN_BOARD_LED);
    }, SET_INT(125u), millis);
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

                        // Checksum compare
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

        digitalToggle(PIN_BOARD_LED);
    }
#else
    while (SerialLoRa.available()) Serial.write(SerialLoRa.read());
#endif

#ifdef FORWARD_COMM
    while (Serial.available()) SerialLoRa.write(Serial.read());
#endif

    scheduler.exec();

    int read_btn = digitalRead(GCS_PIN_BTN);
    int read_key = !digitalRead(GCS_PIN_KEY);

    switch ((read_btn << 1) | read_key) {
        case 0b00:
            btn_cnt = 0;
            key_cnt = 0;
            scheduler.disable(1);
            scheduler.disable(2);
            scheduler.enable(3);
            scheduler.disable(4);
            break;

        case 0b10: {
            if (btn_cnt == 0) {
                scheduler.disable(3);
                digitalWrite(PIN_BOARD_LED, !HIGH);
                sd_btn.reset();
                btn_cnt = 1;
            } else if (btn_cnt == 1) {
                if (sd_btn) {
                    scheduler.enable(1);
                    scheduler.disable(3);
                    scheduler.enable(4);
                }
            } else if (btn_cnt > MAX_NUM_TX) {
                scheduler.disable(1);
                scheduler.enable(3);
                scheduler.disable(4);
            }
            break;
        }

        case 0b01: {
            if (key_cnt == 0) {
                scheduler.disable(3);
                digitalWrite(PIN_BOARD_LED, !LOW);
                sd_key.reset();
                key_cnt = 1;
            } else if (key_cnt == 1) {
                if (sd_key) {
                    scheduler.enable(2);
                    scheduler.disable(3);
                    scheduler.enable(4);
                }
            } else if (key_cnt > MAX_NUM_TX) {
                scheduler.disable(2);
                scheduler.enable(3);
                scheduler.disable(4);
            }
            break;
        }

        case 0b11:
        default:
            scheduler.disable(1);
            scheduler.disable(2);
            break;
    }
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
