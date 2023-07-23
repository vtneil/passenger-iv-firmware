#include <string.h>
#include <Arduino.h>
#include "vt_tools"
#include "vt_lora"
#include "psg_4_definitions.h"

//#define CONFIG_LORA

#define FORWARD_COMM

using namespace vt;

//HardwareSerial Serial2(PA_3, PA_2);
HardwareSerial &SerialLoRa = Serial2;
lora_e22 lora(SerialLoRa, GCS_PIN_LORA_M0, GCS_PIN_LORA_M1);

#ifndef SIMPLE_RXTX
uint8_t payload_buf[sizeof(checksum_t) + sizeof(struct mcu0_data)];
uint8_t compare_buf[sizeof(START_SEQ_DAT)];
String payload_str;
checksum_t checksum_calc;

enum class rx_mode_t : uint8_t {
    RX_WAIT = 0,
    RX_DAT,
    RX_CMD
};

task_scheduler<1> scheduler;

extern void add_to_buf(uint8_t, uint8_t *, size_t);

extern void handle_data(const struct mcu0_data &, const checksum_t &);

#endif

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

    scheduler.add_task([]() -> void { digitalToggle(PIN_BOARD_LED); }, SET_INT(1000u), millis);
}

void loop() {
#ifndef SIMPLE_RXTX
    static size_t i = 0;
    static rx_mode_t rx_mode = rx_mode_t::RX_WAIT;
    static uint8_t b;
    static checksum_t checksum_ref;

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
                if (i < sizeof(struct mcu0_data)) {
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
                    if (i == sizeof(struct mcu0_data)) {
                        // Checksum
                        checksum_ref = *reinterpret_cast<checksum_t *>(payload_buf);
                        calc_checksum<checksum_t>(
                                *reinterpret_cast<struct mcu0_data *>(payload_buf + sizeof(checksum_t)),
                                checksum_calc
                        );
                        if (checksum_ref == checksum_calc) {
                            handle_data(
                                    *reinterpret_cast<struct mcu0_data *>(payload_buf + sizeof(checksum_t)),
                                    checksum_ref
                            );
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
}

#ifndef SIMPLE_RXTX

void add_to_buf(uint8_t b, uint8_t *buf, size_t n) {
    for (size_t i = 0; i < n - 1; ++i) buf[i] = buf[i + 1];
    buf[n - 1] = b;
}

inline void handle_data(const struct mcu0_data &payload, const checksum_t &checksum) {
    static char buf[2 + (2 * sizeof(checksum_t)) + 1] = "";

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

    switch (sizeof(checksum_t)) {
        case 1: // uint8_t
            snprintf(buf, sizeof(buf), "0x%02X", checksum);
            break;
        case 2: // uint16_t
            snprintf(buf, sizeof(buf), "0x%04X", checksum);
            break;
        case 4: // uint32_t
            snprintf(buf, sizeof(buf), "0x%08lX", checksum);
            break;
        case 8: // uint64_t
            snprintf(buf, sizeof(buf), "0x%016llX", checksum);
            break;
    }
    Serial.print(buf);
    Serial.print(",");
    Serial.print(payload_str);
    Serial.print('\n');
}

#endif
