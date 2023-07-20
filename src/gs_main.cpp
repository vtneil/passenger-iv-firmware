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
uint8_t compare_buf[sizeof(START_SEQ)];
String payload_str;

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

    scheduler.add_task([]() -> void { digitalToggle(PIN_BOARD_LED); }, 1000u, millis);
}

void loop() {
#ifndef SIMPLE_RXTX
    static size_t i = 0;
    static bool is_receiving = false;

    while (SerialLoRa.available()) {
        digitalToggle(PIN_BOARD_LED);

        uint8_t b = SerialLoRa.read();

        if (!is_receiving) {
            // Watch for start sequence.
            if (b == START_SEQ[i]) {
                ++i;
                if (i == sizeof(START_SEQ)) {
                    // Reset Index, go to receive mode
                    i = 0;
                    is_receiving = true;
                }
            } else {
                i = 0; // Reset if sequence is broken.
            }

        } else {
            // Start copying data into buffer if start sequence is found.
            // Now receiving mode: check for end sequence.
            if (i < sizeof(struct mcu0_data)) {
                // Receiving into buffer
                payload_buf[i++] = b;

                // Add to compare buffer
                add_to_buf(b, compare_buf, sizeof(compare_buf));

                // Compare latest data with start sequence and reset if found.
                if (0 == memcmp(START_SEQ, compare_buf, sizeof(START_SEQ))) {
                    // Reset Index, go to start mode
                    i = 0;
                    is_receiving = false;
                }

                // Last index is met, use that data.
                // Data might be invalid (CAREFUL!)
                if (is_receiving && i == sizeof(struct mcu0_data)) {

                    // Checksum
                    checksum_t checksum_ref = *reinterpret_cast<checksum_t *>(payload_buf);
                    checksum_t checksum_calc = calc_checksum<checksum_t>(
                            *reinterpret_cast<struct mcu0_data *>(payload_buf + sizeof(checksum_t))
                    );
                    if (checksum_ref == checksum_calc) {
                        handle_data(
                                *reinterpret_cast<struct mcu0_data *>(payload_buf + sizeof(checksum_t)),
                                checksum_ref
                        );
                    }

                    // Reset Index, go to start mode
                    i = 0;
                    is_receiving = false;
                }
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

    Serial.print("0x");
    Serial.print(checksum, 16);
    Serial.print(",");
    Serial.print(payload_str);
    Serial.print('\n');
}

#endif
