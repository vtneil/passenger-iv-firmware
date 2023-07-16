#include <string.h>
#include <Arduino.h>
#include "vt_tools"
#include "vt_lora"
#include "vt_serializer"
#include "psg_4_definitions.h"

//#define SIMPLE_RX

using namespace vt;

HardwareSerial Serial1(PA_10, PA_9);
HardwareSerial &SerialLoRa = Serial1;
lora_e22 lora(SerialLoRa, 0, 0);

#ifndef SIMPLE_RX
uint8_t payload_buf[sizeof(struct mcu0_data)];
uint8_t compare_buf[sizeof(START_SEQ)];
String payload_str;

extern void add_to_buf(uint8_t, uint8_t *, size_t);

extern void handle_data(const struct mcu0_data &);
#endif

void setup() {
    // Enable onboard LED
    pinMode(PIN_BOARD_LED, OUTPUT);

    // USB CDC Serial
    Serial.begin();

    // UARTs
    lora.begin(115200u);

#ifndef SIMPLE_RX
    // String Reserve
    payload_str.reserve(PAYLOAD_STR_MAX_LEN);
#endif
}

void loop() {
#ifndef SIMPLE_RX
    static size_t i = 0;
    static bool is_receiving = false;

    while (SerialLoRa.available()) {
        uint8_t b = SerialLoRa.read();

        if (!is_receiving) {
            // Watch for start sequence.
            if (i < sizeof(START_SEQ) && b == START_SEQ[i]) {
                ++i;
                if (i == sizeof(START_SEQ)) {
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
                    i = 0;
                    is_receiving = false;
                }

                // Last index is met, use that data.
                // Data might be invalid (CAREFUL!)
                if (is_receiving && i == sizeof(struct mcu0_data)) {
                    handle_data(serializer<struct mcu0_data>::deserialize(payload_buf));
                    i = 0;
                    is_receiving = false;
                }
            }
        }
    }
#else
    while (SerialLoRa.available()) Serial.write(SerialLoRa.read());
#endif
}

#ifndef SIMPLE_RX
void add_to_buf(uint8_t b, uint8_t *buf, size_t n) {
    for (size_t i = 0; i < n - 1; ++i) buf[i] = buf[i + 1];
    buf[n - 1] = b;
}

inline void handle_data(const struct mcu0_data &payload) {
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

    Serial.print(payload_str);
    Serial.print('\n');
}
#endif
