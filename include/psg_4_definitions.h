#ifndef PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H
#define PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H

// Common Configurations
//#define SIMPLE_RXTX

// Tool Macros
#define STR_REPR(V) ((V) ? "#" : ".")
#define DIV_ROUND_UP(X, Y) (1 + ((X - 1) / Y))
#define DIV_ROUND(X, Y) (X / Y)
#define MOD_OP(X, Y) (X % Y)

// Target Altitude
#define TARGET_BURST_ALTITUDE   (40000.f)

// Watchdog Timer Parameters
#define WDT_INT_SAFE_MODE       (4000ul * 1000ul)
#define WDT_INT_OPS_MODE        (2000ul * 1000ul)
#define WDT_INT_TIMEOUT         WDT_INT_OPS_MODE
#define WDT_INT_RELOAD          (WDT_INT_TIMEOUT / 2)

// State Updates
#define STATE_INT_UPDATE        (100ul)

// Pins Configuration
#define PIN_BOARD_LED           PC13
#define PIN_LED                 PB4
#define PIN_CTRL_CUT            PB5
#define PIN_VBATT               PA1

#define PIN_I2C_SCL1            PB6
#define PIN_I2C_SDA1            PB7

#define PIN_SPI_SCLK1           PA5
#define PIN_SPI_MISO1           PA6
#define PIN_SPI_MOSI1           PA7
#define PIN_SPI_CS_SD_INT       PA4
#define PIN_SPI_CS_SD_EXT       PB1
#define SPI_SPEED_SD_MHZ        (20)

#define LORA_CHANNEL            (33)

#define PIN_LORA_M1             PB14
#define PIN_LORA_M0             PB15

#define GCS_PIN_LORA_M1         PB4
#define GCS_PIN_LORA_M0         PB3

#define EXT_TEMP_PIN            PA8

// Other parameters
#define UBLOX_CUSTOM_MAX_WAIT   (250u)
#define BME280_ADDRESS          (0x77)

#define I2C_CLOCK_STANDARD      (100000u)
#define I2C_CLOCK_FAST          (400000u)
#define I2C_CLOCK_FAST_PLUS     (1000000u)
#define I2C_CLOCK_HIGH_SPEED    (3400000u)

// MCU Data
#define PAYLOAD_STR_MAX_LEN     (256u)

struct mcu0_data {
    uint8_t band_id;
    uint32_t counter;
    uint8_t state;
    uint32_t uptime;

    uint32_t gps_time;
    uint32_t gps_time_us;
    uint8_t gps_siv;
    double gps_latitude;
    double gps_longitude;
    float gps_altitude;

    double acc_x;
    double acc_y;
    double acc_z;
    double eul_x;
    double eul_y;
    double eul_z;

    float pressure_int;
    float humidity_int;
    float temperature_int;

    float pressure_ext;
    float humidity_ext;
    float temperature_ext;
    float altitude_ext;

    float pressure_probe;
    float temperature_probe;
    float altitude_probe;

    uint32_t batt_volt;
};

// States
enum class GlobalState : uint8_t {
    PREPARATION = 0,
    AIRBORNE_READY,
    ASCENT_SUB_HALF,
    ASCENT_SUB_MAX,
    DESCENT,
    DESCENT_ENB_CELL,
    LANDED_FIXED,
    INVALIDATE = 255
};

// Transmission Sequences
constexpr uint8_t START_SEQ[4] = {0xff, 0xff, 0xff, 0xff};

using checksum_t = uint16_t;

//const uint8_t START_SEQ[4] = {0xaa, 0xaa, 0xaa, 0xaa};

template<typename OutputType = uint8_t, typename InputType>
OutputType calc_checksum(InputType &data) {
    using byte_t = uint8_t;
    constexpr size_t q = DIV_ROUND(sizeof(InputType), sizeof(OutputType));
    constexpr size_t r = MOD_OP(sizeof(InputType), sizeof(OutputType));
    OutputType *ptr = reinterpret_cast<OutputType *>(&data);
    OutputType chk = 0x0;
    // Full XOR
    for (size_t i = 0; i < q; ++i) {
        chk ^= *ptr++;
    }

    // Partial XOR for remainder
    for (size_t i = 0; i < r; ++i) {
        *(reinterpret_cast<byte_t *>(&chk) + i) ^= *(reinterpret_cast<byte_t *>(ptr) + i);
    }
    return ~chk;
}

#endif //PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H
