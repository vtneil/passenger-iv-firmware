#ifndef PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H
#define PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H

// Clock Configurations
#define USE_HSE
#define CLOCK_SPEED_SETTINGS    100

// Common Configurations
#define SPEED_MUL               1
#define SPEED_DIV               1
#define ENABLE_SENSORS
#define ENABLE_IWDT
#define ENABLE_UPLINK
#define BLINK_STATUS
//#define SIMPLE_RXTX
//#define CONFIG_LORA

// Tool Macros
#define SET_INT(X)              (SPEED_DIV * (X) / SPEED_MUL)
#define STR_REPR(V)             ((V) ? "#" : ".")
#define DIV_CEIL(X, Y)          (1 + (((X) - 1) / (Y)))

// Target Altitude
#define FLOOR_ALTITUDE          (250.f)
#define TARGET_BURST_ALTITUDE   (40000.f)
#define HALF_BURST_ALTITUDE     (TARGET_BURST_ALTITUDE / 2.f)

// ADCs
#define MY_ADC_RESOLUTION       (12)
#define MY_ADC_MAX_VALUE        (1 << MY_ADC_RESOLUTION)

// Servo
#define TIME_SERVO_CUT          (20ul * 1000ul)

// Watchdog Timer Parameters
#define WDT_INT_SAFE_MODE       (4000ul * 1000ul)
#define WDT_INT_OPS_MODE        (2000ul * 1000ul)
#define WDT_INT_TIMEOUT         WDT_INT_SAFE_MODE
#define WDT_INT_RELOAD          (WDT_INT_TIMEOUT / 2)

// State Updates
#define STATE_INT_UPDATE        (100ul)
#define STATE_SAMPLE_CNT        (20u)

// Communication Interval
#define COMM_INT_USB            (1000ul)
#define COMM_INT_LORA           (4000ul)

// Pins Configuration
#define PIN_GCS_BTN             PB1

#define PIN_BOARD_LED           PC13
#define PIN_LED                 PB4
#define PIN_BUZZER              PB12
//#define PIN_SERVO_CUT           PB3
#define PIN_SERVO_CUT           PB5
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

// Pin Macros
#define LED_ON()                digitalWrite(PIN_BOARD_LED, 0); \
                                digitalWrite(PIN_LED, 1)
#define LED_OFF()               digitalWrite(PIN_BOARD_LED, 1); \
                                digitalWrite(PIN_LED, 0)
#define LED_TOGGLE()            digitalToggle(PIN_BOARD_LED); \
                                digitalToggle(PIN_LED)
#define BUZZER_ON()             digitalWrite(PIN_BUZZER, 1)
#define BUZZER_OFF()            digitalWrite(PIN_BUZZER, 0)
#define BUZZER_TOGGLE()         digitalToggle(PIN_BUZZER)

//#define SERVO_OFF()             digitalWrite(PIN_SERVO_CUT, LOW)
//#define SERVO_ON()              digitalWrite(PIN_SERVO_CUT, HIGH)
#define SERVO_OFF()             servo.writeMicroseconds(1500)
#define SERVO_ON()              servo.writeMicroseconds(0)

// Other parameters
#define UBLOX_CUSTOM_MAX_WAIT   (250u)
#define BME280_ADDRESS          (0x77)

#define I2C_CLOCK_STANDARD      (100000u)
#define I2C_CLOCK_FAST          (400000u)
#define I2C_CLOCK_FAST_PLUS     (1000000u)
#define I2C_CLOCK_HIGH_SPEED    (3400000u)

#define I2C_CLOCK_SPEED         (I2C_CLOCK_FAST)

// MCU Data
#define PAYLOAD_STR_MAX_LEN     (256u)
#define PAYLOAD_COMM_MAX_CNT    (2)

// States
enum class state_t : uint8_t {
    PREPARATION = 0,
    AIRBORNE_READY,
    ASCENT_TO_HALF,
    ASCENT_TO_MAX,
    DESCENT,
    DESCENT_ENB_CELL,
    LANDED_FIXED,
    INVALID = 255
};

uint8_t eval_state(state_t state) {
    return static_cast<uint8_t>(state);
}

state_t eval_state(uint8_t state) {
    return static_cast<state_t>(state);
}

struct mcu0_data {
    uint8_t band_id;
    uint32_t counter;
    uint32_t uptime_ms;
    state_t state;
    char last_response;

    uint8_t gps_siv;
    uint32_t gps_time;
    uint32_t gps_time_us;
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

    float altitude_valid;
    uint32_t batt_volt;
};

struct mcu1_data {
    uint8_t band_id;
    uint32_t counter;

    uint8_t siv;
    uint32_t gps_time;
    double latitude;
    double longitude;
    float altitude;
};

// Transmission Sequences (each byte must not overlap)
constexpr uint8_t START_SEQ_DAT[4] = {0xff, 0xff, 0xff, 0xff};
constexpr uint8_t START_SEQ_CMD[4] = {0xee, 0xee, 0xee, 0xee};

// Checksum type: uint8_t (1 byte), uint16_t (2 byte), uint32_t (4 byte), or uint64_t (8 byte)
//using checksum_t = uint8_t;
//using checksum_t = uint16_t;
//using checksum_t = uint32_t;
using checksum_t = uint64_t;

template<typename InputType, typename OutputType>
struct calc_checksum_t {
    void operator()(const void *const input_ptr, void *const result) const {
        calc_checksum_impl(*reinterpret_cast<const InputType *const>(input_ptr),
                           reinterpret_cast<OutputType *const>(result));
    }

protected:
    __attribute__((always_inline))
    void calc_checksum_impl(const InputType &input_data, OutputType *const result) const {
        static_assert(sizeof(InputType) >= sizeof(OutputType), "Can\'t calculate checksum of this data type.");

        using byte_t = uint8_t;
        constexpr OutputType ZERO = 0;
        constexpr OutputType ONE = 1;
        constexpr OutputType MASK = ZERO - ONE;
        constexpr size_t Q = sizeof(InputType) / sizeof(OutputType);
        constexpr size_t R = sizeof(InputType) % sizeof(OutputType);
        constexpr size_t S = DIV_CEIL(sizeof(InputType), sizeof(OutputType));
        constexpr size_t BITS_SHIFT = 4 * sizeof(OutputType);
        constexpr OutputType MASK_H = MASK << BITS_SHIFT;
        constexpr OutputType MASK_L = ~MASK_H;

        // Initialization
        byte_t input_as_byte_t[sizeof(InputType)] = {};
        memcpy(input_as_byte_t, &input_data, sizeof(InputType));

        byte_t result_as_byte_t[sizeof(OutputType)] = {};
        OutputType result_tmp;

        // Full XOR
        for (size_t i = 0; i < Q; ++i)
            *reinterpret_cast<OutputType *const>(result_as_byte_t) ^= reinterpret_cast<const OutputType *const>(input_as_byte_t)[i];

        // Partial XOR for remainder
        for (size_t i = 0; i < R; ++i)
            result_as_byte_t[i] ^= input_as_byte_t[Q * sizeof(OutputType) + i];
        result_tmp = *reinterpret_cast<const OutputType *const>(result_as_byte_t);

        // XOR Shifting Left to Right
        for (size_t i = 8 * sizeof(OutputType) - 1; i > 0; --i)
            result_tmp ^= result_tmp << i;
        for (size_t i = 1; i < 8 * sizeof(OutputType); ++i)
            result_tmp ^= result_tmp >> i;

        // XOR with its reverse bits
        OutputType result_rev_tmp = {};
        for (size_t i = 0; i < 8 * sizeof(OutputType); ++i)
            if (result_tmp & (ONE << i))
                result_rev_tmp |= 1 << ((8 * sizeof(OutputType) - 1) - i);
        result_tmp ^= result_rev_tmp;

        // Swap first half bytes/nibbles with not second half bytes/nibbles
        result_tmp = ((result_tmp & MASK_L) << BITS_SHIFT) | ((result_tmp & MASK_H) >> BITS_SHIFT);

        // Invert bits
        result_tmp = ~result_tmp;

        // Return value
        *result = result_tmp;
    }
};

calc_checksum_t<struct mcu0_data, checksum_t> calc_checksum = calc_checksum_t<struct mcu0_data, checksum_t>();

// System Clock Configurations
extern "C" void SystemClock_Config(void) {
#ifdef USE_HSE

#if CLOCK_SPEED_SETTINGS == 100
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
#elif CLOCK_SPEED_SETTINGS == 50
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
#else // Invalid Clock
    static_assert(false, "Invalid Clock! Cannot compile");
#endif

#else // HSI
#if CLOCK_SPEED_SETTINGS == 100
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
#elif CLOCK_SPEED_SETTINGS == 50
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 50;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
#else // Invalid Clock
    static_assert(false, "Invalid Clock! Cannot compile");
#endif

#endif // USE_HSE
}

#endif //PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H
