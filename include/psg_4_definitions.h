#ifndef PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H
#define PASSENGER_IV_FIRMWARE_PSG_4_DEFINITIONS_H

// Common Configurations
#define USE_HSE
#define CLOCK_SPEED_SETTINGS    100
#define ENABLE_SENSORS
//#define SIMPLE_RXTX

// Tool Macros
#define STR_REPR(V)             ((V) ? "#" : ".")
#define DIV_CEIL(X, Y)          (1 + ((X - 1) / Y))
#define DIV_FLOOR(X, Y)         (X / Y)
#define MOD_OP(X, Y)            (X % Y)

// Target Altitude
#define FLOOR_ALTITUDE          (250.f)
#define TARGET_BURST_ALTITUDE   (40000.f)
#define HALF_BURST_ALTITUDE     (TARGET_BURST_ALTITUDE / 2.f)

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

// States
enum class state_t : uint8_t {
    PREPARATION = 0,
    AIRBORNE_READY,
    ASCENT_TO_HALF,
    ASCENT_TO_MAX,
    DESCENT,
    DESCENT_ENB_CELL,
    LANDED_FIXED,
    INVALIDATE = 255
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
    uint32_t uptime;
    state_t state;

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

// Transmission Sequences
constexpr uint8_t START_SEQ[4] = {0xff, 0xff, 0xff, 0xff};

// Checksum type: uint8_t (1 byte), uint16_t (2 byte), uint32_t (4 byte), or uint64_t (8 byte)
using checksum_t = uint64_t;

template<typename OutputType = uint8_t, typename InputType>
void calc_checksum(const InputType &input_data, OutputType &result) {
    using byte_t = uint8_t;
    constexpr size_t q = DIV_FLOOR(sizeof(InputType), sizeof(OutputType));
    constexpr size_t r = MOD_OP(sizeof(InputType), sizeof(OutputType));
    static const OutputType *ptr;

    ptr = reinterpret_cast<const OutputType *>(&input_data);

    // Clear result
    memset(&result, 0, sizeof(result));

    // Full XOR
    for (size_t i = 0; i < q; ++i) {
        result ^= *ptr++;
    }

    // Partial XOR for remainder
    for (size_t i = 0; i < r; ++i) {
        *(reinterpret_cast<byte_t *>(&result) + i) ^= *(reinterpret_cast<const byte_t *>(ptr) + i);
    }

    OutputType swapped = (result << 4 * sizeof(OutputType)) | (result >> 4 * sizeof(OutputType));
    result = (swapped << 1) | (swapped >> 1);
}

template<typename OutputType, typename InputType>
OutputType calc_checksum(const InputType &result) {
    static OutputType chk;
    calc_checksum(result, chk);
    return chk;
}

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
