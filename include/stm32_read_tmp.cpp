#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"

uint16_t adc_read(uint32_t channel) {
    ADC_HandleTypeDef AdcHandle = {};
    ADC_ChannelConfTypeDef AdcChannelConf = {};
    volatile uint16_t uhADCxConvertedValue = 0;

    AdcHandle.Instance = ADC1;
    AdcHandle.State = HAL_ADC_STATE_RESET;
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ScanConvMode = DISABLE;
    AdcHandle.Init.ContinuousConvMode = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    AdcHandle.Init.NbrOfConversion = 1;
    AdcHandle.Init.NbrOfDiscConversion = 0;

    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        return 0;
    }

    AdcChannelConf.Channel = channel;
    AdcChannelConf.Rank = LL_ADC_REG_RANK_1;
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    /*##-2- Configure ADC regular channel ######################################*/
    if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK) {
        /* Channel Configuration Error */
        return 0;
    }

    if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
        return 0;
    }

    if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
        uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
    }

    if (HAL_ADC_Stop(&AdcHandle) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
        return 0;
    }

    return uhADCxConvertedValue;
}