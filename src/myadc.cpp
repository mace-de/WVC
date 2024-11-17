#include <Arduino.h>

void adc_config(void)
{
  rcu_periph_clock_enable(RCU_ADC);
  rcu_adc_clock_config(RCU_ADCCK_APB2_DIV4);
  /* ADC data alignment config */
  adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
  /* ADC channel length config */
  adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);
  /* ADC trigger config */
  adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
  /* ADC external trigger config */
  adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
  /* enable ADC interface */
  adc_enable();
  delay(1U);
  /* ADC calibration and reset calibration */
  adc_calibration_enable();
}

/*!
    \brief      ADC channel sample
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t adc_channel_sample(uint8_t channel)
{
  /* ADC regular channel config */
  adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_7POINT5);
  /* ADC software trigger enable */
  adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
  /* wait the end of conversion flag */
  while (!adc_flag_get(ADC_FLAG_EOC))
    ;
  /* clear the end of conversion flag */
  adc_flag_clear(ADC_FLAG_EOC);
  /* return regular channel sample value */
  return (adc_regular_data_read());
}