#ifndef FIELD_SENSOR_ADC_H
#define FIELD_SENSOR_ADC_H

#include "robot_auto_mode.h"

void field_sensor_adc_init(void);
unsigned int field_sensor_adc_read_channel(unsigned int channel);
void field_sensor_adc_read_raw(field_data_t *sensors);
void field_sensor_adc_update(field_data_t *sensors);

#endif
