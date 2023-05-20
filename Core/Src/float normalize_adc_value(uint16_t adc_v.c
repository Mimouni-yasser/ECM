#include<stdio.h>
#include<stdint.h>
float normalize_adc_value(uint16_t adc_val, float min_val, float max_val) {
    float adc_range = 4095.0f;  // Maximum ADC value (12-bit resolution)
    float val_range = max_val - min_val;
    float val = ((float)adc_val / adc_range) * val_range + min_val;
    return val;
}

int main(){
    printf("%f",normalize_adc_value(4000,-40,100));
    return 0;
}