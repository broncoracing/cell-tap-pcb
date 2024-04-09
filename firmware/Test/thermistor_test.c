#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define N_ADC_CHANNELS 15
#define N_STATUS_LEDS 12

struct SteinhartHartParameters {
  float A,B,C;
};

struct ThermistorCal_T {
  uint8_t temp_calibrations[3]; // stores whether the calibration temperatures have been read
  uint8_t abc_calibration; // stores whether the thermistor calibration values have been calculated
  // readings to 
  float temps[3];
  uint16_t adc_readings[N_ADC_CHANNELS][3];
  struct SteinhartHartParameters parameters[N_ADC_CHANNELS];
};


float compute_resistance(uint16_t adc_reading) {
  float adc_float = (float) adc_reading;
  float voltage_fac = adc_float / ((float) 0x1000); // input voltage as a fraction of VDDA
  const float ref_resistance = 10000.0; // precision reference resistor value
  float rtd_resistance = ref_resistance * ((1.0 / voltage_fac) - 1);
  return rtd_resistance;
}

float c2k(float c) {
    return c + 273.15;
}
float k2c(float k) {
    return k - 273.15;
}

struct SteinhartHartParameters compute_parameters(float r1, float t1, float r2, float t2, float r3, float t3) {
    float ln_r1 = logf(r1);
    float ln_r2 = logf(r2);
    float ln_r3 = logf(r3);

    float Y_1 = 1.0 / c2k(t1);
    float Y_2 = 1.0 / c2k(t2);
    float Y_3 = 1.0 / c2k(t3);

    float gamma_2 = (Y_2 - Y_1) / (ln_r2 - ln_r1);
    float gamma_3 = (Y_3 - Y_1) / (ln_r3 - ln_r1);

    float C = ((gamma_3 - gamma_2) / (ln_r3 - ln_r2)) * 1.0 / (ln_r1 + ln_r2 + ln_r3);

    float B = gamma_2 - C * (ln_r1 * ln_r1 + ln_r1 * ln_r2 + ln_r2 * ln_r2);

    float A = Y_1 - (B + C * ln_r1 * ln_r1) * ln_r1;

    struct SteinhartHartParameters params = {A,B,C};
    return params;
}

float compute_temperature(struct SteinhartHartParameters* p, float r) {
  float ln_r = logf(r);
  float t_inv = p->A + p->B * ln_r + p->C * ln_r * ln_r * ln_r;
  float temp_kelvin = 1.0 / t_inv;
  return k2c(temp_kelvin);
}

int main(void) {
    // float temp1 = -5.0;
    // float res1 = 36290.0;
    // float temp3 = 25.0;
    // float res3 = 10000.0;
    // float temp2 = 50.0;
    // float res2 = 4101.0;

    float temp1 = 22.700000762939453;
    float res1 = compute_resistance(2214);
    float temp2 = -21.399999618530273;
    float res2 = compute_resistance(3652);
    float temp3 = 50.0;
    float res3 = compute_resistance(1229);

    printf("r1: %.1f t1: %.1f\nr2: %.1f t2: %.1f\nr3: %.1f t3: %.1f\n", res1, temp1, res2, temp2, res3, temp3);

    struct SteinhartHartParameters cal = compute_parameters(res1, temp1, res2, temp2, res3, temp3);


    // float test_resistance = 2273.0;
    float test_resistance = 10000.0;
    float test_temp = compute_temperature(&cal, test_resistance);
    printf("Calibration A %f B %f C %.15f\n", cal.A, cal.B, cal.C);
    printf("Computed temperature: %.3fC (expected 50C)\n", test_temp);
    return 0;
}