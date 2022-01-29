#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <math.h>
#include <BMP388.h>

// thank you elvin from BPS

float altLUT[] = {0.        , 0.41629385, 0.47499169, 0.5130931 , 0.54196597,
       0.56547576, 0.58543972, 0.6028679 , 0.61838369, 0.63240071,
       0.64520838, 0.65701764, 0.66798728, 0.67824003, 0.68787284,
       0.69696371, 0.70557638, 0.71376365, 0.72156982, 0.72903237,
       0.73618338, 0.7430505 , 0.74965775, 0.75602613, 0.76217412,
       0.76811807, 0.77387253, 0.77945047, 0.78486357, 0.79012234,
       0.79523626, 0.80021397, 0.80506332, 0.80979148, 0.81440501,
       0.81890995, 0.82331185, 0.82761583, 0.83182663, 0.83594864,
       0.83998594, 0.84394232, 0.84782133, 0.85162627, 0.85536021,
       0.85902606, 0.86262654, 0.86616419, 0.8696414 , 0.87306045,
       0.87642346, 0.87973244, 0.88298929, 0.88619582, 0.88935373,
       0.89246464, 0.89553009, 0.89855152, 0.90153034, 0.90446786,
       0.90736533, 0.91022397, 0.91304491, 0.91582924, 0.91857802,
       0.92129224, 0.92397285, 0.92662078, 0.9292369 , 0.93182205,
       0.93437704, 0.93690264, 0.93939961, 0.94186865, 0.94431045,
       0.94672568, 0.94911498, 0.95147895, 0.9538182 , 0.95613328,
       0.95842476, 0.96069316, 0.962939  , 0.96516277, 0.96736495,
       0.96954601, 0.97170638, 0.97384651, 0.97596682, 0.9780677 ,
       0.98014956, 0.98221277, 0.98425771, 0.98628472, 0.98829416,
       0.99028637, 0.99226167, 0.99422038, 0.99616281, 0.99808925,
       1.        , 1.00189534, 1.00377555, 1.00564089, 1.00749162,
       1.00932801, 1.01115028, 1.01295869, 1.01475346, 1.01653483,
       1.01830301};

float fastPow(float P) {
  int idx = int(P * 100.f);
  float diff = P * 100.f - idx;
  if (idx >= 110) {
    return 1.01830301;
  }
  return altLUT[idx] * (1 - diff) + diff * altLUT[idx+1];
}

void BMP388::read_bytes(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t data, uint8_t len)
{
    uint8_t dat[len];
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, dat, len, false);
    data = *dat;
}

void BMP388::write_byte(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(i2c, addr, buf, 2, false);
    
}

void BMP388::read_calib_data() {

    // raw pressure calibration data
    int8_t PAR_P11_INT, PAR_P10_INT, PAR_P8_INT, PAR_P7_INT, PAR_P4_INT, PAR_P3_INT;
    int16_t PAR_P9_INT, PAR_P2_INT, PAR_P1_INT;
    uint16_t PAR_P6_INT, PAR_P5_INT;

    // raw temperature calibration data
    int8_t PAR_T3_INT;
    uint16_t PAR_T2_INT, PAR_T1_INT;

    // read calibration data

    uint8_t buf[21];

    read_bytes(&i2c, addr, CALIB_DATA, *buf, 21);

    PAR_T1_INT = (buf[1] << 8) | buf[0];
    PAR_T2_INT = (buf[3] << 8) | buf[2];

    PAR_T3_INT = buf[4];

    PAR_P1_INT = (buf[6] << 8) | buf[5];
    PAR_P2_INT = (buf[8] << 8) | buf[7];

    PAR_P3_INT = buf[9];
    PAR_P4_INT = buf[10];

    PAR_P5_INT = (buf[12] << 8) | buf[11];
    PAR_P6_INT = (buf[14] << 8) | buf[13];

    PAR_P7_INT = buf[15];

    PAR_P8_INT = buf[16];

    PAR_P9_INT = (buf[18] << 8) | buf[17];
    
    PAR_P10_INT = buf[19];
    PAR_P11_INT = buf[20];

    // calculate calibration data

    calib_data.PAR_T1 = (float)PAR_T1_INT   / 0.00390625f; // 2 ^ -8
    calib_data.PAR_T2 = (float)PAR_T2_INT   / powf(2, 30); // 2 ^ 30
    calib_data.PAR_T3 = (float)PAR_T3_INT   / powf(2, 48); // 2 ^ 48

    calib_data.PAR_P1 = ((float)PAR_P1_INT  - powf(2, 14)) / powf(2, 20);
    calib_data.PAR_P2 = ((float)PAR_P2_INT  - powf(2, 14)) / powf(2, 29);
    calib_data.PAR_P3 = (float)PAR_P3_INT   / powf(2, 32);

    calib_data.PAR_P4 = (float)PAR_P4_INT   / powf(2, 37);
    calib_data.PAR_P5 = (float)PAR_P5_INT   / 0.125f;
    calib_data.PAR_P6 = (float)PAR_P6_INT   / powf(2, 6);
    calib_data.PAR_P7 = (float)PAR_P7_INT   / powf(2, 8);
    calib_data.PAR_P8 = (float)PAR_P8_INT   / powf(2, 15);
    calib_data.PAR_P9 = (float)PAR_P9_INT   / powf(2, 48);
    calib_data.PAR_P10 = (float)PAR_P10_INT / powf(2, 48);
    calib_data.PAR_P11 = (float)PAR_P11_INT / powf(2, 65);

}

void BMP388::compensate_temp(uint32_t temp_uncomp) {
    float partial_1, partial_2;

    partial_1 = (float)(temp_uncomp - calib_data.PAR_T1);
    partial_2 = (float)(partial_1 * calib_data.PAR_T2);

    temp = partial_2 + (partial_1 * partial_1) * calib_data.PAR_T3;
} 

void BMP388::compensate_pres(uint32_t pres_uncomp) {
    float partial_1, partial_2, partial_3, partial_4;
    float partial_out_1, partial_out_2;

    float pres_uncomp_flt = (float)pres_uncomp;
    float pres_uncomp_flt_2 = pres_uncomp_flt * pres_uncomp_flt;

    float temp_squared = temp * temp;
    float temp_cubed = temp_squared * temp;

    partial_1 = calib_data.PAR_P6 * temp;
    partial_2 = calib_data.PAR_P7 * temp_squared;
    partial_3 = calib_data.PAR_P8 * temp_cubed;
    partial_out_1 = calib_data.PAR_P5 + partial_1 + partial_2 + partial_3;

    partial_1 = calib_data.PAR_P2 * temp;
    partial_2 = calib_data.PAR_P3 * temp_squared;
    partial_3 = calib_data.PAR_P4 * temp_cubed;

    partial_out_2 = pres_uncomp_flt * (calib_data.PAR_P1 + partial_1 + partial_2 + partial_3);

    partial_1 = pres_uncomp_flt_2;
    partial_2 = calib_data.PAR_P9 + calib_data.PAR_P10 * temp;
    partial_3 = partial_1 * partial_2;

    partial_4 = partial_3 + (pres_uncomp_flt_2 * pres_uncomp_flt) * calib_data.PAR_P11;
    pres = partial_1 + partial_2 + partial_4;
}

void BMP388::compute_alt() { alt = (1.0f - pow(pres / 101325.0f, 0.1903f)) * 287.15f / 0.0065f; }

void BMP388::compute_alt_fast() { alt = (1.0f - fastPow(pres / 101325.0f)) * 287.15f / 0.0065f; }

