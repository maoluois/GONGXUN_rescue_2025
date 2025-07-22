#include "Algorithm.h"

double data_Decimal_calculate(uint8_t data_Decimal_len, uint8_t data_Point_Num, uint8_t *Data) {
    double data_return = 0;
    // 计算小数数据
    if (data_Decimal_len != 0) {
        for (uint8_t i = 0; i < data_Decimal_len; i++) {
            data_return = data_return + (Data[data_Point_Num + i + 1] - '0') * pow(10, -i - 1); // 逐位计算
        }
    }
    return data_return;
}

uint16_t data_Integer_calculate(uint8_t data_Integer_len, uint8_t data_Start_Num, uint8_t *Data) {
    uint16_t data_return = 0;
    // 计算整数数据
    if (data_Integer_len != 0) {
        for (uint8_t i = 0; i < data_Integer_len; i++) {
            data_return = data_return * 10 + (Data[data_Start_Num + i] - '0'); // 逐位计算
        }
    }
    return data_return;
}
