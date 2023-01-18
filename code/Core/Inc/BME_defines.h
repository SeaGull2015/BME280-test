/*
 * BME_defines.h
 *
 *  Created on: Aug 31, 2022
 *      Author: mech2
 */

#ifndef INC_BME_DEFINES_H_
#define INC_BME_DEFINES_H_

#define BME_CS_ID 3
#define BME_nOE_Pin GPIO_PIN_13
#define BME_nOE_GPIO_Port GPIOC
#define BME_LATCH_Pin GPIO_PIN_1
#define BME_LATCH_GPIO_Port GPIOC
#define BME_WRITE_MASK 0x7F // our write mask is ~0x80 aka 0x7F, e.g. we send 0x77 to access reg 0xF7
#define BME_READ_MASK 0x80 // first bit in reg address byte is always 1 if we want to read

// Attention, ten-hut: copypasta!!!

//---------------what the actual fuck????--------------------------//
#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00)) // big endian reverse or smth like that, didn't get it yet

#define be24toword(a) ((((a)>>16)&0x000000ff)|((a)&0x0000ff00)|(((a)<<16)&0x00ff0000))

//---------------calib data struct type--------------------------//
typedef struct

{
// we don't need the humidity and temp calib data, so commented for now
  uint16_t dig_T1;

  int16_t dig_T2;

  int16_t dig_T3;

  uint16_t dig_P1;

  int16_t dig_P2;

  int16_t dig_P3;

  int16_t dig_P4;

  int16_t dig_P5;

  int16_t dig_P6;

  int16_t dig_P7;

  int16_t dig_P8;

  int16_t dig_P9;

/*  uint8_t dig_H1;

  int16_t dig_H2;

  uint8_t dig_H3;

  int16_t dig_H4;

  int16_t dig_H5;

  int8_t dig_H6; */

} BME280_CalibData;

/*Информация о регистрах взята из "BME280 Combined humidity and pressure sensor"*/
#define BME280_ADDRESS 0x76<<1 /*I2C Адрес BME280, p.32, 6.2*/
#define BME280_REG_ID 0xD0 /*ID регистр BME280, p.26, 5.2*/
#define BME280_ID 0x60 /*Информация, читаемая от BME280 в ID регистре, 27, 5.4.1*/
#define BME280_REG_SOFTRESET 0xE0 /*Регистр для перезагрузки BME280, p.27, 5.4.2*/
#define BME280_SOFTRESET_VALUE 0xB6 /*Значение, записываемое в регистр для перезагрузки BME280, p.27, 5.4.2*/
#define BME280_REGISTER_STATUS 0XF3 /*Регистр статуса BME280, p.28, 5.4.4*/
#define BME280_STATUS_MEASURING 0X08 /*Значение из регистра статуса при запуске измерения BME280, p.28, 5.4.4*/
#define BME280_STATUS_IM_UPDATE 0X01 /*Значение из регистра статуса при окончании измерения BME280, p.28, 5.4.4*/
#define BME280_REGISTER_DIG_T1 0x88/*Регистр, откуда читаем калибровочное значение 1, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_T2 0x8A/*Регистр, откуда читаем калибровочное значение 2, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_T3 0x8C/*Регистр, откуда читаем калибровочное значение 3, p.24, 4.2.2*/
//------------------------------------------------//
#define BME280_REG_CONFIG 0xF5 /*Регистр конфигурации BME280, задаём время ожидания, значение постоянной времени
фильтра BME280, p.29, 5.4.6*/
#define BME280_STBY_MSK 0xE0/*Вспомогательная "маска" для параметрирования времени ожидания*/
/*Время ожидания t_standby, мс, p.30*/
#define BME280_STBY_0_5 0x00
#define BME280_STBY_62_5 0x20
#define BME280_STBY_125 0x40
#define BME280_STBY_250 0x60
#define BME280_STBY_500 0x80
#define BME280_STBY_1000 0xA0
#define BME280_STBY_10 0xC0
#define BME280_STBY_20 0xE0
//------------------------------------------------//
#define BME280_FILTER_MSK 0x1C/*Вспомогательная "маска" для параметрирования постоянной времени фильтра*/
/*Постоянная времени фильтра, p.30, 5.4.6., p.30*/
#define BME280_FILTER_OFF 0x00
#define BME280_FILTER_2 0x04
#define BME280_FILTER_4 0x08
#define BME280_FILTER_8 0x0C
#define BME280_FILTER_16 0x10
//------------------------------------------------//
#define BME280_REG_CTRL_MEAS 0xF4 /*Регистр параметров сбора данных давления и температуры, p.28, 5.4.5*/
/*p.29, 5.4.5*/
#define BME280_OSRS_T_MSK 0xE0/*Вспомогательная "маска" для параметрирования преддискретизации*/
/*Параметр преддискретизации температуры, p.29*/
#define BME280_OSRS_T_SKIP 0x00
#define BME280_OSRS_T_x1 0x20
#define BME280_OSRS_T_x2 0x40
#define BME280_OSRS_T_x4 0x60
#define BME280_OSRS_T_x8 0x80
#define BME280_OSRS_T_x16 0xA0
//------------------------------------------------//
#define BME280_MODE_MSK 0x03/*Вспомогательная "маска" для параметрирования режима работы BME280*/
/*Режим работы датчика, p.29*/
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03
//------------------------------------------------//
#define BME280_REGISTER_TEMPDATA 0xFA/*Регистр, откуда читаются данные температуры BME280, p.31, 5.4.8*/
//------------------------------------------------
#define be24toword(a) ((((a)>>16)&0x000000ff)|((a)&0x0000ff00)|(((a)<<16)&0x00ff0000))/*Функция перестановки байтов*/

//---------------calib data addresses--------------------------//
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C
#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E
#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7
//------------------------------------------------//

#define BME280_REGISTER_PRESSUREDATA 0xF7 // self-explanatory
#define BME280_REGISTER_TEMPDATA 0xFA


#endif /* INC_BME_DEFINES_H_ */
