/*
 * ds1302.h
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#ifndef INC_DS1302_H_
#define INC_DS1302_H_

#include "stm32f411xx.h"
/*
 *Handle structure for DS1307x peripheral
 */
typedef enum
{
    MONDAY = 1,
    TUESDAY,
    WEDNESDAY,
    THURSDAY,
    FRIDAY,
    SATURDAY,
    SUNDAY
} DS1307_Days_e;

typedef struct
{
    uint8_t date;
    uint8_t month;
    uint8_t year;
    DS1307_Days_e day;
} RTC_date_t;

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t time_format;
} RTC_time_t;

typedef struct
{
    RTC_time_t time;
    RTC_date_t date;
    I2C_Handle_t pI2C;
} DS1307_Handle_t;

#define DS1307_I2C             I2C1
#define DS1307_I2C_ACKCTRL     I2C_ACK_ENABLE
#define DS1307_I2C_DEVICE_ADDR DS1307_ADDR
#define DS1307_I2C_SCLSPEED    I2C_SCL_SPEED_SM
#define DS1307_I2C_DUTYCYCLE   I2C_FM_DUTY_2
/* ADDRESS */
#define DS1307_ADDR 0x68
/* REGISTERS ADDRESS*/
#define DS1307_ADDR_SEG     0x00
#define DS1307_ADDR_MIN     0x01
#define DS1307_ADDR_HRS     0x02
#define DS1307_ADDR_DAY     0x03
#define DS1307_ADDR_DATE    0x04
#define DS1307_ADDR_MONTH   0x05
#define DS1307_ADDR_YEAR    0x06
#define DS1307_ADDR_CONTROL 0x07

#define TIMEFORMAT_12HRS_PM 0x01
#define TIMEFORMAT_12HRS_AM 0x00
#define TIMEFORMAT_24HRS    0x02

#define DS1307_CH_BIT         (1 << 7)
#define DS1307_TIMEFORMAT_BIT (1 << 6)
#define DS1307_PM_BIT         (1 << 5)
#define DS1307_AM_BIT         ~(1 << 5)

/* RETURN VALUES */

#define DS1307_INIT       0
#define DS1307_INIT_ERROR -1

/******************************************************************************************
 *								APIs supported
 * by this driver For more information about the APIs check the function
 * definitions
 ******************************************************************************************/
uint8_t DS1307_init(void);

void DS1307_setCurrentTime(DS1307_Handle_t *RTC_Time);
void DS1307_getCurrentTime(DS1307_Handle_t *RTC_Time);

void DS1307_setCurrentDate(DS1307_Handle_t *RTC_Date);
void DS1307_getCurrentDate(DS1307_Handle_t *RTC_Date);

void format_timeString(uint8_t *buffer,
                       uint8_t *time_format,
                       uint8_t sec,
                       uint8_t min,
                       uint8_t hours);
void format_dateString(uint8_t *buffer,
                       uint8_t day,
                       uint8_t date,
                       uint8_t month,
                       uint8_t year);

void codeString(uint8_t *timeBuffer, uint8_t *dateBuffer, uint8_t *codeBuffer);
void decodeString(uint8_t *timeBuffer, uint8_t *dateBuffer, uint8_t *codeBuffer);

#endif /* INC_DS1302_H_ */
