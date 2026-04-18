/*
 * ds1302.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#include "ds1307.h"
DS1307_Handle_t ds1307Handler;

static void DS1307_Write(uint8_t value, uint8_t reg_addr);
static uint8_t DS1307_Read(uint8_t reg_addr);
static uint8_t Bin2BCD(uint8_t value);
static uint8_t BCD2Bin(uint8_t value);

void DS1307_PinConfig(void)
{
    GPIO_Handle_t I2C_Pins;
    memset(&I2C_Pins, 0, sizeof(I2C_Pins));
    I2C_Pins.pGPIOx = GPIOB;
    I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_OD;
    I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    // SCL
    I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
    GPIO_Init(&I2C_Pins);

    // SDA
    I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
    GPIO_Init(&I2C_Pins);
}
void DS1307_I2C_Config(I2C_Handle_t *ds1307)
{
    ds1307->pI2Cx = DS1307_I2C;
    ds1307->I2C_Config.I2C_AckControl = DS1307_I2C_ACKCTRL;
    ds1307->I2C_Config.I2C_DeviceAddress = DS1307_I2C_DEVICE_ADDR;
    ds1307->I2C_Config.I2C_SCLSpeed = DS1307_I2C_SCLSPEED;
    ds1307->I2C_Config.I2C_FMDutyCycle = DS1307_I2C_DUTYCYCLE;
    I2C_Init(ds1307);
    // ENABLE THE I2C PERIPHERAL
    I2C_PeripheralControl(ds1307->pI2Cx, ENABLE);
}

uint8_t DS1307_init(void)
{
    // Init the I2C Pins
    DS1307_PinConfig();
    // Initialize the I2C Peripheral
    DS1307_I2C_Config(&ds1307Handler.pI2C);

    // Make the CH (CLOCK HALT) bit = 0 to enable the Oscillator
    DS1307_Write(0x00, DS1307_ADDR_SEG);

    // Read back clock halt bit
    if ((DS1307_Read(DS1307_ADDR_SEG) >> 7 & 0x1) != 0)
        return DS1307_INIT_ERROR;
    return DS1307_INIT;
}

void DS1307_setCurrentTime(DS1307_Handle_t *RTC_Time)
{
    uint8_t seg, hours = RTC_Time->time.hours;
    seg = Bin2BCD(RTC_Time->time.seconds);
    seg &= ~DS1307_CH_BIT; // Make the CH (CLOCK HALT) bit = 0
    DS1307_Write(seg, DS1307_ADDR_SEG);
    DS1307_Write(Bin2BCD(RTC_Time->time.minutes), DS1307_ADDR_MIN);

    hours = Bin2BCD(RTC_Time->time.hours);
    if (RTC_Time->time.time_format != TIMEFORMAT_24HRS)
    {
        hours |= DS1307_TIMEFORMAT_BIT; // SET 12 HOURS FORMAT
        hours = RTC_Time->time.time_format != TIMEFORMAT_12HRS_AM
                    ? hours & DS1307_AM_BIT  // PM
                    : hours | DS1307_PM_BIT; // AM
    }
    else
    {
        hours &= ~DS1307_TIMEFORMAT_BIT; // SET 24 HOURS FORMAT
    }

    DS1307_Write(hours, DS1307_ADDR_HRS);
}

void DS1307_setCurrentDate(DS1307_Handle_t *RTC_Date)
{
    DS1307_Write(Bin2BCD(RTC_Date->date.day), DS1307_ADDR_DAY);
    DS1307_Write(Bin2BCD(RTC_Date->date.date), DS1307_ADDR_DATE);
    DS1307_Write(Bin2BCD(RTC_Date->date.month), DS1307_ADDR_MONTH);
    DS1307_Write(Bin2BCD(RTC_Date->date.year), DS1307_ADDR_YEAR);
}

void DS1307_getCurrentTime(DS1307_Handle_t *RTC_Time)
{
    uint8_t sec, hours;
    sec = DS1307_Read(DS1307_ADDR_SEG);
    sec &= ~DS1307_CH_BIT;

    RTC_Time->time.seconds = BCD2Bin(sec);
    RTC_Time->time.minutes = BCD2Bin(DS1307_Read(DS1307_ADDR_MIN));

    hours = DS1307_Read(DS1307_ADDR_HRS);
    if (hours & DS1307_TIMEFORMAT_BIT)
    {
        // 12 HOURS FORMAT
        RTC_Time->time.time_format = !(hours & DS1307_PM_BIT) ? TIMEFORMAT_12HRS_PM : TIMEFORMAT_12HRS_AM;
        hours &= ~(0x3 << 5); // Clear 5 & 6
    }
    else
    {
        // 24 HOURS FORMAT
        RTC_Time->time.time_format = TIMEFORMAT_24HRS;
    }

    RTC_Time->time.hours = BCD2Bin(hours);
}
void DS1307_getCurrentDate(DS1307_Handle_t *RTC_Date)
{
    RTC_Date->date.day = BCD2Bin(DS1307_Read(DS1307_ADDR_DAY));
    RTC_Date->date.date = BCD2Bin(DS1307_Read(DS1307_ADDR_DATE));
    RTC_Date->date.month = BCD2Bin(DS1307_Read(DS1307_ADDR_MONTH));
    RTC_Date->date.year = BCD2Bin(DS1307_Read(DS1307_ADDR_YEAR));
}

static void DS1307_Write(uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = value;

    I2C_MasterSendData(&ds1307Handler.pI2C, tx, 2, ds1307Handler.pI2C.I2C_Config.I2C_DeviceAddress, I2C_DISABLE_SR);
}

static uint8_t DS1307_Read(uint8_t reg_addr)
{
    uint8_t data;
    I2C_MasterSendData(&ds1307Handler.pI2C, &reg_addr, 1, ds1307Handler.pI2C.I2C_Config.I2C_DeviceAddress, I2C_DISABLE_SR);
    I2C_MasterReceiveData(&ds1307Handler.pI2C, &data, 1, ds1307Handler.pI2C.I2C_Config.I2C_DeviceAddress, I2C_DISABLE_SR);
    return data;
}

static uint8_t Bin2BCD(uint8_t value)
{
    uint8_t dec, unit, bcdValue = 0;
    dec = value / 10;
    unit = value % 10;
    bcdValue = (dec << 4) | unit;
    return bcdValue;
}

static uint8_t BCD2Bin(uint8_t value)
{
    uint8_t binValue;
    binValue = (value >> 4) * 10 + (value & 0xF);

    return binValue;
}

void format_timeString(uint8_t *buffer,
                       uint8_t *time_format,
                       uint8_t sec,
                       uint8_t min,
                       uint8_t hours)
{
    uint8_t digit = 0;

    buffer[digit++] = (hours / 10) + '0';
    buffer[digit++] = (hours % 10) + '0';
    buffer[digit++] = ':';
    buffer[digit++] = (min / 10) + '0';
    buffer[digit++] = (min % 10) + '0';
    buffer[digit++] = ':';
    buffer[digit++] = (sec / 10) + '0';
    buffer[digit++] = (sec % 10) + '0';
    buffer[digit++] = ' ';

    if (*time_format != TIMEFORMAT_24HRS)
    {
        buffer[digit++] = *time_format ? 'P' : 'A';
        buffer[digit++] = 'M';
    }

    buffer[digit++] = '\0';
}

void format_dateString(uint8_t *buffer,
                       uint8_t day,
                       uint8_t date,
                       uint8_t month,
                       uint8_t year)
{
    uint8_t daysName[7][3] = {"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
    uint8_t digit = 0;

    buffer[digit++] = (date / 10) + '0';
    buffer[digit++] = (date % 10) + '0';
    buffer[digit++] = '/';
    buffer[digit++] = (month / 10) + '0';
    buffer[digit++] = (month % 10) + '0';
    buffer[digit++] = '/';
    buffer[digit++] = (year / 10) + '0';
    buffer[digit++] = (year % 10) + '0';
    buffer[digit++] = ' ';

    for (size_t i = 0; i < 3; i++)
    {
        buffer[digit++] = daysName[day - 1][i];
    }

    buffer[digit++] = '\0';
}

void codeString(uint8_t *timeBuffer, uint8_t *dateBuffer, uint8_t *codeBuffer)
{
    uint8_t index = 0, indexTmp = 0;

    while (timeBuffer[indexTmp] != '\0')
    {
        codeBuffer[index++] = timeBuffer[indexTmp++];
    }
    codeBuffer[index++] = '\n';
    indexTmp = 0;

    while (dateBuffer[indexTmp] != '\0')
    {
        codeBuffer[index++] = dateBuffer[indexTmp++];
    }

    codeBuffer[index] = '\0';
}

void decodeString(uint8_t *timeBuffer, uint8_t *dateBuffer, uint8_t *codeBuffer)
{
    uint8_t index = 0, indexTmp = 0;

    while (codeBuffer[index] != '\n')
    {
        timeBuffer[indexTmp++] = codeBuffer[index++];
    }
    timeBuffer[indexTmp] = '\0';
    indexTmp = 0;
    index++;

    while (codeBuffer[index] != '\0')
    {
        dateBuffer[indexTmp++] = codeBuffer[index++];
    }

    dateBuffer[indexTmp] = '\0';
}