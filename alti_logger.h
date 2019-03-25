#ifndef _ALTI_LOGGER_H__
#define _ALTI_LOGGER_H__

#define SERIAL_BAUD      		(115200)

#define pinLED 					(PC13)

// ...::: SPI :::...
#define MS5611_SS 				(PA0)
#define FLASH_SS  				(PA1)

// ...::: MS5611 :::...
#define MS5611_CMD_ADC_READ     (0x00)
#define MS5611_CMD_RESET        (0x1E)
#define MS5611_CMD_CONV_D1      (0x48)
#define MS5611_CMD_CONV_D2      (0x58)
#define MS5611_PROM_C1 			(0xA2)
#define MS5611_PROM_C2 			(0xA4)
#define MS5611_PROM_C3 			(0xA6)
#define MS5611_PROM_C4 			(0xA8)
#define MS5611_PROM_C5 			(0xAA)
#define MS5611_PROM_C6 			(0xAC)
#define MS5611_CMD_READ_PROM    (0xA0)

#define Flash_CMD_page_program  (0x02)
#define Flash_CMD_read_data     (0x03)
#define Flash_max_addres        (4194303)   // 32Mbit = 4Mbyte = 4 x 1024 x 1024 = 4194304 max adress 419304 - 1
#define Flash_FAT_0_addres      (32)
#define Flash_data_0_addres     (64)

// ...::: SERIAL :::...
#define serial_TX1 				(PA9)
#define serial_RX1 				(PA10)

extern uint32_t referencePressure; // start pressure
extern uint8_t MS5611_CONV_T;	// ADC convert time
extern uint16_t C1,C2,C3,C4,C5,C6; // Calibration data
extern uint32_t rawTemp, rawPress; // Digital pressure and temperature data
extern int16_t Temperature;		// Calculate temperature
extern uint32_t Pressure;			// Calculate pressure
extern int16_t Altitude;			// Calculate altitude
extern String input;

extern double timeCount;   //licznik milisecund od power on

uint8_t altiLogger_init();
bool altiLogger_error(uint8_t error_num);
bool sprawdzMode();
void showLoggerHelp();

void LEDping(uint8_t count);

void selectMS5611();
void deselectMS5611();
void wyswietlCx();
uint8_t MS5611_read_8bits(uint8_t reg);
uint16_t MS5611_read_16bits(uint8_t reg);
uint32_t MS5611_read_24bits(uint8_t reg);
void MS5611_write(uint8_t reg);
void readConv(void);
void readParameters();
uint32_t calcPressure();
int16_t calcTemperature();
int16_t calcAltitude(double pressure, double seaLevelPressure);

void selectFlash();
void deselectFlash();
uint8_t Flash_Manufacturer_ID();
void Flash_Write_Enable();
void Flash_Write_Disable();
bool FlashBusy();
void FlashErase();
uint32_t Flash_write_string(uint32_t addr, String str);
uint8_t Flash_read_8bit(uint32_t addr);
void Flash_write_8bit(uint32_t addr, uint8_t data);
String measurement_to_FlashStringFormat();

#endif
