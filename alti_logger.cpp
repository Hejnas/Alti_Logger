#include <Arduino.h>
#include <SPI.h>
#include "alti_logger.h"

uint32_t referencePressure; // start pressure
uint8_t MS5611_CONV_T = 10;	// ADC convert time
uint16_t C1,C2,C3,C4,C5,C6; // Calibration data
uint32_t rawTemp, rawPress; // Digital pressure and temperature data
int16_t Temperature;		// Calculate temperature
uint32_t Pressure;			// Calculate pressure
int16_t Altitude;			// Calculate altitude


// ---------------------------------------------------
//
//      init logger led, sensor and flash
//
// ---------------------------------------------------
void initLogger()
{
	// Init SPI
	SPI.begin();
	SPI.setBitOrder(MSBFIRST); // Set the SPI bit order
	SPI.setDataMode(SPI_MODE0); //Set the  SPI_mode 0
	SPI.setClockDivider(SPI_CLOCK_DIV8);      // Slow speed (72 / 8 = 9 MHz SPI_1 speed)
	
	// LED
	pinMode(pinLED, OUTPUT);
	digitalWrite(pinLED, LOW);
	
	// MS5611
	pinMode(MS5611_SS, OUTPUT); // Define MS5611 NSS bit
	deselectMS5611(); // manually take CSN high between spi transmissions
	
	// W25Q32
	pinMode(FLASH_SS, OUTPUT); // Define MS5611 NSS bit
	deselectFlash(); // manually take CSN high between spi transmissions
	
	//reset MS5611
	MS5611_write(MS5611_CMD_RESET);
	delay(20);
  
	//read calibration data
	C1 = MS5611_read_16bits(MS5611_PROM_C1);
	C2 = MS5611_read_16bits(MS5611_PROM_C2);
	C3 = MS5611_read_16bits(MS5611_PROM_C3);
	C4 = MS5611_read_16bits(MS5611_PROM_C4);
	C5 = MS5611_read_16bits(MS5611_PROM_C5);
	C6 = MS5611_read_16bits(MS5611_PROM_C6);

	//read referencePressure
	readConv();
	referencePressure = calcPressure();
	
}


// ---------------------------------------------------
//
//      select and deselect MS5611 and Flash
//
// ---------------------------------------------------
void selectMS5611(){ digitalWrite(MS5611_SS, LOW);}
void deselectMS5611(){ digitalWrite(MS5611_SS, HIGH);}
void selectFlash(){ digitalWrite(FLASH_SS, LOW);}
void deselectFlash(){ digitalWrite(FLASH_SS, HIGH);}


// ---------------------------------------------------
//
//      mrugnięcie LEDem na PC13
//
// ---------------------------------------------------

void LEDping(uint8_t count)
{ 
  for (uint16_t i=0;i<count;i++)
  {
    digitalWrite(pinLED, LOW);
    delay(50);
    digitalWrite(pinLED, HIGH);
    delay(50);
  }
}//end of LEDping

// ---------------------------------------------------
//
//      wyswietlCx
//
// ---------------------------------------------------
void wyswietlCx(){
  Serial.print("C1: ");
  Serial.print(C1);
  Serial.print("; C2: ");
  Serial.print(C2);
  Serial.print("; C3: ");
  Serial.print(C3);
  Serial.print("; C4: ");
  Serial.print(C4);
  Serial.print("; C5: ");
  Serial.print(C5);
  Serial.print("; C6: ");
  Serial.println(C6);
}//end of wyswietlCx


// ---------------------------------------------------
//
//      odczytaj 8 bity z MS5611
//
// ---------------------------------------------------
uint8_t MS5611_read_8bits(uint8_t reg)
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg;
  selectMS5611();
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  deselectMS5611();
  return return_value;
}//end of MS5611_read_8bits


// ---------------------------------------------------
//
//      odczytaj 16 bity z MS5611
//
// ---------------------------------------------------
uint16_t MS5611_read_16bits(uint8_t reg)
{
  uint8_t dump, byteH, byteL;
  uint16_t return_value;
  uint8_t addr = reg;
  selectMS5611();
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteL = SPI.transfer(0);
  deselectMS5611();
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return return_value;
}//end of MS5611_read_16bits


// ---------------------------------------------------
//
//      odczytaj 24 bity z MS5611
//
// ---------------------------------------------------
uint32_t MS5611_read_24bits(uint8_t reg)
{
  uint8_t dump,byteH,byteM,byteL;
  uint32_t return_value;
  uint8_t addr = reg;
  selectMS5611();
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  deselectMS5611();
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return return_value;
}//end of MS5611_read_24bits


// ---------------------------------------------------
//
//      zapisz 8 bitów do MS5611
//
// ---------------------------------------------------

void MS5611_write(uint8_t reg)
{
  uint8_t dump;
  selectMS5611();
  dump = SPI.transfer(reg);
  deselectMS5611();
}//end of MS5611_write


// ---------------------------------------------------
//
//      odczytaj rawTemp i rawPress z MS5611
//
// ---------------------------------------------------
void readConv(void){

  //odczyt raw ciśnienia
  MS5611_write(MS5611_CMD_CONV_D1);
  delay(MS5611_CONV_T);
  rawPress = MS5611_read_24bits(MS5611_CMD_ADC_READ);

  //odczyt raw temperatury
  MS5611_write(MS5611_CMD_CONV_D2);
  delay(MS5611_CONV_T);
  rawTemp = MS5611_read_24bits(MS5611_CMD_ADC_READ);
  
}// end of readConv


// ---------------------------------------------------
//
//      odczytaj i kalkulacja parametrów
//
// ---------------------------------------------------
void readParameters()
{

  // odczyt D1 i D2
  readConv();

  // obliczanie parametrów
  Temperature = calcTemperature();
  Pressure = calcPressure();
  Altitude = calcAltitude(rawPress,referencePressure);
}//end of readParameters


// ---------------------------------------------------
//
//      kalkulacja ciśnienia MS5611
//
// ---------------------------------------------------
uint32_t calcPressure()
{
    uint32_t D1 = rawPress;

    uint32_t D2 = rawTemp;
    int32_t dT = D2 - (uint32_t)C5 * 256;

    int64_t OFF = (int64_t)C2 * 65536 + (int64_t)C4 * dT / 128;
    int64_t SENS = (int64_t)C1 * 32768 + (int64_t)C3 * dT / 256;

  int32_t TEMP = 2000 + ((int64_t) dT * C6) / 8388608;

  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  if (TEMP < 2000)
  {
      OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
      SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
  }

  if (TEMP < -1500)
  {
      OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  }

  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

    double P = (D1 * SENS / 2097152 - OFF) / 32768;

    uint32_t PRS = uint32_t(P);

    return P;
}//end of calcPressure

// ---------------------------------------------------
//
//      kalkulacja temperatury MS5611
//
// ---------------------------------------------------
int16_t calcTemperature()
{
    uint32_t D2 = rawTemp;
    int32_t dT = D2 - (uint32_t)C5 * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * C6) / 8388608;

    int64_t TEMP2 = 0;

  if (TEMP < 2000)
  {
      TEMP2 = (dT * dT) / (2 << 30);
  }

    TEMP = TEMP - TEMP2;

    return ((int16_t)TEMP/10);
}//end of calcTemperature


// ---------------------------------------------------
//
//      kalkulacja wysokości MS5611
//
// ---------------------------------------------------
int16_t calcAltitude(double pressure, double seaLevelPressure)
{
    //double seaLevelPressure = 101325;
    double alti = (44330.0f * (1.0f - pow((double)Pressure / (double)seaLevelPressure, 0.1902949f)));
    alti=alti*10;
    return alti;
}//end of calcAltitude


// ---------------------------------------------------
//
//      read flash manufactured ID
//
// ---------------------------------------------------
uint8_t Flash_Manufacturer_ID()
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = 0x9F;
  selectFlash();
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  dump = SPI.transfer(0);
  dump = SPI.transfer(0);
  deselectFlash();
  return return_value;
}//end of Flash_Manufacturer_ID


// ---------------------------------------------------
//
//      flash write enable
//
// ---------------------------------------------------
void Flash_Write_Enable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x06);
  deselectFlash();
}//end of Flash_Write_Enable


// ---------------------------------------------------
//
//      flash write disable
//
// ---------------------------------------------------
void Flash_Write_Disable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x04);
  deselectFlash();
}//end of Flash_Write_Disable


// ---------------------------------------------------
//
//      check busy Flash
//
// ---------------------------------------------------
bool FlashBusy()
{
  uint8_t dump;
  selectFlash();
  SPI.transfer(0x05);
  dump = SPI.transfer(0xff);
  deselectFlash();
  if(dump & 0x01)
    return true;
  return false;
}


// ---------------------------------------------------
//
//      Flash chip erase
//
// ---------------------------------------------------
void FlashErase()
{
  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(0xC7);
  deselectFlash();
  delay(300);
  Flash_Write_Disable();
  while(FlashBusy());
}

