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
double timeCount = 0;   	//licznik milisecund od power on
String input;

const String altiLogger_ID = "DLG_LOGGER_20090605";
const String altiLogger_ver = "v0.2";
const String altiLogger_file_extension = ".csv";


/*
 *  ---------------------------------------------------
 *  
 *      init logger led, sensor and flash
 * 		return  1  	altiLogger mode
 * 		return	2	MS5611 no connected
 * 		return	3	Flash not connected
 * 		return	4	Flash not formated
 * 		return	5	logger connected to PC
 *  
 *   ---------------------------------------------------*/
uint8_t altiLogger_init()
{
	uint8_t x;
	
//set SS MS5661, Flash and LED pin as output and set value
	
	pinMode(pinLED, OUTPUT);		//LED
	LEDping(3);
	LEDon();
	
	pinMode(MS5611_SS, OUTPUT); 	//MS5611
	deselectMS5611();
	
	pinMode(FLASH_SS, OUTPUT); 		// W25Q32
	deselectFlash();
		
//Init SPI
	
	SPI.begin();
	SPI.setBitOrder(MSBFIRST); 				// Set the SPI bit order
	SPI.setDataMode(SPI_MODE0); 			//Set the  SPI_mode 0
	SPI.setClockDivider(SPI_CLOCK_DIV8);    // Speed (72 / 8 = 9 MHz SPI_1 speed)
	
//MS5611 reset
	MS5611_write(MS5611_CMD_RESET);
	delay(100);
	
//MS5611 read calibration data
	C1 = MS5611_read_16bits(MS5611_PROM_C1);
	C2 = MS5611_read_16bits(MS5611_PROM_C2);
	C3 = MS5611_read_16bits(MS5611_PROM_C3);
	C4 = MS5611_read_16bits(MS5611_PROM_C4);
	C5 = MS5611_read_16bits(MS5611_PROM_C5);
	C6 = MS5611_read_16bits(MS5611_PROM_C6);
	
//MS5611 check connection
//if C1 == 0x00 or 0xff then return 2 //error nr 2 "MS5611 not connect"
	
	if((C1==0x0000) || (C1==0xFFFF)) return 2;
	if((C2==0x0000) || (C2==0xFFFF)) return 2;
	if((C3==0x0000) || (C3==0xFFFF)) return 2;
	if((C4==0x0000) || (C4==0xFFFF)) return 2;
	if((C5==0x0000) || (C5==0xFFFF)) return 2;
	if((C6==0x0000) || (C6==0xFFFF)) return 2;
	
//MS5611 read referencePresure
	readConv();
	referencePressure = calcPressure();
	
//Flash check connection
//if manufacture != 0xFE or device != 0x15 then return 3 //error nr 3 "Flash not connect"
	
	selectFlash();
	SPI.transfer(0x90);	//read ID command
	SPI.transfer(0);
	SPI.transfer(0);
	if(SPI.transfer(0)!=Flash_manufacturer_ID) return 3;
	if(SPI.transfer(0)!=Flash_W25Q32_ID_8bit) return 3;
	deselectFlash();
	
//Flash check logger_id at the beginning of memory
//if != ltiLogger_ID then return 4 //error 4 "Flash memory not formated"
	
	x = altiLogger_ID.length();
	String buffer = "";
	
	selectFlash();
	SPI.transfer(Flash_CMD_read_data);
	SPI.transfer(0x00);
	SPI.transfer(0x00);
	SPI.transfer(0x00);
	for(uint8_t i=0;i<x;i++){
		buffer = buffer + String(SPI.transfer(i));
	}
	deselectFlash();
	if(buffer!=altiLogger_ID) return 4;	
	
	
//check TX1 and RX1 connent
//if no connect then PCmode return 5 //PCmode "altiLogger is connented to PC"
	
	x = 0xCA;
	pinMode(serial_TX1, OUTPUT);
	pinMode(serial_RX1, INPUT_PULLUP);
	for (int8_t i=0;i<8;i++)
	{
		digitalWrite(serial_TX1, bitRead(x,i));
		if(digitalRead(serial_RX1)!=bitRead(x,i)) return 5;
	}
	
	return 1;	//no error and altiLogger_mode
	
}


/* ---------------------------------------------------
 *
 *      Flsh_print_mem - print all write mem
 *
 * ---------------------------------------------------*/
bool altiLogger_error(uint8_t error_num)
{

  uint8_t LEDping_count;
  String error_desc;

  if(error_num==3){		//Flash mem is not formated
    LEDping_count = 4;
    error_desc = "Flash mem is not formatted";
  }
  
  if(error_num==100){		//error return value altiLogger_error
    while(1){
		Serial.println("error return value altiLogger_error");
		LEDping(32);
		delay(500);
	}    
  }
  
  
  LEDping_count = 32;
  error_desc = "unknow error";
  
  while(1)
  {

    //show and print error info
    LEDping(LEDping_count);
    Serial.println(error_desc);
    Serial.println("fixed it or more info send 'PC' command");
    delay(500);

    //sheck serial commend "PC" avilable
    if (Serial.available() > 0) { //sprawdz czy nie ma komendy
      input = Serial.readString();
      if (input.indexOf("PC") > -1) return true;
    }
  }
  return false;
}//end of altiLogger_error


/* ---------------------------------------------------
 *
 *      check PC connection
 *
 * ---------------------------------------------------*/
bool sprawdzMode()
{
  
  uint16_t x = 0xB4;
  
  pinMode(serial_TX1, OUTPUT);
  pinMode(serial_RX1, INPUT_PULLUP);
  digitalWrite(pinLED, LOW);
  
  for (int8_t i=0;i<16;i++)
  {
    digitalWrite(serial_TX1, bitRead(x,i));
    if(digitalRead(serial_RX1)!=bitRead(x,i)) return true;
  }
  return false;
  
}//end of sprawdzMode()


/* ---------------------------------------------------
 *
 *      showLoggerHelp
 *
 * ---------------------------------------------------*/
void showLoggerHelp()
{
  
  Serial.print("DLG LOGGER ");
        Serial.println("v0.2");
        Serial.println("lista komend:");        
        Serial.println("   start - wyświetlania pomiarów");
        Serial.println("   stop - wyświetlania pomiarów");
        Serial.println("   read - ...");
        Serial.println("   erase - kasuj pamięć flash");
        Serial.println("   identyfikator - wyświatl");
        Serial.println("   help - to menu");
  
}//end of showLoggerHelp


/* ---------------------------------------------------
 *
 *      select and deselect MS5611 and Flash
 *
 * ---------------------------------------------------*/
void selectMS5611(){ digitalWrite(MS5611_SS, LOW);}
void deselectMS5611(){ digitalWrite(MS5611_SS, HIGH);}
void selectFlash(){ digitalWrite(FLASH_SS, LOW);}
void deselectFlash(){ digitalWrite(FLASH_SS, HIGH);}


/* ---------------------------------------------------
 *
 *      50ms LED blink - PC13
 *
 * ---------------------------------------------------*/
void LEDping(uint8_t count){ 
	
	for(uint8_t i=0;i<count;i++){	//50ms LED blink
		digitalWrite(pinLED, LOW);
		delay(50);
		digitalWrite(pinLED, HIGH);
		delay(50);
	}
	
}//end of LEDping


/* ---------------------------------------------------
 *
 *      LEDon and LEDoff
 *
 * ---------------------------------------------------*/
void LEDon(){digitalWrite(pinLED, LOW);}
void LEDoff(){digitalWrite(pinLED, HIGH);}


/* ---------------------------------------------------
 *
 *      wyswietlCx
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      odczytaj 8 bity z MS5611
 *
 * ---------------------------------------------------*/
uint8_t MS5611_read_8bits(uint8_t addr)
{
  uint8_t return_value;
  selectMS5611();
  SPI.transfer(addr);
  return_value = SPI.transfer(0);
  deselectMS5611();
  return return_value;
}//end of MS5611_read_8bits


/* ---------------------------------------------------
 *
 *      odczytaj 16 bity z MS5611
 *
 * ---------------------------------------------------*/
uint16_t MS5611_read_16bits(uint8_t addr)
{
  uint16_t return_value;
  selectMS5611();
  SPI.transfer(addr);
  return_value = SPI.transfer(0) << 8;
  return_value = return_value + SPI.transfer(0);
  deselectMS5611();
  return return_value;
}//end of MS5611_read_16bits


/* ---------------------------------------------------
 *
 *      odczytaj 24 bity z MS5611
 *
 * ---------------------------------------------------*/
uint32_t MS5611_read_24bits(uint8_t addr)
{
  uint32_t return_value;
  selectMS5611();
  SPI.transfer(addr);
  return_value = SPI.transfer(0) << 16;
  return_value = return_value + SPI.transfer(0) << 8;
  return_value = return_value + SPI.transfer(0);
  deselectMS5611();
  return return_value;
}//end of MS5611_read_24bits


/* ---------------------------------------------------
 *
 *      zapisz 8 bitów do MS5611
 *
 * ---------------------------------------------------*/
void MS5611_write(uint8_t addr)
{
  selectMS5611();
  SPI.transfer(addr);
  deselectMS5611();
}//end of MS5611_write


/* ---------------------------------------------------
 *
 *      odczytaj rawTemp i rawPress z MS5611
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      odczytaj i kalkulacja parametrów
 *
 * ---------------------------------------------------*/
void readParameters()
{

  // odczyt D1 i D2
  readConv();

  // obliczanie parametrów
  Temperature = calcTemperature();
  Pressure = calcPressure();
  Altitude = calcAltitude(rawPress,referencePressure);
}//end of readParameters


/* ---------------------------------------------------
 *
 *      kalkulacja ciśnienia MS5611
 *
 * ---------------------------------------------------*/
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

/* ---------------------------------------------------
 *
 *      kalkulacja temperatury MS5611
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      kalkulacja wysokości MS5611
 *
 * ---------------------------------------------------*/
int16_t calcAltitude(double pressure, double seaLevelPressure)
{
    //double seaLevelPressure = 101325;
    double alti = (44330.0f * (1.0f - pow((double)Pressure / (double)seaLevelPressure, 0.1902949f)));
    alti=alti*10;
    return alti;
}//end of calcAltitude


/* ---------------------------------------------------
 *
 *      read flash manufactured ID
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      flash write enable
 *
 * ---------------------------------------------------*/
void Flash_Write_Enable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x06);
  deselectFlash();
}//end of Flash_Write_Enable


/* ---------------------------------------------------
 *
 *      flash write disable
 *
 * ---------------------------------------------------*/
void Flash_Write_Disable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x04);
  deselectFlash();
}//end of Flash_Write_Disable


/* ---------------------------------------------------
 *
 *      check busy Flash
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      Flash chip erase
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      Flash_write_string
 *      zapisuje pod wskazany adres flash ciąg
 *      i zwraca nowy dres flash
 *
 * ---------------------------------------------------*/
uint32_t Flash_write_string(uint32_t addr, String str){
  
  while(FlashBusy());
  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(Flash_CMD_page_program);       //Flash page program
  SPI.transfer(addr>>16);                 //High byte
  SPI.transfer(addr>>8);                  //Middle byte
  SPI.transfer(addr);                     //Low byte
  for(int i = 0; i < str.length(); i++){  //write str to Flash
    SPI.transfer(str.charAt(i));
  }
  deselectFlash();
  Flash_Write_Disable();
  addr = addr + str.length(); //przelicz adres
  return addr;
  
}//end of Flash_write_string


/* ---------------------------------------------------
 *
 *      Flash_read_8bit
 *
 * ---------------------------------------------------*/
uint8_t Flash_read_8bit(uint32_t addr)
{
  
  while(FlashBusy());

  uint8_t return_value;
  uint8_t byteH = addr >> 16;
  uint8_t byteM = addr >> 8;
  uint8_t byteL = addr;

  selectFlash();
  SPI.transfer(Flash_CMD_read_data);
  SPI.transfer(byteH);
  SPI.transfer(byteM);
  SPI.transfer(byteL);
  return_value = SPI.transfer(0x00);
  deselectFlash();
  
  return return_value;
  
}//end of Flash_read_8bit


/* ---------------------------------------------------
 *
 *      Flash_write_8bit
 *
 * ---------------------------------------------------*/
void Flash_write_8bit(uint32_t addr, uint8_t data)
{
  
  while(FlashBusy());

  uint8_t byteH = addr >> 16;
  uint8_t byteM = addr >> 8;
  uint8_t byteL = addr;

  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(Flash_CMD_page_program);
  SPI.transfer(byteH);
  SPI.transfer(byteM);
  SPI.transfer(byteL);
  SPI.transfer(data);
  deselectFlash();
  Flash_Write_Disable();
  
}//end of Flash_write_8bit


/* ---------------------------------------------------
 *
 *      measurement_to_FlashStringFormat
 *      zwraca zformatowwane dane do zapisu do Flash
 *
 * ---------------------------------------------------*/
String measurement_to_FlashStringFormat()
{
  
  double zmienna = 0;
  String dane = "";

  zmienna = timeCount;
  zmienna = zmienna/1000;
  dane = String(zmienna,3) + ";";
  zmienna = Temperature;
  zmienna = zmienna / 10;
  dane = dane + String(zmienna,1) + ";";
  zmienna = Altitude;
  zmienna = zmienna / 10;
  dane = dane + String(zmienna,1) + "\r\n";

  return dane;
  
}//end of measurement_to_FlashStringFormat
