/*
	
  ******************************************************************************
  * @file 					( фаил ):   ADXL345_config.h
  * @brief 			( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

#ifndef _ADXL345_CONFIG_H
#define _ADXL345_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------*/

#include "main.h"

/*

	Подключение:
	
				VDD 							Напряжение питания от 2,0 В до 3,6 В
				GND 
				CS								Выбор режима работы SPI/I2C если он подтянут к HIGH работает по I2C если к LOW то с SPI ( у модуля по умолчанию подтянут к HIGH )
				INT1							Выход прерывания 1. при включении прерывания незабываем подтянуть его к провотиположному питанию, по умолчанию стоит прерывание по восходящему фронту ( активный уровень HIGH )
				INT2							Выход прерывания 2. при включении прерывания незабываем подтянуть его к провотиположному питанию, по умолчанию стоит прерывание по восходящему фронту ( активный уровень HIGH )
				SDO/ALT						Последовательный вывод данных (SPI 4-Wire MISO) / Альтернативный I2C Выбор адреса (I2C). ( у модуля меняет адрес шины I2C подтянут к LOW ( default ) )
				SDA/SDI/SDIO			Серийные данные (I2C)/Последовательный ввод данных (SPI 4-Wire MOSI)/Последовательный ввод и вывод данных (SPI 3-Wire). 
				SCL/SCLK					Часы последовательной связи. SCL - это часы для I2C, а SCLK - это часы для SPI.
				
				
				для SPI -> maximum SPI clock speed is 5 MHz -> (CPOL) = 1 (HIGH) and clock phase (CPHA) = 1 (2EDGE)
*/

//===  SETUP ===========================================================

		#define ADXL345_I2C_MODE
		//#define ADXL345_SPI_MODE		// по SPI не работало прерывание для двойного удара


#if (defined ADXL345_I2C_MODE) && !(defined ADXL345_SPI_MODE)
		// выбераем адрес датчика
		#define ADXL345_ADDRESS 		(0x53 << 1)			// адрес устройства если пин  SDO/ALT подтянут к LOW ( default )
		//#define ADXL345_ADDRESS 	(0x1D << 1)			// адрес устройства если пин  SDO/ALT подтянут к HIGH
		
		// указываем шину I2C
		#define ADXL345_I2C					hi2c1

#elif !(defined ADXL345_I2C_MODE) && (defined ADXL345_SPI_MODE)
		//для SPI -> maximum SPI clock speed is 5 MHz -> (CPOL) = 1 (HIGH) and clock phase (CPHA) = 1 (2EDGE)
		// указываем шину SPI
		#define ADXL345_SPI					hspi1
		
		// указываем порт и пин CS
		#if !defined (CS_GPIO_Port)
			#define CS_GPIO_Port    	GPIOA
			#define CS_Pin						GPIO_PIN_14
		#endif
		
#endif


//======================================================================







#ifdef __cplusplus
}
#endif

#endif	/*	_ADXL345_CONFIG_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
