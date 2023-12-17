/*
	
  ******************************************************************************
  * @file 					( фаил ):   LIS3DSH_config.h
  * @brief 			( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

#ifndef _LIS3DSH_CONFIG_H
#define _LIS3DSH_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------*/

#include "main.h"

/*

	Подключение:
				
				VDD 							Напряжение питания от 1,71 В до 3,6 В
				GND 
				CS								Выбор режима работы SPI/I2C если он подтянут к HIGH работает по I2C если к LOW то с SPI ( у модуля по умолчанию подтянут к HIGH )
				INT1							Выход прерывания 1. при включении прерывания незабываем подтянуть его к провотиположному питанию, по умолчанию стоит прерывание по восходящему фронту ( активный уровень HIGH )
				INT2							Выход прерывания 2. при включении прерывания незабываем подтянуть его к провотиположному питанию, по умолчанию стоит прерывание по восходящему фронту ( активный уровень HIGH )
				SDO/ALT						Последовательный вывод данных (SPI 4-Wire MISO) / Альтернативный I2C Выбор адреса (I2C). ( у модуля меняет адрес шины I2C подтянут к LOW ( default ) )
				SDA/SDI/SDIO			Серийные данные (I2C)/Последовательный ввод данных (SPI 4-Wire MOSI)/Последовательный ввод и вывод данных (SPI 3-Wire). 
				SCL/SCLK					Часы последовательной связи. SCL - это часы для I2C, а SCLK - это часы для SPI.
				
				
				для SPI -> maximum SPI clock speed is 10 MHz -> (CPOL) = 0 (LOW) and clock phase (CPHA) = 0 (1EDGE)
*/

//===  SETUP ===========================================================

		#define LIS3DSH_I2C_MODE
		//#define LIS3DSH_SPI_MODE	


#if (defined LIS3DSH_I2C_MODE) && !(defined LIS3DSH_SPI_MODE)
		// выбераем адрес датчика
		#define LIS3DSH_ADDRESS 		(0x1E << 1)			// адрес устройства если пин  SDO/ALT подтянут к LOW
		//#define LIS3DSH_ADDRESS 	(0x1D << 1)			// адрес устройства если пин  SDO/ALT подтянут к HIGH
		
		// указываем шину I2C
		#define LIS3DSH_I2C					hi2c1

#elif !(defined LIS3DSH_I2C_MODE) && (defined LIS3DSH_SPI_MODE)
		//для SPI -> maximum SPI clock speed is 10 MHz -> (CPOL) = 0 (LOW) and clock phase (CPHA) = 0 (1EDGE)
		// указываем шину SPI
		#define LIS3DSH_SPI					hspi1
		
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

#endif	/*	_LIS3DSH_CONFIG_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
