/*

  ******************************************************************************
  * @file 			( фаил ):   LIS3DSH.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

/* Includes ----------------------------------------------------------*/
#include "LIS3DSH.h"

#if (defined LIS3DSH_I2C_MODE) && !(defined LIS3DSH_SPI_MODE)
	extern I2C_HandleTypeDef LIS3DSH_I2C;
#elif !(defined LIS3DSH_I2C_MODE) && (defined LIS3DSH_SPI_MODE)	
	extern SPI_HandleTypeDef LIS3DSH_SPI;
#endif


// LIS3DSH_Private_FunctionPrototypes ------------------------------------
static void LIS3DSH_WriteReg(uint8_t *dataW, uint8_t reg, uint8_t size);
static void LIS3DSH_ReadReg(uint8_t *dataR, uint8_t reg, uint8_t size);
static uint8_t LIS3DSH_convertTime(float milliseconds);
static uint8_t LIS3DSH_convertThreshold(float milliG);


// LIS3DSH_Private_Functions ---------------------------------------------

static void LIS3DSH_WriteReg(uint8_t *dataW, uint8_t reg, uint8_t size)
{
	#if (defined LIS3DSH_I2C_MODE) && !(defined LIS3DSH_SPI_MODE)
	
			HAL_I2C_Mem_Write(&LIS3DSH_I2C, LIS3DSH_ADDRESS, reg, 1, (uint8_t*)dataW, size, 10);
	
	#elif !(defined LIS3DSH_I2C_MODE) && (defined LIS3DSH_SPI_MODE)

			HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET );
			
			HAL_SPI_Transmit(&LIS3DSH_SPI, &reg, 1, 500 );
			HAL_SPI_Transmit(&LIS3DSH_SPI, dataW, size, 500 );
	
			HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET );
	
	#endif
}


static void LIS3DSH_ReadReg(uint8_t *dataR, uint8_t reg, uint8_t size)
{
	#if (defined LIS3DSH_I2C_MODE) && !(defined LIS3DSH_SPI_MODE)
	
			HAL_I2C_Mem_Read(&LIS3DSH_I2C, LIS3DSH_ADDRESS, reg, 1, (uint8_t*)dataR, size, 10);
	
	#elif !(defined LIS3DSH_I2C_MODE) && (defined LIS3DSH_SPI_MODE)
	
			reg |= 0x80;  // multibyte read
			
			HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET );
	
			HAL_SPI_Transmit(&LIS3DSH_SPI, &reg, 1, 500 );
			HAL_SPI_Receive(&LIS3DSH_SPI, dataR, size, 500 );
	
			HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET );
	
	#endif
}

//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
 * @brief converts time from milliseconds to the corresponding byte
 * @param milliseconds : the freefall detection time interval expressed in milliseconds
 *			LIS3DSH_DATARATE_3_125:		// 320.0ms - 81600.0ms
 *			LIS3DSH_DATARATE_6_25:		// 160.0ms - 40800.0ms
 *			LIS3DSH_DATARATE_12_5:		// 80.0ms - 20400.0ms
 *			LIS3DSH_DATARATE_25:			// 40.0ms - 10200.0ms
 *			LIS3DSH_DATARATE_50:			// 20.0ms - 5100.0ms
 *			LIS3DSH_DATARATE_100:			// 10.0ms - 2550.0ms
 *			LIS3DSH_DATARATE_400:			// 2.5ms - 637.5ms
 *			LIS3DSH_DATARATE_800:			// 1.25ms - 318.75ms
 *			LIS3DSH_DATARATE_1600:		// 0.625ms - 159.375ms
 * @return byte : the time interval converted in byte
 */
static uint8_t LIS3DSH_convertTime(float milliseconds)
{
		uint8_t tmpreg = 0;
		float coeff = 0;
	
		/* Read CTRL_REG4 register */
		LIS3DSH_ReadReg(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);
  
		/* Set new data rate configuration */
		tmpreg &= 0xF0;
	
		switch( tmpreg ){
			case LIS3DSH_DATARATE_3_125:	// 320.0ms - 81600.0ms
				coeff = 1.0 / 3.125 * 1000.0;
			break;
			case LIS3DSH_DATARATE_6_25:		// 160.0ms - 40800.0ms
				coeff = 1.0 / 6.25 * 1000.0;
			break;
			case LIS3DSH_DATARATE_12_5:		// 80.0ms - 20400.0ms
				coeff = 1.0 / 12.5 * 1000.0;
			break;
			case LIS3DSH_DATARATE_25:			// 40.0ms - 10200.0ms
				coeff = 1.0 / 25.0 * 1000.0;
			break;
			case LIS3DSH_DATARATE_50:			// 20.0ms - 5100.0ms
				coeff = 1.0 / 50.0 * 1000.0;
			break;
			case LIS3DSH_DATARATE_100:		// 10.0ms - 2550.0ms
				coeff = 1.0 / 100.0 * 1000.0;
			break;
			case LIS3DSH_DATARATE_400:		// 2.5ms - 637.5ms
				coeff = 1.0 / 400.0 * 1000.0;
			break;
			case LIS3DSH_DATARATE_800:		// 1.25ms - 318.75ms
				coeff = 1.0 / 800.0 * 1000.0;
			break;
			case LIS3DSH_DATARATE_1600:		// 0.625ms - 159.375ms
				coeff = 1.0 / 1600.0 * 1000.0;
			break;
			
		}

    // 1 LSB = 1/ODR
    float temp = milliseconds / (coeff);
    int byte = (int) temp;
    if (byte < 0){ return 0; }
    else if (byte > 255){ return 255; }
    else { return (uint8_t) byte; }
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
 * @brief converts acceleration from milli-g to the corresponding byte
 * @param milliG : the threshold expressed in milliG
 * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
 * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
 * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
 * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
 * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
 * @return byte : the threshold converted in byte
 */
static uint8_t LIS3DSH_convertThreshold(float milliG)
{
		uint8_t tmpreg = 0;
		float coeff = 0;
	
		/* Read CTRL_REG5 register */
		LIS3DSH_ReadReg(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
  
		/* Set new full scale configuration */
		tmpreg &= (uint8_t)LIS3DSH__FULLSCALE_SELECTION;
	
		switch( tmpreg ){
			case LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
				coeff = 2.0 / 128.0 * 1000.0;
			break;		
			case LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
				coeff = 4.0 / 128.0 * 1000.0;
			break;
			case LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
				coeff = 6.0 / 128.0 * 1000.0;
			break;
			case LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
				coeff = 8.0 / 128.0 * 1000.0;
			break;
			case LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
				coeff = 16.0 / 128.0 * 1000.0;
			break;
		}
		
    // 1 LSB = 2g/(2^7) -> 1LSb = FS/2^7 -> 1LSb = FS/128.
    float temp = milliG / (coeff);
    int byte = (int) temp;
    if (byte < 0){ return 0; }
    else if (byte > 255){ return 255; }
    else { return (uint8_t) byte; }
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH Initialization.
  * @param  InitStruct: contains mask of different init parameters
  * @retval None
  */
void LIS3DSH_Init(LIS3DSH_InitTypeDef* InitData)
{
	uint8_t ctrl = 0x00;
	uint16_t InitStruct = ((uint16_t)(InitData->SPI_Wire | InitData->Self_Test | InitData->Full_Scale | InitData->Filter_BW )<<8) | (InitData->Output_DataRate | InitData->Axes_Enable);
	
	
  /* Configure MEMS: power mode(ODR) and axes enable */
  ctrl = (uint8_t) (InitStruct);
  
  /* Write value to MEMS CTRL_REG4 register */
  LIS3DSH_WriteReg(&ctrl, LIS3DSH_CTRL_REG4_ADDR, 1);
  
  /* Configure MEMS: full scale and self test */
  ctrl = (uint8_t) (InitStruct >> 8);
  
  /* Write value to MEMS CTRL_REG5 register */
  LIS3DSH_WriteReg(&ctrl, LIS3DSH_CTRL_REG5_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  LIS3DSH De-Initialization.
  * @param  None
  * @retval None.
  */
void LIS3DSH_DeInit(void)
{
  
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Read LIS3DSH device ID.
  * @param  None
  * @retval The Device ID (two bytes).
  */
uint8_t LIS3DSH_ReadID(void)
{
  uint8_t tmp = 0;

  /* Read WHO_AM_I register */
  LIS3DSH_ReadReg(&tmp, LIS3DSH_WHO_AM_I_ADDR, 1);
  
  /* Return the ID */
  return (uint16_t)tmp;
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH Interrupt configuration
  * @param  LIS3DSH_InterruptConfig_TypeDef: pointer to a LIS3DSH_InterruptConfig_TypeDef 
  *         structure that contains the configuration setting for the LIS3DSH Interrupt.
  * @retval None
  */
void LIS3DSH_InterruptConfig(LIS3DSH_InterruptConfigTypeDef *LIS3DSH_IntConfigStruct)
{
  uint8_t ctrl = 0x00;
  
  /* Configure Interrupt Selection , Request and Signal */                   
  ctrl = (uint8_t)(LIS3DSH_IntConfigStruct->Interrupt_Selection_Enable | \
                   LIS3DSH_IntConfigStruct->Interrupt_Request | \
									 LIS3DSH_IntConfigStruct->Interrupt_DataReady_INT1 | \
                   LIS3DSH_IntConfigStruct->Interrupt_Vector_Filter | \
									 LIS3DSH_IntConfigStruct->Interrupt_Signal);
  
  /* Write value to MEMS CTRL_REG3 register */
  LIS3DSH_WriteReg(&ctrl, LIS3DSH_CTRL_REG3_ADDR, 1);
  
  /* Configure State Machine 1 */                   
  ctrl = (uint8_t)(LIS3DSH_IntConfigStruct->State_Machine1_Enable | \
									 LIS3DSH_IntConfigStruct->State_Machine1_Hysteresis | \
                   LIS3DSH_IntConfigStruct->State_Machine1_Interrupt);
  
  /* Write value to MEMS CTRL_REG1 register */
  LIS3DSH_WriteReg(&ctrl, LIS3DSH_CTRL_REG1_ADDR, 1);
  
  /* Configure State Machine 2 */                   
  ctrl = (uint8_t)(LIS3DSH_IntConfigStruct->State_Machine2_Enable | \
									 LIS3DSH_IntConfigStruct->State_Machine2_Hysteresis | \
                   LIS3DSH_IntConfigStruct->State_Machine2_Interrupt);
  
  /* Write value to MEMS CTRL_REG2 register */
  LIS3DSH_WriteReg(&ctrl, LIS3DSH_CTRL_REG2_ADDR, 1);
	
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for WakeUp detection
  * @param  Num State Machine, two -
  * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
  * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
  * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
  * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
  * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
  * @retval None
  */
// срабатывает от любого движения
void LIS3DSH_WakeUp_InterruptConfig(StateMachine_Num numStateMachine, float milliG)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2

		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_THRS1_2_ADDR,1);					// SM2

		ctrl =( LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH1 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_1_ADDR,1);						// SM2
		ctrl = LIS3DSH_CONT;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_2_ADDR,1);						// SM2
	}
	else{
			
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1

		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_THRS1_1_ADDR,1);					// SM1

		ctrl =( LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH1 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_1_ADDR,1);						// SM1
		ctrl = LIS3DSH_CONT;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_2_ADDR,1);						// SM1
	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for Toggle is a simple State Machine configuration that generates an interrupt every n sample.
  * @param  Num State Machine, two - samples
  * @retval None
  */
// генерирует прерывание через указанное кол-во замеров 
void LIS3DSH_Toggle_InterruptConfig(StateMachine_Num numStateMachine, uint8_t samples)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2
		
		/* Set LIS3DSH State Machines configuration */
		ctrl = 0x0F; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_2_ADDR,1);						// SM2
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI3 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_1_ADDR,1);						// SM2
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_2_ADDR,1);						// SM2
		
	}
	else{
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1
		
		/* Set LIS3DSH State Machines configuration */
		ctrl = 0x0F; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_1_ADDR,1);						// SM1
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI3 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_1_ADDR,1);						// SM1
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_2_ADDR,1);						// SM1
	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for If the object is in freefall, the X-axis, Y-axis and Z-axis have zero acceleration.
  * @param  Num State Machine,  
  * 	two - Allowed range
  * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
  * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
  * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
  * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
  * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
  * 	three - time
  * 			LIS3DSH_DATARATE_3_125:		// 320.0ms - 81600.0ms
  * 			LIS3DSH_DATARATE_6_25:		// 160.0ms - 40800.0ms
  * 			LIS3DSH_DATARATE_12_5:		// 80.0ms - 20400.0ms
  * 			LIS3DSH_DATARATE_25:			// 40.0ms - 10200.0ms
  * 			LIS3DSH_DATARATE_50:			// 20.0ms - 5100.0ms
  * 			LIS3DSH_DATARATE_100:			// 10.0ms - 2550.0ms
  * 			LIS3DSH_DATARATE_400:			// 2.5ms - 637.5ms
  * 			LIS3DSH_DATARATE_800:			// 1.25ms - 318.75ms
  * 			LIS3DSH_DATARATE_1600:		// 0.625ms - 159.375ms
  * @retval None
  */
// прерывание от свободного падения
void LIS3DSH_FreeFall_InterruptConfig(StateMachine_Num numStateMachine, float milliG, float time)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2
		
		/* Set LIS3DSH State Machines configuration */
		ctrl = LIS3DSH_convertTime(time); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_2_L_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_THRS2_2_ADDR,1);					// SM2
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LLTH2 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_1_ADDR,1);						// SM2
		
		ctrl = (LIS3DSH_GNTH2 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_2_ADDR,1);						// SM2
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_3_ADDR,1);						// SM2
	}
	else{
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_B_P_X | LIS3DSH_MASK_B_P_Y | LIS3DSH_MASK_B_P_Z; //enable positive X, Y and Z in mask B
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1
		
		/* Set LIS3DSH State Machines configuration */
		ctrl = LIS3DSH_convertTime(time); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_1_L_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_THRS2_1_ADDR,1);					// SM1
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LLTH2 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_1_ADDR,1);						// SM1
		
		ctrl = (LIS3DSH_GNTH2 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT); 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_2_ADDR,1);						// SM1
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_3_ADDR,1);						// SM1
	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for If the object is in Single Click, the X-axis, Y-axis and Z-axis have zero acceleration.
  * @param  Num State Machine,  
  * 	two - Allowed range
  * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
  * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
  * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
  * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
  * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
  * @retval None
  */
// одинарный удар
void LIS3DSH_SingleClick_InterruptConfig(StateMachine_Num numStateMachine, float milliG)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2
		
		ctrl = 0x02;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM4_2_ADDR,1);						// SM2		При обнаружении первого касания система ждет указанное время (TI4),
		ctrl = 0x08;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_2_ADDR,1);						// SM2		время окна (TI3), в котором ускорение сначала выше, а затем ниже, чем порог
		ctrl = 0x32;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM2_2_L_ADDR,1);					// SM2		
		ctrl = 0x07;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_2_L_ADDR,1);					// SM2		для создания временного окна «до молчания»,
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_2_ADDR,1);					// SM2
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_2_ADDR,1);					// SM2
		
		ctrl = (LIS3DSH_GNTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_1_ADDR,1);						// SM2
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_2_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_3_ADDR,1);						// SM2
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_4_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI4 LIS3DSH_NEXT);

		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_5_ADDR,1);						// SM2
	}
	else{
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1
		
		ctrl = 0x02;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM4_1_ADDR,1);						// SM1		При обнаружении первого касания система ждет указанное время (TI4),
		ctrl = 0x08;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_1_ADDR,1);						// SM1		время окна (TI3), в котором ускорение сначала выше, а затем ниже, чем порог
		ctrl = 0x32;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM2_1_L_ADDR,1);					// SM1		
		ctrl = 0x07;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_1_L_ADDR,1);					// SM1		для создания временного окна «до молчания»,
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_1_ADDR,1);					// SM1
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_1_ADDR,1);					// SM1
		
		ctrl = (LIS3DSH_GNTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_1_ADDR,1);						// SM1
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_2_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_3_ADDR,1);						// SM1
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_4_ADDR,1);						// SM1

		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_5_ADDR,1);						// SM1

	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for If the object is in Double Turn, the X-axis, Y-axis and Z-axis have zero acceleration.
  * @param  Num State Machine,  
  * 	two and three - Allowed range
  * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
  * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
  * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
  * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
  * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
  * @retval None
  */
// перевертывание устройства
// Идея может заключаться в том, чтобы использовать эту функцию в мобильном телефоне для включения или выключения звонка.
// тон, распознавая такие жесты, как «Лицом вверх» — «Лицом вниз» — «Лицом вверх».
// Функция выполняется с использованием 4 состояний:
// проверьте, не ниже ли ускорение по оси Z порогового значения 1.
// проверьте, не ниже ли ускорение по оси Z порогового значения 2.
// проверьте, не превышает ли ускорение по оси Z пороговое значение 2.
// проверьте, не превышает ли ускорение по оси Z пороговое значение 1.
void LIS3DSH_DoubleTurn_InterruptConfig(StateMachine_Num numStateMachine, float milliG_THRS1, float milliG_THRS2)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_Z; //enable positive Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_Z; //enable positive Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		ctrl |= LIS3DSH_SETT_ABS;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2
				
		ctrl = LIS3DSH_convertThreshold(milliG_THRS2); 						// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_2_ADDR,1);					// SM2
		ctrl = LIS3DSH_convertThreshold(milliG_THRS1); 						// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_2_ADDR,1);					// SM2
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LNTH1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_1_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_2_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_3_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_4_ADDR,1);						// SM2

		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_5_ADDR,1);						// SM2
	}
	else{
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_Z; //enable positive Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_Z; //enable positive Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		ctrl |= LIS3DSH_SETT_ABS;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1
				
		ctrl = LIS3DSH_convertThreshold(milliG_THRS2); 						// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_1_ADDR,1);					// SM1
		ctrl = LIS3DSH_convertThreshold(milliG_THRS1); 						// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_1_ADDR,1);					// SM1
		
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LNTH1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_1_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_2_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_3_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_4_ADDR,1);						// SM1

		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_5_ADDR,1);						// SM1
	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set LIS3DSH for If the object is in Double Tap, the X-axis, Y-axis and Z-axis have zero acceleration.
  * @param  Num State Machine,  
  * 	two - Allowed range
  * 			LIS3DSH_FULLSCALE_2:	// [15,625 - 3984]mg
  * 			LIS3DSH_FULLSCALE_4:	// [31.25 - 7968.75]mg
  * 			LIS3DSH_FULLSCALE_6:	// [46.875 - 11953.125]mg
  * 			LIS3DSH_FULLSCALE_8:	// [62.5 - 15937.5]mg
  * 			LIS3DSH_FULLSCALE_16:	// [125 - 31875]mg
	* 	three - time
  * 			LIS3DSH_DATARATE_3_125:		// 320.0ms - 81600.0ms
  * 			LIS3DSH_DATARATE_6_25:		// 160.0ms - 40800.0ms
  * 			LIS3DSH_DATARATE_12_5:		// 80.0ms - 20400.0ms
  * 			LIS3DSH_DATARATE_25:			// 40.0ms - 10200.0ms
  * 			LIS3DSH_DATARATE_50:			// 20.0ms - 5100.0ms
  * 			LIS3DSH_DATARATE_100:			// 10.0ms - 2550.0ms
  * 			LIS3DSH_DATARATE_400:			// 2.5ms - 637.5ms
  * 			LIS3DSH_DATARATE_800:			// 1.25ms - 318.75ms
  * 			LIS3DSH_DATARATE_1600:		// 0.625ms - 159.375ms
  * @retval None
  */
// двойной удар
void LIS3DSH_DoubleTap_InterruptConfig(StateMachine_Num numStateMachine, float milliG, float time)
{
  uint8_t ctrl = 0x00;
	
	if( STATE_MACHINE_TWO == numStateMachine ){
		
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_B_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK2_A_ADDR,1);					// SM2
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT2_ADDR,1);						// SM2
		
		ctrl = 0x02;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM4_2_ADDR,1);						// SM2		При обнаружении первого касания система ждет указанное время (TI4),
		ctrl = 0x01;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_2_ADDR,1);						// SM2		время окна (TI3), в котором ускорение сначала выше, а затем ниже, чем порог
		ctrl = LIS3DSH_convertTime(time);
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM2_2_L_ADDR,1);					// SM2		
		ctrl = 0x07;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_2_L_ADDR,1);					// SM2		для создания временного окна «до молчания»,
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_2_ADDR,1);					// SM2
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_2_ADDR,1);					// SM2
		
		ctrl = (LIS3DSH_GNTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_1_ADDR,1);						// SM2
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_2_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_3_ADDR,1);						// SM2
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_4_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI4 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_5_ADDR,1);						// SM2
		ctrl = (LIS3DSH_GTTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_6_ADDR,1);						// SM2
		
		ctrl = (LIS3DSH_TI2 LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_7_ADDR,1);						// SM2
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_8_ADDR,1);						// SM2
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI4 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_9_ADDR,1);						// SM2
		ctrl = (LIS3DSH_GTTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST2_10_ADDR,1);					// SM2
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST2_11_ADDR,1);						// SM2
	}
	else{
		// Set LIS3DSH State Machines configuration
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_B_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_MASK_A_P_X | LIS3DSH_MASK_A_P_Y | LIS3DSH_MASK_A_P_Z; //enable positive X, Y and Z in mask A
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_MASK1_A_ADDR,1);					// SM1
			
		ctrl = LIS3DSH_SETT_SITR; 			//STOP and CONT commands generate an interrupt and perform output actions as OUTC command
		ctrl |= LIS3DSH_SETT_R_TAM; 		//NEXT condition validation : standard mask always evaluated 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_SETT1_ADDR,1);						// SM1
		
		ctrl = 0x02;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM4_1_ADDR,1);						// SM1		При обнаружении первого касания система ждет указанное время (TI4),
		ctrl = 0x01;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM3_1_ADDR,1);						// SM1		время окна (TI3), в котором ускорение сначала выше, а затем ниже, чем порог
		ctrl = LIS3DSH_convertTime(time);
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM2_1_L_ADDR,1);					// SM1		
		ctrl = 0x07;
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_TIM1_1_L_ADDR,1);					// SM1		для создания временного окна «до молчания»,
		
		ctrl = LIS3DSH_convertThreshold(milliG); 									// чувствительность;
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS2_1_ADDR,1);					// SM1
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_THRS1_1_ADDR,1);					// SM1
		
		ctrl = (LIS3DSH_GNTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_1_ADDR,1);						// SM1
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_2_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_3_ADDR,1);						// SM1
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_4_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI4 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_5_ADDR,1);						// SM1
		ctrl = (LIS3DSH_GTTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_6_ADDR,1);						// SM1
		
		ctrl = (LIS3DSH_TI2 LIS3DSH_RESET) | (LIS3DSH_GNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_7_ADDR,1);						// SM1
		ctrl = (LIS3DSH_TI3 LIS3DSH_RESET) | (LIS3DSH_LNTH2 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_8_ADDR,1);						// SM1
		ctrl = (LIS3DSH_NOP LIS3DSH_RESET) | (LIS3DSH_TI4 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_9_ADDR,1);						// SM1
		ctrl = (LIS3DSH_GTTH1 LIS3DSH_RESET) | (LIS3DSH_TI1 LIS3DSH_NEXT);
		LIS3DSH_WriteReg(&ctrl,  LIS3DSH_ST1_10_ADDR,1);					// SM1
		
		ctrl = LIS3DSH_CONT; 
		LIS3DSH_WriteReg(&ctrl, LIS3DSH_ST1_11_ADDR,1);						// SM1		
	}
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Data Rate command. 
  * @param  DataRateValue: Data rate value.
  *   This parameter can be one of the following values:
  *     @arg LIS3DSH_DATARATE_POWERDOWN: Power down mode
  *     @arg LIS3DSH_DATARATE_3_125: Normal mode. ODR: 3.125 Hz
  *     @arg LIS3DSH_DATARATE_6_25: Normal mode. ODR: 6.25 Hz
  *     @arg LIS3DSH_DATARATE_12_5: Normal mode. ODR: 12.5 Hz
  *     @arg LIS3DSH_DATARATE_25: Normal mode. ODR: 25 Hz
  *     @arg LIS3DSH_DATARATE_50: Normal mode. ODR: 50 Hz
  *     @arg LIS3DSH_DATARATE_100: Normal mode. ODR: 100 Hz
  *     @arg LIS3DSH_DATARATE_400: Normal mode. ODR: 400 Hz
  *     @arg LIS3DSH_DATARATE_800: Normal mode. ODR: 800 Hz
  *     @arg LIS3DSH_DATARATE_1600: Normal mode. ODR: 1600 Hz
  * @retval None
  */
void LIS3DSH_DataRateCmd(uint8_t DataRateValue)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG4 register */
  LIS3DSH_ReadReg(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);
  
  /* Set new data rate configuration */
  tmpreg &= 0x0F; 
  tmpreg |= DataRateValue;
  
  /* Write value to MEMS CTRL_REG4 register */
  LIS3DSH_WriteReg(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Change the Full Scale of LIS3DSH.
  * @param  FS_value: new full scale value. 
  *   This parameter can be one of the following values:
  *     @arg LIS3DSH_FULLSCALE_2: +-2g
  *     @arg LIS3DSH_FULLSCALE_4: +-4g
  *     @arg LIS3DSH_FULLSCALE_6: +-6g
  *     @arg LIS3DSH_FULLSCALE_8: +-8g
  *     @arg LIS3DSH_FULLSCALE_16: +-16g
  * @retval None
  */
void LIS3DSH_FullScaleCmd(uint8_t FS_value)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  LIS3DSH_ReadReg(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
  
  /* Set new full scale configuration */
  tmpreg &= (uint8_t)~LIS3DSH__FULLSCALE_SELECTION;
  tmpreg |= FS_value;
  
  /* Write value to MEMS CTRL_REG5 register */
  LIS3DSH_WriteReg(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Reboot memory content of LIS3DSH.
  * @param  None
  * @retval None
  */
void LIS3DSH_RebootCmd(void)
{
  uint8_t tmpreg;
  /* Read CTRL_REG6 register */
  LIS3DSH_ReadReg(&tmpreg, LIS3DSH_CTRL_REG6_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= LIS3DSH_BOOT_FORCED;
  
  /* Write value to MEMS CTRL_REG6 register */
  LIS3DSH_WriteReg(&tmpreg, LIS3DSH_CTRL_REG6_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Read LIS3DSH output register, and calculate the acceleration 
  *         ACC[mg]=SENSITIVITY* (out_h*256+out_l)/16 (12 bit representation).
  * @param  pointer on floating buffer.
  * @retval None
  */
void LIS3DSH_ReadXYZ(int16_t *pData)
{
  int8_t buffer[6];
  uint8_t crtl, i = 0x00;
	float sensitivity = 0;
  float valueinfloat = 0;
  
  LIS3DSH_ReadReg(&crtl, LIS3DSH_CTRL_REG5_ADDR, 1);  
  LIS3DSH_ReadReg((uint8_t*)&buffer[0], LIS3DSH_OUT_X_L_ADDR, 1);
  LIS3DSH_ReadReg((uint8_t*)&buffer[1], LIS3DSH_OUT_X_H_ADDR, 1);
  LIS3DSH_ReadReg((uint8_t*)&buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1);
  LIS3DSH_ReadReg((uint8_t*)&buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1);
  LIS3DSH_ReadReg((uint8_t*)&buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1);
  LIS3DSH_ReadReg((uint8_t*)&buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1);
  
  switch(crtl & LIS3DSH__FULLSCALE_SELECTION) 
  {
    /* FS bit = 000 ==> Sensitivity typical value = 0.06milligals/digit */ 
  case LIS3DSH_FULLSCALE_2:
    sensitivity = LIS3DSH_SENSITIVITY_0_06G;
    break;
    
    /* FS bit = 001 ==> Sensitivity typical value = 0.12milligals/digit */ 
  case LIS3DSH_FULLSCALE_4:
    sensitivity = LIS3DSH_SENSITIVITY_0_12G;
    break;
    
    /* FS bit = 010 ==> Sensitivity typical value = 0.18milligals/digit */ 
  case LIS3DSH_FULLSCALE_6:
    sensitivity = LIS3DSH_SENSITIVITY_0_18G;
    break;
    
    /* FS bit = 011 ==> Sensitivity typical value = 0.24milligals/digit */ 
  case LIS3DSH_FULLSCALE_8:
    sensitivity = LIS3DSH_SENSITIVITY_0_24G;
    break;
    
    /* FS bit = 100 ==> Sensitivity typical value = 0.73milligals/digit */ 
  case LIS3DSH_FULLSCALE_16:
    sensitivity = LIS3DSH_SENSITIVITY_0_73G;
    break;
    
  default:
    break;
  }
  
  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    valueinfloat = ((buffer[2*i+1] << 8) + buffer[2*i]) * sensitivity;
    pData[i] = (int16_t)valueinfloat;
  }
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Set OFFSET LIS3DSH.
  * @param  OFFSET -> Output(axis) = Measurement(axis) - OFFSET_x(axis) * 32
  * @retval None
  */
void LIS3DSH_setAxisOffsetXYZ(int8_t* offset_value)
{
  LIS3DSH_WriteReg((uint8_t*)&offset_value[0], LIS3DSH_OFF_X_ADDR, 1);
	LIS3DSH_WriteReg((uint8_t*)&offset_value[1], LIS3DSH_OFF_Y_ADDR, 1);
	LIS3DSH_WriteReg((uint8_t*)&offset_value[2], LIS3DSH_OFF_Z_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Get OFFSET LIS3DSH.
  * @param  OFFSET -> Output(axis) = Measurement(axis) - OFFSET_x(axis) * 32
  * @retval None
  */
// смещение по каждой оси можно компенсировать от -4095 до 4096 младших
// битов с шагом в 32 младших бита.
void LIS3DSH_getAxisOffsetXYZ(int8_t* offset_value)
{
  LIS3DSH_ReadReg((uint8_t*)&offset_value[0], LIS3DSH_OFF_X_ADDR, 1);
	LIS3DSH_ReadReg((uint8_t*)&offset_value[1], LIS3DSH_OFF_Y_ADDR, 1);
	LIS3DSH_ReadReg((uint8_t*)&offset_value[2], LIS3DSH_OFF_Z_ADDR, 1);
}
//-------------------------------------------------------------------------


//==================================================================================================================
// Калибровка осей акселерометра ( нужна для получения градусов наклона,
// если это не нужно калибровать нет смысла, для более точного получения градусов наклона устанавливать на LIS3DSH_FULLSCALE_8 )
// запускаем калибровку после установки настроек LIS3DSH_Init(&myAccConfigDef);
// в момент калибровки нужно вращать ( крутить ) датчик во всех направлениях ( но окуратно без ударов и резких движений так как они некоректно влияют на калибровку )
// в этот момент калибровка будет записывать в функцию максимум и минимум значения как только значения запишуться и новых небудет 
// то по истечении времени #define TIME_CALIB_END калибровка закончиться и в датчик будут записаны смещения LIS3DSH_setAxisOffsetXYZ для всех осей
// эти данные храняться в датчике пока есть питание ( после перезагрузки там будут 0 )
// если мы хотим при следующем включении не калибровать то можно после калибровки 
// с помощью функций LIS3DSH_пetAxisOffsetXYZ считать все смещения и записать в EEPROM. 
// а при запуске устройства оттуда считывать и уже напрямую записывать в датчик без калибровки с помощью функции LIS3DSH_setAxisOffsetXYZ
void LIS3DSH_AccelerometerCalibration(void)
{
		// время которое пройдет после того какне будут изменены значения
		#define TIME_CALIB_END			10000		// millisec
		
		int AccelMinX = 0;
		int AccelMaxX = 0;
		int AccelMinY = 0;
		int AccelMaxY = 0;
		int AccelMinZ = 0;
		int AccelMaxZ = 0; 

		float offsetAxisX = 0.0f;
		float offsetAxisY = 0.0f;
		float offsetAxisZ = 0.0f;
		
		int16_t AccXYZ[3] = {0, };
		int8_t offset_xyz[3] = { 0, 0, 0 };
		
		// обнуляем смещение
		LIS3DSH_setAxisOffsetXYZ( offset_xyz );
				
		uint32_t time_calib_end = HAL_GetTick();
		
		// постонно считываем данные ( ищем максимум и минимум )
		// как только новых данных не будет пройдет указаное время и калибровка будет завершена
		while( HAL_GetTick() - time_calib_end < TIME_CALIB_END ){
			
				// получаем данные по осям ( также их можно получать по прерыванию )
				// считываем данные с осей XYZ
				LIS3DSH_ReadXYZ( AccXYZ );

			  if(AccXYZ[0] < AccelMinX) {AccelMinX = AccXYZ[0]; time_calib_end = HAL_GetTick(); };
				if(AccXYZ[0] > AccelMaxX) {AccelMaxX = AccXYZ[0]; time_calib_end = HAL_GetTick(); };

				if(AccXYZ[1] < AccelMinY) {AccelMinY = AccXYZ[1]; time_calib_end = HAL_GetTick(); };
				if(AccXYZ[1] > AccelMaxY) {AccelMaxY = AccXYZ[1]; time_calib_end = HAL_GetTick(); };

				if(AccXYZ[2] < AccelMinZ) {AccelMinZ = AccXYZ[2]; time_calib_end = HAL_GetTick(); };
				if(AccXYZ[2] > AccelMaxZ) {AccelMaxZ = AccXYZ[2]; time_calib_end = HAL_GetTick(); };
	
				offsetAxisX = 0.5f * (AccelMaxX + AccelMinX);
				offsetAxisY = 0.5f * (AccelMaxY + AccelMinY);
				offsetAxisZ = 0.5f * (AccelMaxZ + AccelMinZ);
				
				
			}
		
			// записываем в датчик смещение
			// Output(axis) = Measurement(axis) - OFFSET_x(axis) * 32
			offset_xyz[0] =	(offsetAxisX / 32.0f);	
			offset_xyz[1] =	(offsetAxisY / 32.0f);
			offset_xyz[2] =	(offsetAxisZ / 32.0f);
			LIS3DSH_setAxisOffsetXYZ( offset_xyz );			
}
//==================================================================================================================


//-------------------------------------------------------------------------
/**
  * @brief  Get Temperature (degrees Celsius).
  * @param  None
  * @retval degrees Celsius
  */
int8_t LIS3DSH_getTemperature(void)
{
		int8_t data;
		LIS3DSH_ReadReg((uint8_t*)&data, LIS3DSH_OUT_T_ADDR, 1);
		return (data + 25);		// The resolution is 1 LSB/deg and 00h corresponds to 25 degrees Celsius.

}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Flag new data ready
  * @param  None
  * @retval flag redy new data 0- not  1 -ready
  */
uint8_t LIS3DSH_newDataRedy(void)
{
		int8_t data;
		LIS3DSH_ReadReg((uint8_t*)&data, LIS3DSH_STAT_ADDR, 1);
		return (data & LIS3DSH_STAT_NEW_DATA_READY);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Clear Flag Interrupt State Machine 1
  * @param  None
  * @retval None
  */
void LIS3DSH_clearInterruptSM1(void)
{
		int8_t data;
		LIS3DSH_ReadReg((uint8_t*)&data, LIS3DSH_OUTS1_ADDR, 1);
}
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/**
  * @brief  Clear Flag Interrupt State Machine 2
  * @param  None
  * @retval None
  */
void LIS3DSH_clearInterruptSM2(void)
{
		int8_t data;
		LIS3DSH_ReadReg((uint8_t*)&data, LIS3DSH_OUTS2_ADDR, 1);
}
//-------------------------------------------------------------------------


//----------------------------------------------------------------------------------

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
