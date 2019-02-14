/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define T_D2_Pin GPIO_PIN_2
#define T_D2_GPIO_Port GPIOE
#define T_D3_Pin GPIO_PIN_3
#define T_D3_GPIO_Port GPIOE
#define T_D4_Pin GPIO_PIN_4
#define T_D4_GPIO_Port GPIOE
#define T_D5_Pin GPIO_PIN_5
#define T_D5_GPIO_Port GPIOE
#define T_D6_Pin GPIO_PIN_6
#define T_D6_GPIO_Port GPIOE
#define STAT_Pin GPIO_PIN_13
#define STAT_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PC0_Pin GPIO_PIN_0
#define PC0_GPIO_Port GPIOC
#define PC1_Pin GPIO_PIN_1
#define PC1_GPIO_Port GPIOC
#define PC2_Pin GPIO_PIN_2
#define PC2_GPIO_Port GPIOC
#define PC3_Pin GPIO_PIN_3
#define PC3_GPIO_Port GPIOC
#define VSENCE_Pin GPIO_PIN_0
#define VSENCE_GPIO_Port GPIOA
#define PA1_Pin GPIO_PIN_1
#define PA1_GPIO_Port GPIOA
#define PA2_Pin GPIO_PIN_2
#define PA2_GPIO_Port GPIOA
#define PA3_Pin GPIO_PIN_3
#define PA3_GPIO_Port GPIOA
#define PA4_Pin GPIO_PIN_4
#define PA4_GPIO_Port GPIOA
#define PA5_Pin GPIO_PIN_5
#define PA5_GPIO_Port GPIOA
#define PA6_Pin GPIO_PIN_6
#define PA6_GPIO_Port GPIOA
#define PA7_Pin GPIO_PIN_7
#define PA7_GPIO_Port GPIOA
#define PC4_Pin GPIO_PIN_4
#define PC4_GPIO_Port GPIOC
#define PC5_Pin GPIO_PIN_5
#define PC5_GPIO_Port GPIOC
#define LCD_YD_Pin GPIO_PIN_0
#define LCD_YD_GPIO_Port GPIOB
#define LCD_XL_Pin GPIO_PIN_1
#define LCD_XL_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define T_D7_Pin GPIO_PIN_7
#define T_D7_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOE
#define LCD_BLED_Pin GPIO_PIN_9
#define LCD_BLED_GPIO_Port GPIOE
#define PMRD_Pin GPIO_PIN_10
#define PMRD_GPIO_Port GPIOE
#define PMWR_Pin GPIO_PIN_11
#define PMWR_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOE
#define PE13_Pin GPIO_PIN_13
#define PE13_GPIO_Port GPIOE
#define PE14_Pin GPIO_PIN_14
#define PE14_GPIO_Port GPIOE
#define LCD_CS__Pin GPIO_PIN_15
#define LCD_CS__GPIO_Port GPIOE
#define PB10_Pin GPIO_PIN_10
#define PB10_GPIO_Port GPIOB
#define PB11_Pin GPIO_PIN_11
#define PB11_GPIO_Port GPIOB
#define PB12_Pin GPIO_PIN_12
#define PB12_GPIO_Port GPIOB
#define PD8_Pin GPIO_PIN_8
#define PD8_GPIO_Port GPIOD
#define PD9_Pin GPIO_PIN_9
#define PD9_GPIO_Port GPIOD
#define PD10_Pin GPIO_PIN_10
#define PD10_GPIO_Port GPIOD
#define PD11_Pin GPIO_PIN_11
#define PD11_GPIO_Port GPIOD
#define PD12_Pin GPIO_PIN_12
#define PD12_GPIO_Port GPIOD
#define PD13_Pin GPIO_PIN_13
#define PD13_GPIO_Port GPIOD
#define PD14_Pin GPIO_PIN_14
#define PD14_GPIO_Port GPIOD
#define SD_CD__Pin GPIO_PIN_15
#define SD_CD__GPIO_Port GPIOD
#define MP3_DREQ_Pin GPIO_PIN_6
#define MP3_DREQ_GPIO_Port GPIOC
#define MP3_RST__Pin GPIO_PIN_7
#define MP3_RST__GPIO_Port GPIOC
#define MP3_CS__Pin GPIO_PIN_8
#define MP3_CS__GPIO_Port GPIOC
#define MP3_DCS_Pin GPIO_PIN_9
#define MP3_DCS_GPIO_Port GPIOC
#define PA8_Pin GPIO_PIN_8
#define PA8_GPIO_Port GPIOA
#define UDB_DET_Pin GPIO_PIN_9
#define UDB_DET_GPIO_Port GPIOA
#define PA10_Pin GPIO_PIN_10
#define PA10_GPIO_Port GPIOA
#define USB_D_N_Pin GPIO_PIN_11
#define USB_D_N_GPIO_Port GPIOA
#define USB_D_P_Pin GPIO_PIN_12
#define USB_D_P_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TDI_Pin GPIO_PIN_15
#define TDI_GPIO_Port GPIOA
#define PD0_Pin GPIO_PIN_0
#define PD0_GPIO_Port GPIOD
#define PD1_Pin GPIO_PIN_1
#define PD1_GPIO_Port GPIOD
#define PD2_Pin GPIO_PIN_2
#define PD2_GPIO_Port GPIOD
#define SD_CD_D3_Pin GPIO_PIN_3
#define SD_CD_D3_GPIO_Port GPIOD
#define PD4_Pin GPIO_PIN_4
#define PD4_GPIO_Port GPIOD
#define FLASH_CS__Pin GPIO_PIN_7
#define FLASH_CS__GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PB5_Pin GPIO_PIN_5
#define PB5_GPIO_Port GPIOB
#define DRIVEA_Pin GPIO_PIN_8
#define DRIVEA_GPIO_Port GPIOB
#define DRIVEB_Pin GPIO_PIN_9
#define DRIVEB_GPIO_Port GPIOB
#define T_D0_Pin GPIO_PIN_0
#define T_D0_GPIO_Port GPIOE
#define T_D1_Pin GPIO_PIN_1
#define T_D1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
