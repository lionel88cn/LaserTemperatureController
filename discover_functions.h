 /**
  ******************************************************************************
  * @file   discover_functions.h
  * @author  Microcontroller Division
  * @version V1.0.3
  * @date    May-2013
  * @brief   This file contains measurement values and board
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISCOVER_FUNCTIONS_H
#define __DISCOVER_FUNCTIONS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"  

//#define SLIDER_DETECTED (sMCKeyInfo[0].Setting.b.DETECTED)
//#define SLIDER_POSITION (sMCKeyInfo[0].UnScaledPosition)

#define enableGlobalInterrupts()   __set_PRIMASK(0);
#define disableGlobalInterrupts()  __set_PRIMASK(1);

#define STR_VERSION     tab[1] = 'V';tab[2] = '2'|DOT; tab[3] = '0'|DOT; tab[4] = '4'

#define STATE_VREF	        0
#define STATE_SLIDER_VALUE 	1
#define STATE_SLIDER_BUTTON 	2


#define MAX_CURRENT 	99999
  

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
  
/* Exported functions ------------------------------------------------------- */

void convert_into_char(uint32_t number, uint16_t *p_tab);


#endif /* __DISCOVER_FUNCTIONS_H*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
