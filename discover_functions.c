 /**
 ******************************************************************************
 * @file    discover_functions.c
 * @author  Microcontroller Division
  * @version V1.0.3
  * @date    May-2013
 * @brief   Discover demo functions
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

/* Includes ------------------------------------------------------------------*/

/* stm32l1xxx std peripheral drivers headers*/ 
#include "stm32l1xx_exti.h"
#include "misc.h"

/* touch sensing library headers*/ 
//#include "stm32_tsl_api.h" -- superseded
//#include "stm32l15x_tsl_ct_acquisition.h" -- superseded
#include "tsl.h"
#include "tsl_user.h"
/* discover application headers*/ 
#include "discover_board.h"
#include "discover_functions.h"
#include "stm32l_discovery_lcd.h"


/**
  * @brief converts a 32bit unsined int into ASCII 
  * @caller several callers for display values
  * @param Number digit to displays
  *  p_tab values in array in ASCII   
  * @retval None
  */ 
void convert_into_char(uint32_t number, uint16_t *p_tab)
{
  uint16_t units=0, tens=0, hundreds=0, thousands=0, misc=0;
  
  units = (((number%10000)%1000)%100)%10;
  tens = ((((number-units)/10)%1000)%100)%10;
  hundreds = (((number-tens-units)/100))%100%10;
  thousands = ((number-hundreds-tens-units)/1000)%10;
  misc = ((number-thousands-hundreds-tens-units)/10000);
  
  *(p_tab+4) = units + 0x30;
  *(p_tab+3) = tens + 0x30;
  *(p_tab+2) = hundreds + 0x30;
  *(p_tab+1) = thousands + 0x30;
  *(p_tab) = misc + 0x30;

}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
