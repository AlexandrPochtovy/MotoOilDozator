/*********************************************************************************
	Original author: user

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
 * Peripherals.c
 * Created on: Aug 24, 2023
 ********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Peripherals.h"

/* CRC init function */
void HardwareInit(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC | LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                           LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3 | LL_APB1_GRP1_PERIPH_TIM14);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1 | LL_APB1_GRP2_PERIPH_TIM16 | LL_APB1_GRP2_PERIPH_TIM17);

  /*  CRC setup  */
  CRC->CR = 0;
  CRC->INIT = LL_CRC_DEFAULT_CRC_INITVALUE;
}
