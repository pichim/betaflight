/**
  ******************************************************************************
  * @file    system_stm32g4xx.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32G4xx devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#pragma once

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32g4xx_system
  * @{
  */

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup STM32G4xx_System_Includes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
extern uint32_t SystemCoreClock;            /*!< System Clock Frequency (Core Clock) */

extern const uint8_t  AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8];     /*!< APB prescalers table values */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

extern void systemClockSetHSEValue(uint32_t frequency);
extern void OverclockRebootIfNecessary(unsigned requestedOverclockLevel);
extern int SystemSYSCLKSource(void);
extern int SystemPLLSource(void);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
