/**
  @page IWDG_WindowMode IWDG Reset with window mode
  
  @verbatim
  ********************* COPYRIGHT(c) 2019 STMicroelectronics *******************
  * @file    IWDG/IWDG_WindowMode/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the IWDG Reset with window mode.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

How to periodically update the IWDG reload counter and simulate a software fault
that generates an MCU IWDG reset when a programmed time period has elapsed.

CPU1 (Cortex-M7) and CPU2 (Cortex-M4) are booting at once (with respect to configured boot Flash options)
System Init, System clock, voltage scaling and L1-Cache configuration are done by CPU1 (Cortex-M7).
In the meantime Domain D2 is put in STOP mode (CPU2: Cortex-M4 in deep sleep mode) to save power consumption.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
The SystemClock_Config() function is used to configure the system clock for STM32H747xx Devices :
The CPU1 at 400MHz 
The HCLK for D1 Domain AXI and AHB3 peripherals , D2 Domain AHB1/AHB2 peripherals and D3 Domain AHB4  peripherals at 200MHz.
The APB clock dividers for D1 Domain APB3 peripherals, D2 Domain APB1 and APB2 peripherals and D3 Domain APB4 peripherals to  run at 100MHz.

The IWDG time-out is set to 762 ms (the time-out may vary due to LSI frequency 
dispersion).

The Window option is enabled with a window register value set to 400 ms.
To prevent a reset, the down-counter must be reloaded when its value is:
 -lower than the window register value (400ms)
 -greater than 0x0
The IWDG counter is therefore refreshed each 450 ms in the main program infinite loop to 
prevent a IWDG reset (762 - 450 = 312 within the interval).
LED1 is also toggled each 450 ms indicating that the program is running. 
LED3 will turn on if any error occurs.

An EXTI Line is connected to a GPIO pin, and configured to generate an interrupt
on the rising edge of the signal.

Once the EXTI Line event occurs by pressing the Tamper push-button (PC.13), 
the refresh period is set to 200 ms.
That will make refresh being outside window value. As a result, when the IWDG counter is reloaded, 
the IWDG reset occurs.

If the IWDG reset is generated, after the system resumes from reset, LED1 turns on for 4 seconds.

If the EXTI Line event does not occur, the IWDG counter is indefinitely refreshed
in the main program infinite loop, and there is no IWDG reset.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then 
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The example need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@Note If the  application is using the DTCM/ITCM memories (@0x20000000/ 0x0000000: not cacheable and only accessible
      by the Cortex M7 and the  MDMA), no need for cache maintenance when the Cortex M7 and the MDMA access these RAMs.
      If the application needs to use DMA(or other masters) based access or requires more RAM, then  the user has to:
              - Use a non TCM SRAM. (example : D1 AXI-SRAM @ 0x24000000)
              - Add a cache maintenance mechanism to ensure the cache coherence between CPU and other masters(DMAs,DMA2D,LTDC,MDMA).
              - The addresses and the size of cacheable buffers (shared between CPU and other masters)
                must be	properly defined to be aligned to L1-CACHE line size (32 bytes). 
 
@Note It is recommended to enable the cache and maintain its coherence.
      Depending on the use case it is also possible to configure the cache attributes using the MPU.
      Please refer to the AN4838 "Managing memory protection unit (MPU) in STM32 MCUs"
      Please refer to the AN4839 "Level 1 cache on STM32F7 Series"

@par Keywords

System, IWDG, Timeout, Reload Counter, MCU Reset, Downcounter, LSI, Timer, Measure Frequency, Window

@par Directory contents 

  - IWDG/IWDG_WindowMode/Common/Src/system_stm32h7xx.c     STM32H7xx system configuration file
  - IWDG/IWDG_WindowMode/CM7/Src/main.c                    Main program for Cortex-M7
  - IWDG/IWDG_WindowMode/CM7/Src/stm32h7xx_it.c            Interrupt handlers for Cortex-M7
  - IWDG/IWDG_WindowMode/CM7/Inc/main.h                    Main program header file for Cortex-M7  
  - IWDG/IWDG_WindowMode/CM7/Inc/stm32h7xx_hal_conf.h      HAL Configuration file for Cortex-M7
  - IWDG/IWDG_WindowMode/CM7/Inc/stm32h7xx_it.h            Interrupt handlers header file for Cortex-M7

  - IWDG/IWDG_WindowMode/CM4/Src/main.c                    Main program for Cortex-M4
  - IWDG/IWDG_WindowMode/CM4/Src/stm32h7xx_it.c            Interrupt handlers for Cortex-M4
  - IWDG/IWDG_WindowMode/CM4/Src/stm32h7xx_hal_msp.c       HAL MSP module for Cortex-M4
  - IWDG/IWDG_WindowMode/CM4/Inc/main.h                    Main program header file for Cortex-M4  
  - IWDG/IWDG_WindowMode/CM4/Inc/stm32h7xx_hal_conf.h      HAL Configuration file for Cortex-M4
  - IWDG/IWDG_WindowMode/CM4/Inc/stm32h7xx_it.h            Interrupt handlers header file for Cortex-M4
  

@par Hardware and Software environment

  - This example runs on STM32H747xx devices.
    
  - This example has been tested with STM32H747I-DISCO board with SMPS (SD Convertor) power supply 
   config and can be easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - For each target configuration (STM32H747I_DISCO_CM7 and STM32H747I_DISCO_CM4) : 
     - Rebuild all files 
     - Load images into target memory
 - After loading the two images, you have to reset the board in order to boot (Cortex-M7) and CPU2 (Cortex-M4) at once.
 - Run the example



 */
