/**
  *************** (C) COPYRIGHT 2017 STMicroelectronics ************************
  * @file      startup_stm32f103xb.s
  * @author    MCD Application Team
  * @brief     STM32F103xB Devices vector table for Atollic toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the clock system   
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M3 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
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

  .syntax unified
  .arch armv6-m
  .cpu cortex-m0plus
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
/* Call static constructors */
/*    bl __libc_init_array */
/* Call the application's entry point.*/
  bl entry
  bx lr
.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word 0
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word IRQ000_Handler
    .word IRQ001_Handler
    .word IRQ002_Handler
    .word IRQ003_Handler
    .word IRQ004_Handler
    .word IRQ005_Handler
    .word IRQ006_Handler
    .word IRQ007_Handler
    .word IRQ008_Handler
    .word IRQ009_Handler
    .word IRQ010_Handler
    .word IRQ011_Handler
    .word IRQ012_Handler
    .word IRQ013_Handler
    .word IRQ014_Handler
    .word IRQ015_Handler
    .word IRQ016_Handler
    .word IRQ017_Handler
    .word IRQ018_Handler
    .word IRQ019_Handler
    .word IRQ020_Handler
    .word IRQ021_Handler
    .word IRQ022_Handler
    .word IRQ023_Handler
    .word IRQ024_Handler
    .word IRQ025_Handler
    .word IRQ026_Handler
    .word IRQ027_Handler
    .word IRQ028_Handler
    .word IRQ029_Handler
    .word IRQ030_Handler
    .word IRQ031_Handler

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

// 中断处理函数的默认实现
.weak IRQ000_Handler
.thumb_set IRQ000_Handler, Default_Handler

.weak IRQ001_Handler
.thumb_set IRQ001_Handler, Default_Handler

.weak IRQ002_Handler
.thumb_set IRQ002_Handler, Default_Handler

.weak IRQ003_Handler
.thumb_set IRQ003_Handler, Default_Handler

.weak IRQ004_Handler
.thumb_set IRQ004_Handler, Default_Handler

.weak IRQ005_Handler
.thumb_set IRQ005_Handler, Default_Handler

.weak IRQ006_Handler
.thumb_set IRQ006_Handler, Default_Handler

.weak IRQ007_Handler
.thumb_set IRQ007_Handler, Default_Handler

.weak IRQ008_Handler
.thumb_set IRQ008_Handler, Default_Handler

.weak IRQ009_Handler
.thumb_set IRQ009_Handler, Default_Handler

.weak IRQ010_Handler
.thumb_set IRQ010_Handler, Default_Handler

.weak IRQ011_Handler
.thumb_set IRQ011_Handler, Default_Handler

.weak IRQ012_Handler
.thumb_set IRQ012_Handler, Default_Handler

.weak IRQ013_Handler
.thumb_set IRQ013_Handler, Default_Handler

.weak IRQ014_Handler
.thumb_set IRQ014_Handler, Default_Handler

.weak IRQ015_Handler
.thumb_set IRQ015_Handler, Default_Handler

.weak IRQ016_Handler
.thumb_set IRQ016_Handler, Default_Handler

.weak IRQ017_Handler
.thumb_set IRQ017_Handler, Default_Handler

.weak IRQ018_Handler
.thumb_set IRQ018_Handler, Default_Handler

.weak IRQ019_Handler
.thumb_set IRQ019_Handler, Default_Handler

.weak IRQ020_Handler
.thumb_set IRQ020_Handler, Default_Handler

.weak IRQ021_Handler
.thumb_set IRQ021_Handler, Default_Handler

.weak IRQ022_Handler
.thumb_set IRQ022_Handler, Default_Handler

.weak IRQ023_Handler
.thumb_set IRQ023_Handler, Default_Handler

.weak IRQ024_Handler
.thumb_set IRQ024_Handler, Default_Handler

.weak IRQ025_Handler
.thumb_set IRQ025_Handler, Default_Handler

.weak IRQ026_Handler
.thumb_set IRQ026_Handler, Default_Handler

.weak IRQ027_Handler
.thumb_set IRQ027_Handler, Default_Handler

.weak IRQ028_Handler
.thumb_set IRQ028_Handler, Default_Handler

.weak IRQ029_Handler
.thumb_set IRQ029_Handler, Default_Handler

.weak IRQ030_Handler
.thumb_set IRQ030_Handler, Default_Handler

.weak IRQ031_Handler
.thumb_set IRQ031_Handler, Default_Handler


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

