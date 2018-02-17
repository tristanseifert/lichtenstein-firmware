/**
  ******************************************************************************
  * @file    stm32f1xx_hal_rcc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    12-May-2017
  * @brief   Header file of RCC HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_HAL_RCC_H
#define __STM32F1xx_HAL_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

 /******************************************************************************/
 /*                                                                            */
 /*                         Reset and Clock Control                            */
 /*                                                                            */
 /******************************************************************************/

 /********************  Bit definition for RCC_CR register  ********************/
 #define RCC_CR_HSION_Pos                     (0U)
 #define RCC_CR_HSION_Msk                     (0x1U << RCC_CR_HSION_Pos)        /*!< 0x00000001 */
 #define RCC_CR_HSION                         RCC_CR_HSION_Msk                  /*!< Internal High Speed clock enable */
 #define RCC_CR_HSIRDY_Pos                    (1U)
 #define RCC_CR_HSIRDY_Msk                    (0x1U << RCC_CR_HSIRDY_Pos)       /*!< 0x00000002 */
 #define RCC_CR_HSIRDY                        RCC_CR_HSIRDY_Msk                 /*!< Internal High Speed clock ready flag */
 #define RCC_CR_HSITRIM_Pos                   (3U)
 #define RCC_CR_HSITRIM_Msk                   (0x1FU << RCC_CR_HSITRIM_Pos)     /*!< 0x000000F8 */
 #define RCC_CR_HSITRIM                       RCC_CR_HSITRIM_Msk                /*!< Internal High Speed clock trimming */
 #define RCC_CR_HSICAL_Pos                    (8U)
 #define RCC_CR_HSICAL_Msk                    (0xFFU << RCC_CR_HSICAL_Pos)      /*!< 0x0000FF00 */
 #define RCC_CR_HSICAL                        RCC_CR_HSICAL_Msk                 /*!< Internal High Speed clock Calibration */
 #define RCC_CR_HSEON_Pos                     (16U)
 #define RCC_CR_HSEON_Msk                     (0x1U << RCC_CR_HSEON_Pos)        /*!< 0x00010000 */
 #define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed clock enable */
 #define RCC_CR_HSERDY_Pos                    (17U)
 #define RCC_CR_HSERDY_Msk                    (0x1U << RCC_CR_HSERDY_Pos)       /*!< 0x00020000 */
 #define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed clock ready flag */
 #define RCC_CR_HSEBYP_Pos                    (18U)
 #define RCC_CR_HSEBYP_Msk                    (0x1U << RCC_CR_HSEBYP_Pos)       /*!< 0x00040000 */
 #define RCC_CR_HSEBYP                        RCC_CR_HSEBYP_Msk                 /*!< External High Speed clock Bypass */
 #define RCC_CR_CSSON_Pos                     (19U)
 #define RCC_CR_CSSON_Msk                     (0x1U << RCC_CR_CSSON_Pos)        /*!< 0x00080000 */
 #define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< Clock Security System enable */
 #define RCC_CR_PLLON_Pos                     (24U)
 #define RCC_CR_PLLON_Msk                     (0x1U << RCC_CR_PLLON_Pos)        /*!< 0x01000000 */
 #define RCC_CR_PLLON                         RCC_CR_PLLON_Msk                  /*!< PLL enable */
 #define RCC_CR_PLLRDY_Pos                    (25U)
 #define RCC_CR_PLLRDY_Msk                    (0x1U << RCC_CR_PLLRDY_Pos)       /*!< 0x02000000 */
 #define RCC_CR_PLLRDY                        RCC_CR_PLLRDY_Msk                 /*!< PLL clock ready flag */

 /*
  * @brief Specific device feature definitions (not present on all devices in the STM32F1 serie)
  */
 #define RCC_PLL2_SUPPORT                                                       /*!< Support PLL2 */

 #define RCC_CR_PLL2ON_Pos                    (26U)
 #define RCC_CR_PLL2ON_Msk                    (0x1U << RCC_CR_PLL2ON_Pos)       /*!< 0x04000000 */
 #define RCC_CR_PLL2ON                        RCC_CR_PLL2ON_Msk                 /*!< PLL2 enable */
 #define RCC_CR_PLL2RDY_Pos                   (27U)
 #define RCC_CR_PLL2RDY_Msk                   (0x1U << RCC_CR_PLL2RDY_Pos)      /*!< 0x08000000 */
 #define RCC_CR_PLL2RDY                       RCC_CR_PLL2RDY_Msk                /*!< PLL2 clock ready flag */

 /*
  * @brief Specific device feature definitions (not present on all devices in the STM32F1 serie)
  */
 #define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLL3 (PLLI2S)*/

 #define RCC_CR_PLL3ON_Pos                    (28U)
 #define RCC_CR_PLL3ON_Msk                    (0x1U << RCC_CR_PLL3ON_Pos)       /*!< 0x10000000 */
 #define RCC_CR_PLL3ON                        RCC_CR_PLL3ON_Msk                 /*!< PLL3 enable */
 #define RCC_CR_PLL3RDY_Pos                   (29U)
 #define RCC_CR_PLL3RDY_Msk                   (0x1U << RCC_CR_PLL3RDY_Pos)      /*!< 0x20000000 */
 #define RCC_CR_PLL3RDY                       RCC_CR_PLL3RDY_Msk                /*!< PLL3 clock ready flag */

 /*******************  Bit definition for RCC_CFGR register  *******************/
 /*!< SW configuration */
 #define RCC_CFGR_SW_Pos                      (0U)
 #define RCC_CFGR_SW_Msk                      (0x3U << RCC_CFGR_SW_Pos)         /*!< 0x00000003 */
 #define RCC_CFGR_SW                          RCC_CFGR_SW_Msk                   /*!< SW[1:0] bits (System clock Switch) */
 #define RCC_CFGR_SW_0                        (0x1U << RCC_CFGR_SW_Pos)         /*!< 0x00000001 */
 #define RCC_CFGR_SW_1                        (0x2U << RCC_CFGR_SW_Pos)         /*!< 0x00000002 */

 #define RCC_CFGR_SW_HSI                      0x00000000U                       /*!< HSI selected as system clock */
 #define RCC_CFGR_SW_HSE                      0x00000001U                       /*!< HSE selected as system clock */
 #define RCC_CFGR_SW_PLL                      0x00000002U                       /*!< PLL selected as system clock */

 /*!< SWS configuration */
 #define RCC_CFGR_SWS_Pos                     (2U)
 #define RCC_CFGR_SWS_Msk                     (0x3U << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
 #define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                  /*!< SWS[1:0] bits (System Clock Switch Status) */
 #define RCC_CFGR_SWS_0                       (0x1U << RCC_CFGR_SWS_Pos)        /*!< 0x00000004 */
 #define RCC_CFGR_SWS_1                       (0x2U << RCC_CFGR_SWS_Pos)        /*!< 0x00000008 */

 #define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
 #define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
 #define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

 /*!< HPRE configuration */
 #define RCC_CFGR_HPRE_Pos                    (4U)
 #define RCC_CFGR_HPRE_Msk                    (0xFU << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
 #define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
 #define RCC_CFGR_HPRE_0                      (0x1U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000010 */
 #define RCC_CFGR_HPRE_1                      (0x2U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000020 */
 #define RCC_CFGR_HPRE_2                      (0x4U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000040 */
 #define RCC_CFGR_HPRE_3                      (0x8U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000080 */

 #define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /*!< SYSCLK not divided */
 #define RCC_CFGR_HPRE_DIV2                   0x00000080U                       /*!< SYSCLK divided by 2 */
 #define RCC_CFGR_HPRE_DIV4                   0x00000090U                       /*!< SYSCLK divided by 4 */
 #define RCC_CFGR_HPRE_DIV8                   0x000000A0U                       /*!< SYSCLK divided by 8 */
 #define RCC_CFGR_HPRE_DIV16                  0x000000B0U                       /*!< SYSCLK divided by 16 */
 #define RCC_CFGR_HPRE_DIV64                  0x000000C0U                       /*!< SYSCLK divided by 64 */
 #define RCC_CFGR_HPRE_DIV128                 0x000000D0U                       /*!< SYSCLK divided by 128 */
 #define RCC_CFGR_HPRE_DIV256                 0x000000E0U                       /*!< SYSCLK divided by 256 */
 #define RCC_CFGR_HPRE_DIV512                 0x000000F0U                       /*!< SYSCLK divided by 512 */

 /*!< PPRE1 configuration */
 #define RCC_CFGR_PPRE1_Pos                   (8U)
 #define RCC_CFGR_PPRE1_Msk                   (0x7U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
 #define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
 #define RCC_CFGR_PPRE1_0                     (0x1U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000100 */
 #define RCC_CFGR_PPRE1_1                     (0x2U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000200 */
 #define RCC_CFGR_PPRE1_2                     (0x4U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000400 */

 #define RCC_CFGR_PPRE1_DIV1                  0x00000000U                       /*!< HCLK not divided */
 #define RCC_CFGR_PPRE1_DIV2                  0x00000400U                       /*!< HCLK divided by 2 */
 #define RCC_CFGR_PPRE1_DIV4                  0x00000500U                       /*!< HCLK divided by 4 */
 #define RCC_CFGR_PPRE1_DIV8                  0x00000600U                       /*!< HCLK divided by 8 */
 #define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

 /*!< PPRE2 configuration */
 #define RCC_CFGR_PPRE2_Pos                   (11U)
 #define RCC_CFGR_PPRE2_Msk                   (0x7U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
 #define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
 #define RCC_CFGR_PPRE2_0                     (0x1U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00000800 */
 #define RCC_CFGR_PPRE2_1                     (0x2U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00001000 */
 #define RCC_CFGR_PPRE2_2                     (0x4U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00002000 */

 #define RCC_CFGR_PPRE2_DIV1                  0x00000000U                       /*!< HCLK not divided */
 #define RCC_CFGR_PPRE2_DIV2                  0x00002000U                       /*!< HCLK divided by 2 */
 #define RCC_CFGR_PPRE2_DIV4                  0x00002800U                       /*!< HCLK divided by 4 */
 #define RCC_CFGR_PPRE2_DIV8                  0x00003000U                       /*!< HCLK divided by 8 */
 #define RCC_CFGR_PPRE2_DIV16                 0x00003800U                       /*!< HCLK divided by 16 */

 /*!< ADCPPRE configuration */
 #define RCC_CFGR_ADCPRE_Pos                  (14U)
 #define RCC_CFGR_ADCPRE_Msk                  (0x3U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x0000C000 */
 #define RCC_CFGR_ADCPRE                      RCC_CFGR_ADCPRE_Msk               /*!< ADCPRE[1:0] bits (ADC prescaler) */
 #define RCC_CFGR_ADCPRE_0                    (0x1U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00004000 */
 #define RCC_CFGR_ADCPRE_1                    (0x2U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00008000 */

 #define RCC_CFGR_ADCPRE_DIV2                 0x00000000U                       /*!< PCLK2 divided by 2 */
 #define RCC_CFGR_ADCPRE_DIV4                 0x00004000U                       /*!< PCLK2 divided by 4 */
 #define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /*!< PCLK2 divided by 6 */
 #define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /*!< PCLK2 divided by 8 */

 #define RCC_CFGR_PLLSRC_Pos                  (16U)
 #define RCC_CFGR_PLLSRC_Msk                  (0x1U << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
 #define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

 #define RCC_CFGR_PLLXTPRE_Pos                (17U)
 #define RCC_CFGR_PLLXTPRE_Msk                (0x1U << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
 #define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk             /*!< HSE divider for PLL entry */

 /*!< PLLMUL configuration */
 #define RCC_CFGR_PLLMULL_Pos                 (18U)
 #define RCC_CFGR_PLLMULL_Msk                 (0xFU << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
 #define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk              /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
 #define RCC_CFGR_PLLMULL_0                   (0x1U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00040000 */
 #define RCC_CFGR_PLLMULL_1                   (0x2U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00080000 */
 #define RCC_CFGR_PLLMULL_2                   (0x4U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00100000 */
 #define RCC_CFGR_PLLMULL_3                   (0x8U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00200000 */

 #define RCC_CFGR_PLLXTPRE_PREDIV1            0x00000000U                       /*!< PREDIV1 clock not divided for PLL entry */
 #define RCC_CFGR_PLLXTPRE_PREDIV1_DIV2       0x00020000U                       /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define RCC_CFGR_PLLMULL4_Pos                (19U)
 #define RCC_CFGR_PLLMULL4_Msk                (0x1U << RCC_CFGR_PLLMULL4_Pos)   /*!< 0x00080000 */
 #define RCC_CFGR_PLLMULL4                    RCC_CFGR_PLLMULL4_Msk             /*!< PLL input clock * 4 */
 #define RCC_CFGR_PLLMULL5_Pos                (18U)
 #define RCC_CFGR_PLLMULL5_Msk                (0x3U << RCC_CFGR_PLLMULL5_Pos)   /*!< 0x000C0000 */
 #define RCC_CFGR_PLLMULL5                    RCC_CFGR_PLLMULL5_Msk             /*!< PLL input clock * 5 */
 #define RCC_CFGR_PLLMULL6_Pos                (20U)
 #define RCC_CFGR_PLLMULL6_Msk                (0x1U << RCC_CFGR_PLLMULL6_Pos)   /*!< 0x00100000 */
 #define RCC_CFGR_PLLMULL6                    RCC_CFGR_PLLMULL6_Msk             /*!< PLL input clock * 6 */
 #define RCC_CFGR_PLLMULL7_Pos                (18U)
 #define RCC_CFGR_PLLMULL7_Msk                (0x5U << RCC_CFGR_PLLMULL7_Pos)   /*!< 0x00140000 */
 #define RCC_CFGR_PLLMULL7                    RCC_CFGR_PLLMULL7_Msk             /*!< PLL input clock * 7 */
 #define RCC_CFGR_PLLMULL8_Pos                (19U)
 #define RCC_CFGR_PLLMULL8_Msk                (0x3U << RCC_CFGR_PLLMULL8_Pos)   /*!< 0x00180000 */
 #define RCC_CFGR_PLLMULL8                    RCC_CFGR_PLLMULL8_Msk             /*!< PLL input clock * 8 */
 #define RCC_CFGR_PLLMULL9_Pos                (18U)
 #define RCC_CFGR_PLLMULL9_Msk                (0x7U << RCC_CFGR_PLLMULL9_Pos)   /*!< 0x001C0000 */
 #define RCC_CFGR_PLLMULL9                    RCC_CFGR_PLLMULL9_Msk             /*!< PLL input clock * 9 */
 #define RCC_CFGR_PLLMULL6_5                  0x00340000U                       /*!< PLL input clock * 6.5 */

 #define RCC_CFGR_OTGFSPRE_Pos                (22U)
 #define RCC_CFGR_OTGFSPRE_Msk                (0x1U << RCC_CFGR_OTGFSPRE_Pos)   /*!< 0x00400000 */
 #define RCC_CFGR_OTGFSPRE                    RCC_CFGR_OTGFSPRE_Msk             /*!< USB OTG FS prescaler */

 /*!< MCO configuration */
 #define RCC_CFGR_MCO_Pos                     (24U)
 #define RCC_CFGR_MCO_Msk                     (0xFU << RCC_CFGR_MCO_Pos)        /*!< 0x0F000000 */
 #define RCC_CFGR_MCO                         RCC_CFGR_MCO_Msk                  /*!< MCO[3:0] bits (Microcontroller Clock Output) */
 #define RCC_CFGR_MCO_0                       (0x1U << RCC_CFGR_MCO_Pos)        /*!< 0x01000000 */
 #define RCC_CFGR_MCO_1                       (0x2U << RCC_CFGR_MCO_Pos)        /*!< 0x02000000 */
 #define RCC_CFGR_MCO_2                       (0x4U << RCC_CFGR_MCO_Pos)        /*!< 0x04000000 */
 #define RCC_CFGR_MCO_3                       (0x8U << RCC_CFGR_MCO_Pos)        /*!< 0x08000000 */

 #define RCC_CFGR_MCO_NOCLOCK                 0x00000000U                       /*!< No clock */
 #define RCC_CFGR_MCO_SYSCLK                  0x04000000U                       /*!< System clock selected as MCO source */
 #define RCC_CFGR_MCO_HSI                     0x05000000U                       /*!< HSI clock selected as MCO source */
 #define RCC_CFGR_MCO_HSE                     0x06000000U                       /*!< HSE clock selected as MCO source */
 #define RCC_CFGR_MCO_PLLCLK_DIV2             0x07000000U                       /*!< PLL clock divided by 2 selected as MCO source */
 #define RCC_CFGR_MCO_PLL2CLK                 0x08000000U                       /*!< PLL2 clock selected as MCO source*/
 #define RCC_CFGR_MCO_PLL3CLK_DIV2            0x09000000U                       /*!< PLL3 clock divided by 2 selected as MCO source*/
 #define RCC_CFGR_MCO_EXT_HSE                 0x0A000000U                       /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
 #define RCC_CFGR_MCO_PLL3CLK                 0x0B000000U                       /*!< PLL3 clock selected as MCO source */

  /* Reference defines */
  #define RCC_CFGR_MCOSEL                      RCC_CFGR_MCO
  #define RCC_CFGR_MCOSEL_0                    RCC_CFGR_MCO_0
  #define RCC_CFGR_MCOSEL_1                    RCC_CFGR_MCO_1
  #define RCC_CFGR_MCOSEL_2                    RCC_CFGR_MCO_2
  #define RCC_CFGR_MCOSEL_3                    RCC_CFGR_MCO_3
  #define RCC_CFGR_MCOSEL_NOCLOCK              RCC_CFGR_MCO_NOCLOCK
  #define RCC_CFGR_MCOSEL_SYSCLK               RCC_CFGR_MCO_SYSCLK
  #define RCC_CFGR_MCOSEL_HSI                  RCC_CFGR_MCO_HSI
  #define RCC_CFGR_MCOSEL_HSE                  RCC_CFGR_MCO_HSE
  #define RCC_CFGR_MCOSEL_PLL_DIV2             RCC_CFGR_MCO_PLLCLK_DIV2
  #define RCC_CFGR_MCOSEL_PLL2                 RCC_CFGR_MCO_PLL2CLK
  #define RCC_CFGR_MCOSEL_PLL3_DIV2            RCC_CFGR_MCO_PLL3CLK_DIV2
  #define RCC_CFGR_MCOSEL_EXT_HSE              RCC_CFGR_MCO_EXT_HSE
  #define RCC_CFGR_MCOSEL_PLL3CLK              RCC_CFGR_MCO_PLL3CLK

 /*!<******************  Bit definition for RCC_CIR register  ********************/
 #define RCC_CIR_LSIRDYF_Pos                  (0U)
 #define RCC_CIR_LSIRDYF_Msk                  (0x1U << RCC_CIR_LSIRDYF_Pos)     /*!< 0x00000001 */
 #define RCC_CIR_LSIRDYF                      RCC_CIR_LSIRDYF_Msk               /*!< LSI Ready Interrupt flag */
 #define RCC_CIR_LSERDYF_Pos                  (1U)
 #define RCC_CIR_LSERDYF_Msk                  (0x1U << RCC_CIR_LSERDYF_Pos)     /*!< 0x00000002 */
 #define RCC_CIR_LSERDYF                      RCC_CIR_LSERDYF_Msk               /*!< LSE Ready Interrupt flag */
 #define RCC_CIR_HSIRDYF_Pos                  (2U)
 #define RCC_CIR_HSIRDYF_Msk                  (0x1U << RCC_CIR_HSIRDYF_Pos)     /*!< 0x00000004 */
 #define RCC_CIR_HSIRDYF                      RCC_CIR_HSIRDYF_Msk               /*!< HSI Ready Interrupt flag */
 #define RCC_CIR_HSERDYF_Pos                  (3U)
 #define RCC_CIR_HSERDYF_Msk                  (0x1U << RCC_CIR_HSERDYF_Pos)     /*!< 0x00000008 */
 #define RCC_CIR_HSERDYF                      RCC_CIR_HSERDYF_Msk               /*!< HSE Ready Interrupt flag */
 #define RCC_CIR_PLLRDYF_Pos                  (4U)
 #define RCC_CIR_PLLRDYF_Msk                  (0x1U << RCC_CIR_PLLRDYF_Pos)     /*!< 0x00000010 */
 #define RCC_CIR_PLLRDYF                      RCC_CIR_PLLRDYF_Msk               /*!< PLL Ready Interrupt flag */
 #define RCC_CIR_CSSF_Pos                     (7U)
 #define RCC_CIR_CSSF_Msk                     (0x1U << RCC_CIR_CSSF_Pos)        /*!< 0x00000080 */
 #define RCC_CIR_CSSF                         RCC_CIR_CSSF_Msk                  /*!< Clock Security System Interrupt flag */
 #define RCC_CIR_LSIRDYIE_Pos                 (8U)
 #define RCC_CIR_LSIRDYIE_Msk                 (0x1U << RCC_CIR_LSIRDYIE_Pos)    /*!< 0x00000100 */
 #define RCC_CIR_LSIRDYIE                     RCC_CIR_LSIRDYIE_Msk              /*!< LSI Ready Interrupt Enable */
 #define RCC_CIR_LSERDYIE_Pos                 (9U)
 #define RCC_CIR_LSERDYIE_Msk                 (0x1U << RCC_CIR_LSERDYIE_Pos)    /*!< 0x00000200 */
 #define RCC_CIR_LSERDYIE                     RCC_CIR_LSERDYIE_Msk              /*!< LSE Ready Interrupt Enable */
 #define RCC_CIR_HSIRDYIE_Pos                 (10U)
 #define RCC_CIR_HSIRDYIE_Msk                 (0x1U << RCC_CIR_HSIRDYIE_Pos)    /*!< 0x00000400 */
 #define RCC_CIR_HSIRDYIE                     RCC_CIR_HSIRDYIE_Msk              /*!< HSI Ready Interrupt Enable */
 #define RCC_CIR_HSERDYIE_Pos                 (11U)
 #define RCC_CIR_HSERDYIE_Msk                 (0x1U << RCC_CIR_HSERDYIE_Pos)    /*!< 0x00000800 */
 #define RCC_CIR_HSERDYIE                     RCC_CIR_HSERDYIE_Msk              /*!< HSE Ready Interrupt Enable */
 #define RCC_CIR_PLLRDYIE_Pos                 (12U)
 #define RCC_CIR_PLLRDYIE_Msk                 (0x1U << RCC_CIR_PLLRDYIE_Pos)    /*!< 0x00001000 */
 #define RCC_CIR_PLLRDYIE                     RCC_CIR_PLLRDYIE_Msk              /*!< PLL Ready Interrupt Enable */
 #define RCC_CIR_LSIRDYC_Pos                  (16U)
 #define RCC_CIR_LSIRDYC_Msk                  (0x1U << RCC_CIR_LSIRDYC_Pos)     /*!< 0x00010000 */
 #define RCC_CIR_LSIRDYC                      RCC_CIR_LSIRDYC_Msk               /*!< LSI Ready Interrupt Clear */
 #define RCC_CIR_LSERDYC_Pos                  (17U)
 #define RCC_CIR_LSERDYC_Msk                  (0x1U << RCC_CIR_LSERDYC_Pos)     /*!< 0x00020000 */
 #define RCC_CIR_LSERDYC                      RCC_CIR_LSERDYC_Msk               /*!< LSE Ready Interrupt Clear */
 #define RCC_CIR_HSIRDYC_Pos                  (18U)
 #define RCC_CIR_HSIRDYC_Msk                  (0x1U << RCC_CIR_HSIRDYC_Pos)     /*!< 0x00040000 */
 #define RCC_CIR_HSIRDYC                      RCC_CIR_HSIRDYC_Msk               /*!< HSI Ready Interrupt Clear */
 #define RCC_CIR_HSERDYC_Pos                  (19U)
 #define RCC_CIR_HSERDYC_Msk                  (0x1U << RCC_CIR_HSERDYC_Pos)     /*!< 0x00080000 */
 #define RCC_CIR_HSERDYC                      RCC_CIR_HSERDYC_Msk               /*!< HSE Ready Interrupt Clear */
 #define RCC_CIR_PLLRDYC_Pos                  (20U)
 #define RCC_CIR_PLLRDYC_Msk                  (0x1U << RCC_CIR_PLLRDYC_Pos)     /*!< 0x00100000 */
 #define RCC_CIR_PLLRDYC                      RCC_CIR_PLLRDYC_Msk               /*!< PLL Ready Interrupt Clear */
 #define RCC_CIR_CSSC_Pos                     (23U)
 #define RCC_CIR_CSSC_Msk                     (0x1U << RCC_CIR_CSSC_Pos)        /*!< 0x00800000 */
 #define RCC_CIR_CSSC                         RCC_CIR_CSSC_Msk                  /*!< Clock Security System Interrupt Clear */

 #define RCC_CIR_PLL2RDYF_Pos                 (5U)
 #define RCC_CIR_PLL2RDYF_Msk                 (0x1U << RCC_CIR_PLL2RDYF_Pos)    /*!< 0x00000020 */
 #define RCC_CIR_PLL2RDYF                     RCC_CIR_PLL2RDYF_Msk              /*!< PLL2 Ready Interrupt flag */
 #define RCC_CIR_PLL3RDYF_Pos                 (6U)
 #define RCC_CIR_PLL3RDYF_Msk                 (0x1U << RCC_CIR_PLL3RDYF_Pos)    /*!< 0x00000040 */
 #define RCC_CIR_PLL3RDYF                     RCC_CIR_PLL3RDYF_Msk              /*!< PLL3 Ready Interrupt flag */
 #define RCC_CIR_PLL2RDYIE_Pos                (13U)
 #define RCC_CIR_PLL2RDYIE_Msk                (0x1U << RCC_CIR_PLL2RDYIE_Pos)   /*!< 0x00002000 */
 #define RCC_CIR_PLL2RDYIE                    RCC_CIR_PLL2RDYIE_Msk             /*!< PLL2 Ready Interrupt Enable */
 #define RCC_CIR_PLL3RDYIE_Pos                (14U)
 #define RCC_CIR_PLL3RDYIE_Msk                (0x1U << RCC_CIR_PLL3RDYIE_Pos)   /*!< 0x00004000 */
 #define RCC_CIR_PLL3RDYIE                    RCC_CIR_PLL3RDYIE_Msk             /*!< PLL3 Ready Interrupt Enable */
 #define RCC_CIR_PLL2RDYC_Pos                 (21U)
 #define RCC_CIR_PLL2RDYC_Msk                 (0x1U << RCC_CIR_PLL2RDYC_Pos)    /*!< 0x00200000 */
 #define RCC_CIR_PLL2RDYC                     RCC_CIR_PLL2RDYC_Msk              /*!< PLL2 Ready Interrupt Clear */
 #define RCC_CIR_PLL3RDYC_Pos                 (22U)
 #define RCC_CIR_PLL3RDYC_Msk                 (0x1U << RCC_CIR_PLL3RDYC_Pos)    /*!< 0x00400000 */
 #define RCC_CIR_PLL3RDYC                     RCC_CIR_PLL3RDYC_Msk              /*!< PLL3 Ready Interrupt Clear */

 /*****************  Bit definition for RCC_APB2RSTR register  *****************/
 #define RCC_APB2RSTR_AFIORST_Pos             (0U)
 #define RCC_APB2RSTR_AFIORST_Msk             (0x1U << RCC_APB2RSTR_AFIORST_Pos) /*!< 0x00000001 */
 #define RCC_APB2RSTR_AFIORST                 RCC_APB2RSTR_AFIORST_Msk          /*!< Alternate Function I/O reset */
 #define RCC_APB2RSTR_IOPARST_Pos             (2U)
 #define RCC_APB2RSTR_IOPARST_Msk             (0x1U << RCC_APB2RSTR_IOPARST_Pos) /*!< 0x00000004 */
 #define RCC_APB2RSTR_IOPARST                 RCC_APB2RSTR_IOPARST_Msk          /*!< I/O port A reset */
 #define RCC_APB2RSTR_IOPBRST_Pos             (3U)
 #define RCC_APB2RSTR_IOPBRST_Msk             (0x1U << RCC_APB2RSTR_IOPBRST_Pos) /*!< 0x00000008 */
 #define RCC_APB2RSTR_IOPBRST                 RCC_APB2RSTR_IOPBRST_Msk          /*!< I/O port B reset */
 #define RCC_APB2RSTR_IOPCRST_Pos             (4U)
 #define RCC_APB2RSTR_IOPCRST_Msk             (0x1U << RCC_APB2RSTR_IOPCRST_Pos) /*!< 0x00000010 */
 #define RCC_APB2RSTR_IOPCRST                 RCC_APB2RSTR_IOPCRST_Msk          /*!< I/O port C reset */
 #define RCC_APB2RSTR_IOPDRST_Pos             (5U)
 #define RCC_APB2RSTR_IOPDRST_Msk             (0x1U << RCC_APB2RSTR_IOPDRST_Pos) /*!< 0x00000020 */
 #define RCC_APB2RSTR_IOPDRST                 RCC_APB2RSTR_IOPDRST_Msk          /*!< I/O port D reset */
 #define RCC_APB2RSTR_ADC1RST_Pos             (9U)
 #define RCC_APB2RSTR_ADC1RST_Msk             (0x1U << RCC_APB2RSTR_ADC1RST_Pos) /*!< 0x00000200 */
 #define RCC_APB2RSTR_ADC1RST                 RCC_APB2RSTR_ADC1RST_Msk          /*!< ADC 1 interface reset */

 #define RCC_APB2RSTR_ADC2RST_Pos             (10U)
 #define RCC_APB2RSTR_ADC2RST_Msk             (0x1U << RCC_APB2RSTR_ADC2RST_Pos) /*!< 0x00000400 */
 #define RCC_APB2RSTR_ADC2RST                 RCC_APB2RSTR_ADC2RST_Msk          /*!< ADC 2 interface reset */

 #define RCC_APB2RSTR_TIM1RST_Pos             (11U)
 #define RCC_APB2RSTR_TIM1RST_Msk             (0x1U << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
 #define RCC_APB2RSTR_TIM1RST                 RCC_APB2RSTR_TIM1RST_Msk          /*!< TIM1 Timer reset */
 #define RCC_APB2RSTR_SPI1RST_Pos             (12U)
 #define RCC_APB2RSTR_SPI1RST_Msk             (0x1U << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
 #define RCC_APB2RSTR_SPI1RST                 RCC_APB2RSTR_SPI1RST_Msk          /*!< SPI 1 reset */
 #define RCC_APB2RSTR_USART1RST_Pos           (14U)
 #define RCC_APB2RSTR_USART1RST_Msk           (0x1U << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
 #define RCC_APB2RSTR_USART1RST               RCC_APB2RSTR_USART1RST_Msk        /*!< USART1 reset */


 #define RCC_APB2RSTR_IOPERST_Pos             (6U)
 #define RCC_APB2RSTR_IOPERST_Msk             (0x1U << RCC_APB2RSTR_IOPERST_Pos) /*!< 0x00000040 */
 #define RCC_APB2RSTR_IOPERST                 RCC_APB2RSTR_IOPERST_Msk          /*!< I/O port E reset */




 /*****************  Bit definition for RCC_APB1RSTR register  *****************/
 #define RCC_APB1RSTR_TIM2RST_Pos             (0U)
 #define RCC_APB1RSTR_TIM2RST_Msk             (0x1U << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
 #define RCC_APB1RSTR_TIM2RST                 RCC_APB1RSTR_TIM2RST_Msk          /*!< Timer 2 reset */
 #define RCC_APB1RSTR_TIM3RST_Pos             (1U)
 #define RCC_APB1RSTR_TIM3RST_Msk             (0x1U << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
 #define RCC_APB1RSTR_TIM3RST                 RCC_APB1RSTR_TIM3RST_Msk          /*!< Timer 3 reset */
 #define RCC_APB1RSTR_WWDGRST_Pos             (11U)
 #define RCC_APB1RSTR_WWDGRST_Msk             (0x1U << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
 #define RCC_APB1RSTR_WWDGRST                 RCC_APB1RSTR_WWDGRST_Msk          /*!< Window Watchdog reset */
 #define RCC_APB1RSTR_USART2RST_Pos           (17U)
 #define RCC_APB1RSTR_USART2RST_Msk           (0x1U << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
 #define RCC_APB1RSTR_USART2RST               RCC_APB1RSTR_USART2RST_Msk        /*!< USART 2 reset */
 #define RCC_APB1RSTR_I2C1RST_Pos             (21U)
 #define RCC_APB1RSTR_I2C1RST_Msk             (0x1U << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
 #define RCC_APB1RSTR_I2C1RST                 RCC_APB1RSTR_I2C1RST_Msk          /*!< I2C 1 reset */

 #define RCC_APB1RSTR_CAN1RST_Pos             (25U)
 #define RCC_APB1RSTR_CAN1RST_Msk             (0x1U << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
 #define RCC_APB1RSTR_CAN1RST                 RCC_APB1RSTR_CAN1RST_Msk          /*!< CAN1 reset */

 #define RCC_APB1RSTR_BKPRST_Pos              (27U)
 #define RCC_APB1RSTR_BKPRST_Msk              (0x1U << RCC_APB1RSTR_BKPRST_Pos) /*!< 0x08000000 */
 #define RCC_APB1RSTR_BKPRST                  RCC_APB1RSTR_BKPRST_Msk           /*!< Backup interface reset */
 #define RCC_APB1RSTR_PWRRST_Pos              (28U)
 #define RCC_APB1RSTR_PWRRST_Msk              (0x1U << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
 #define RCC_APB1RSTR_PWRRST                  RCC_APB1RSTR_PWRRST_Msk           /*!< Power interface reset */

 #define RCC_APB1RSTR_TIM4RST_Pos             (2U)
 #define RCC_APB1RSTR_TIM4RST_Msk             (0x1U << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
 #define RCC_APB1RSTR_TIM4RST                 RCC_APB1RSTR_TIM4RST_Msk          /*!< Timer 4 reset */
 #define RCC_APB1RSTR_SPI2RST_Pos             (14U)
 #define RCC_APB1RSTR_SPI2RST_Msk             (0x1U << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
 #define RCC_APB1RSTR_SPI2RST                 RCC_APB1RSTR_SPI2RST_Msk          /*!< SPI 2 reset */
 #define RCC_APB1RSTR_USART3RST_Pos           (18U)
 #define RCC_APB1RSTR_USART3RST_Msk           (0x1U << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
 #define RCC_APB1RSTR_USART3RST               RCC_APB1RSTR_USART3RST_Msk        /*!< USART 3 reset */
 #define RCC_APB1RSTR_I2C2RST_Pos             (22U)
 #define RCC_APB1RSTR_I2C2RST_Msk             (0x1U << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
 #define RCC_APB1RSTR_I2C2RST                 RCC_APB1RSTR_I2C2RST_Msk          /*!< I2C 2 reset */


 #define RCC_APB1RSTR_TIM5RST_Pos             (3U)
 #define RCC_APB1RSTR_TIM5RST_Msk             (0x1U << RCC_APB1RSTR_TIM5RST_Pos) /*!< 0x00000008 */
 #define RCC_APB1RSTR_TIM5RST                 RCC_APB1RSTR_TIM5RST_Msk          /*!< Timer 5 reset */
 #define RCC_APB1RSTR_TIM6RST_Pos             (4U)
 #define RCC_APB1RSTR_TIM6RST_Msk             (0x1U << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
 #define RCC_APB1RSTR_TIM6RST                 RCC_APB1RSTR_TIM6RST_Msk          /*!< Timer 6 reset */
 #define RCC_APB1RSTR_TIM7RST_Pos             (5U)
 #define RCC_APB1RSTR_TIM7RST_Msk             (0x1U << RCC_APB1RSTR_TIM7RST_Pos) /*!< 0x00000020 */
 #define RCC_APB1RSTR_TIM7RST                 RCC_APB1RSTR_TIM7RST_Msk          /*!< Timer 7 reset */
 #define RCC_APB1RSTR_SPI3RST_Pos             (15U)
 #define RCC_APB1RSTR_SPI3RST_Msk             (0x1U << RCC_APB1RSTR_SPI3RST_Pos) /*!< 0x00008000 */
 #define RCC_APB1RSTR_SPI3RST                 RCC_APB1RSTR_SPI3RST_Msk          /*!< SPI 3 reset */
 #define RCC_APB1RSTR_UART4RST_Pos            (19U)
 #define RCC_APB1RSTR_UART4RST_Msk            (0x1U << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
 #define RCC_APB1RSTR_UART4RST                RCC_APB1RSTR_UART4RST_Msk         /*!< UART 4 reset */
 #define RCC_APB1RSTR_UART5RST_Pos            (20U)
 #define RCC_APB1RSTR_UART5RST_Msk            (0x1U << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
 #define RCC_APB1RSTR_UART5RST                RCC_APB1RSTR_UART5RST_Msk         /*!< UART 5 reset */



 #define RCC_APB1RSTR_CAN2RST_Pos             (26U)
 #define RCC_APB1RSTR_CAN2RST_Msk             (0x1U << RCC_APB1RSTR_CAN2RST_Pos) /*!< 0x04000000 */
 #define RCC_APB1RSTR_CAN2RST                 RCC_APB1RSTR_CAN2RST_Msk          /*!< CAN2 reset */

 #define RCC_APB1RSTR_DACRST_Pos              (29U)
 #define RCC_APB1RSTR_DACRST_Msk              (0x1U << RCC_APB1RSTR_DACRST_Pos) /*!< 0x20000000 */
 #define RCC_APB1RSTR_DACRST                  RCC_APB1RSTR_DACRST_Msk           /*!< DAC interface reset */

 /******************  Bit definition for RCC_AHBENR register  ******************/
 #define RCC_AHBENR_DMA1EN_Pos                (0U)
 #define RCC_AHBENR_DMA1EN_Msk                (0x1U << RCC_AHBENR_DMA1EN_Pos)   /*!< 0x00000001 */
 #define RCC_AHBENR_DMA1EN                    RCC_AHBENR_DMA1EN_Msk             /*!< DMA1 clock enable */
 #define RCC_AHBENR_SRAMEN_Pos                (2U)
 #define RCC_AHBENR_SRAMEN_Msk                (0x1U << RCC_AHBENR_SRAMEN_Pos)   /*!< 0x00000004 */
 #define RCC_AHBENR_SRAMEN                    RCC_AHBENR_SRAMEN_Msk             /*!< SRAM interface clock enable */
 #define RCC_AHBENR_FLITFEN_Pos               (4U)
 #define RCC_AHBENR_FLITFEN_Msk               (0x1U << RCC_AHBENR_FLITFEN_Pos)  /*!< 0x00000010 */
 #define RCC_AHBENR_FLITFEN                   RCC_AHBENR_FLITFEN_Msk            /*!< FLITF clock enable */
 #define RCC_AHBENR_CRCEN_Pos                 (6U)
 #define RCC_AHBENR_CRCEN_Msk                 (0x1U << RCC_AHBENR_CRCEN_Pos)    /*!< 0x00000040 */
 #define RCC_AHBENR_CRCEN                     RCC_AHBENR_CRCEN_Msk              /*!< CRC clock enable */

 #define RCC_AHBENR_DMA2EN_Pos                (1U)
 #define RCC_AHBENR_DMA2EN_Msk                (0x1U << RCC_AHBENR_DMA2EN_Pos)   /*!< 0x00000002 */
 #define RCC_AHBENR_DMA2EN                    RCC_AHBENR_DMA2EN_Msk             /*!< DMA2 clock enable */


 #define RCC_AHBENR_OTGFSEN_Pos               (12U)
 #define RCC_AHBENR_OTGFSEN_Msk               (0x1U << RCC_AHBENR_OTGFSEN_Pos)  /*!< 0x00001000 */
 #define RCC_AHBENR_OTGFSEN                   RCC_AHBENR_OTGFSEN_Msk            /*!< USB OTG FS clock enable */
 #define RCC_AHBENR_ETHMACEN_Pos              (14U)
 #define RCC_AHBENR_ETHMACEN_Msk              (0x1U << RCC_AHBENR_ETHMACEN_Pos) /*!< 0x00004000 */
 #define RCC_AHBENR_ETHMACEN                  RCC_AHBENR_ETHMACEN_Msk           /*!< ETHERNET MAC clock enable */
 #define RCC_AHBENR_ETHMACTXEN_Pos            (15U)
 #define RCC_AHBENR_ETHMACTXEN_Msk            (0x1U << RCC_AHBENR_ETHMACTXEN_Pos) /*!< 0x00008000 */
 #define RCC_AHBENR_ETHMACTXEN                RCC_AHBENR_ETHMACTXEN_Msk         /*!< ETHERNET MAC Tx clock enable */
 #define RCC_AHBENR_ETHMACRXEN_Pos            (16U)
 #define RCC_AHBENR_ETHMACRXEN_Msk            (0x1U << RCC_AHBENR_ETHMACRXEN_Pos) /*!< 0x00010000 */
 #define RCC_AHBENR_ETHMACRXEN                RCC_AHBENR_ETHMACRXEN_Msk         /*!< ETHERNET MAC Rx clock enable */

 /******************  Bit definition for RCC_APB2ENR register  *****************/
 #define RCC_APB2ENR_AFIOEN_Pos               (0U)
 #define RCC_APB2ENR_AFIOEN_Msk               (0x1U << RCC_APB2ENR_AFIOEN_Pos)  /*!< 0x00000001 */
 #define RCC_APB2ENR_AFIOEN                   RCC_APB2ENR_AFIOEN_Msk            /*!< Alternate Function I/O clock enable */
 #define RCC_APB2ENR_IOPAEN_Pos               (2U)
 #define RCC_APB2ENR_IOPAEN_Msk               (0x1U << RCC_APB2ENR_IOPAEN_Pos)  /*!< 0x00000004 */
 #define RCC_APB2ENR_IOPAEN                   RCC_APB2ENR_IOPAEN_Msk            /*!< I/O port A clock enable */
 #define RCC_APB2ENR_IOPBEN_Pos               (3U)
 #define RCC_APB2ENR_IOPBEN_Msk               (0x1U << RCC_APB2ENR_IOPBEN_Pos)  /*!< 0x00000008 */
 #define RCC_APB2ENR_IOPBEN                   RCC_APB2ENR_IOPBEN_Msk            /*!< I/O port B clock enable */
 #define RCC_APB2ENR_IOPCEN_Pos               (4U)
 #define RCC_APB2ENR_IOPCEN_Msk               (0x1U << RCC_APB2ENR_IOPCEN_Pos)  /*!< 0x00000010 */
 #define RCC_APB2ENR_IOPCEN                   RCC_APB2ENR_IOPCEN_Msk            /*!< I/O port C clock enable */
 #define RCC_APB2ENR_IOPDEN_Pos               (5U)
 #define RCC_APB2ENR_IOPDEN_Msk               (0x1U << RCC_APB2ENR_IOPDEN_Pos)  /*!< 0x00000020 */
 #define RCC_APB2ENR_IOPDEN                   RCC_APB2ENR_IOPDEN_Msk            /*!< I/O port D clock enable */
 #define RCC_APB2ENR_ADC1EN_Pos               (9U)
 #define RCC_APB2ENR_ADC1EN_Msk               (0x1U << RCC_APB2ENR_ADC1EN_Pos)  /*!< 0x00000200 */
 #define RCC_APB2ENR_ADC1EN                   RCC_APB2ENR_ADC1EN_Msk            /*!< ADC 1 interface clock enable */

 #define RCC_APB2ENR_ADC2EN_Pos               (10U)
 #define RCC_APB2ENR_ADC2EN_Msk               (0x1U << RCC_APB2ENR_ADC2EN_Pos)  /*!< 0x00000400 */
 #define RCC_APB2ENR_ADC2EN                   RCC_APB2ENR_ADC2EN_Msk            /*!< ADC 2 interface clock enable */

 #define RCC_APB2ENR_TIM1EN_Pos               (11U)
 #define RCC_APB2ENR_TIM1EN_Msk               (0x1U << RCC_APB2ENR_TIM1EN_Pos)  /*!< 0x00000800 */
 #define RCC_APB2ENR_TIM1EN                   RCC_APB2ENR_TIM1EN_Msk            /*!< TIM1 Timer clock enable */
 #define RCC_APB2ENR_SPI1EN_Pos               (12U)
 #define RCC_APB2ENR_SPI1EN_Msk               (0x1U << RCC_APB2ENR_SPI1EN_Pos)  /*!< 0x00001000 */
 #define RCC_APB2ENR_SPI1EN                   RCC_APB2ENR_SPI1EN_Msk            /*!< SPI 1 clock enable */
 #define RCC_APB2ENR_USART1EN_Pos             (14U)
 #define RCC_APB2ENR_USART1EN_Msk             (0x1U << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
 #define RCC_APB2ENR_USART1EN                 RCC_APB2ENR_USART1EN_Msk          /*!< USART1 clock enable */


 #define RCC_APB2ENR_IOPEEN_Pos               (6U)
 #define RCC_APB2ENR_IOPEEN_Msk               (0x1U << RCC_APB2ENR_IOPEEN_Pos)  /*!< 0x00000040 */
 #define RCC_APB2ENR_IOPEEN                   RCC_APB2ENR_IOPEEN_Msk            /*!< I/O port E clock enable */




 /*****************  Bit definition for RCC_APB1ENR register  ******************/
 #define RCC_APB1ENR_TIM2EN_Pos               (0U)
 #define RCC_APB1ENR_TIM2EN_Msk               (0x1U << RCC_APB1ENR_TIM2EN_Pos)  /*!< 0x00000001 */
 #define RCC_APB1ENR_TIM2EN                   RCC_APB1ENR_TIM2EN_Msk            /*!< Timer 2 clock enabled*/
 #define RCC_APB1ENR_TIM3EN_Pos               (1U)
 #define RCC_APB1ENR_TIM3EN_Msk               (0x1U << RCC_APB1ENR_TIM3EN_Pos)  /*!< 0x00000002 */
 #define RCC_APB1ENR_TIM3EN                   RCC_APB1ENR_TIM3EN_Msk            /*!< Timer 3 clock enable */
 #define RCC_APB1ENR_WWDGEN_Pos               (11U)
 #define RCC_APB1ENR_WWDGEN_Msk               (0x1U << RCC_APB1ENR_WWDGEN_Pos)  /*!< 0x00000800 */
 #define RCC_APB1ENR_WWDGEN                   RCC_APB1ENR_WWDGEN_Msk            /*!< Window Watchdog clock enable */
 #define RCC_APB1ENR_USART2EN_Pos             (17U)
 #define RCC_APB1ENR_USART2EN_Msk             (0x1U << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
 #define RCC_APB1ENR_USART2EN                 RCC_APB1ENR_USART2EN_Msk          /*!< USART 2 clock enable */
 #define RCC_APB1ENR_I2C1EN_Pos               (21U)
 #define RCC_APB1ENR_I2C1EN_Msk               (0x1U << RCC_APB1ENR_I2C1EN_Pos)  /*!< 0x00200000 */
 #define RCC_APB1ENR_I2C1EN                   RCC_APB1ENR_I2C1EN_Msk            /*!< I2C 1 clock enable */

 #define RCC_APB1ENR_CAN1EN_Pos               (25U)
 #define RCC_APB1ENR_CAN1EN_Msk               (0x1U << RCC_APB1ENR_CAN1EN_Pos)  /*!< 0x02000000 */
 #define RCC_APB1ENR_CAN1EN                   RCC_APB1ENR_CAN1EN_Msk            /*!< CAN1 clock enable */

 #define RCC_APB1ENR_BKPEN_Pos                (27U)
 #define RCC_APB1ENR_BKPEN_Msk                (0x1U << RCC_APB1ENR_BKPEN_Pos)   /*!< 0x08000000 */
 #define RCC_APB1ENR_BKPEN                    RCC_APB1ENR_BKPEN_Msk             /*!< Backup interface clock enable */
 #define RCC_APB1ENR_PWREN_Pos                (28U)
 #define RCC_APB1ENR_PWREN_Msk                (0x1U << RCC_APB1ENR_PWREN_Pos)   /*!< 0x10000000 */
 #define RCC_APB1ENR_PWREN                    RCC_APB1ENR_PWREN_Msk             /*!< Power interface clock enable */

 #define RCC_APB1ENR_TIM4EN_Pos               (2U)
 #define RCC_APB1ENR_TIM4EN_Msk               (0x1U << RCC_APB1ENR_TIM4EN_Pos)  /*!< 0x00000004 */
 #define RCC_APB1ENR_TIM4EN                   RCC_APB1ENR_TIM4EN_Msk            /*!< Timer 4 clock enable */
 #define RCC_APB1ENR_SPI2EN_Pos               (14U)
 #define RCC_APB1ENR_SPI2EN_Msk               (0x1U << RCC_APB1ENR_SPI2EN_Pos)  /*!< 0x00004000 */
 #define RCC_APB1ENR_SPI2EN                   RCC_APB1ENR_SPI2EN_Msk            /*!< SPI 2 clock enable */
 #define RCC_APB1ENR_USART3EN_Pos             (18U)
 #define RCC_APB1ENR_USART3EN_Msk             (0x1U << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
 #define RCC_APB1ENR_USART3EN                 RCC_APB1ENR_USART3EN_Msk          /*!< USART 3 clock enable */
 #define RCC_APB1ENR_I2C2EN_Pos               (22U)
 #define RCC_APB1ENR_I2C2EN_Msk               (0x1U << RCC_APB1ENR_I2C2EN_Pos)  /*!< 0x00400000 */
 #define RCC_APB1ENR_I2C2EN                   RCC_APB1ENR_I2C2EN_Msk            /*!< I2C 2 clock enable */


 #define RCC_APB1ENR_TIM5EN_Pos               (3U)
 #define RCC_APB1ENR_TIM5EN_Msk               (0x1U << RCC_APB1ENR_TIM5EN_Pos)  /*!< 0x00000008 */
 #define RCC_APB1ENR_TIM5EN                   RCC_APB1ENR_TIM5EN_Msk            /*!< Timer 5 clock enable */
 #define RCC_APB1ENR_TIM6EN_Pos               (4U)
 #define RCC_APB1ENR_TIM6EN_Msk               (0x1U << RCC_APB1ENR_TIM6EN_Pos)  /*!< 0x00000010 */
 #define RCC_APB1ENR_TIM6EN                   RCC_APB1ENR_TIM6EN_Msk            /*!< Timer 6 clock enable */
 #define RCC_APB1ENR_TIM7EN_Pos               (5U)
 #define RCC_APB1ENR_TIM7EN_Msk               (0x1U << RCC_APB1ENR_TIM7EN_Pos)  /*!< 0x00000020 */
 #define RCC_APB1ENR_TIM7EN                   RCC_APB1ENR_TIM7EN_Msk            /*!< Timer 7 clock enable */
 #define RCC_APB1ENR_SPI3EN_Pos               (15U)
 #define RCC_APB1ENR_SPI3EN_Msk               (0x1U << RCC_APB1ENR_SPI3EN_Pos)  /*!< 0x00008000 */
 #define RCC_APB1ENR_SPI3EN                   RCC_APB1ENR_SPI3EN_Msk            /*!< SPI 3 clock enable */
 #define RCC_APB1ENR_UART4EN_Pos              (19U)
 #define RCC_APB1ENR_UART4EN_Msk              (0x1U << RCC_APB1ENR_UART4EN_Pos) /*!< 0x00080000 */
 #define RCC_APB1ENR_UART4EN                  RCC_APB1ENR_UART4EN_Msk           /*!< UART 4 clock enable */
 #define RCC_APB1ENR_UART5EN_Pos              (20U)
 #define RCC_APB1ENR_UART5EN_Msk              (0x1U << RCC_APB1ENR_UART5EN_Pos) /*!< 0x00100000 */
 #define RCC_APB1ENR_UART5EN                  RCC_APB1ENR_UART5EN_Msk           /*!< UART 5 clock enable */



 #define RCC_APB1ENR_CAN2EN_Pos               (26U)
 #define RCC_APB1ENR_CAN2EN_Msk               (0x1U << RCC_APB1ENR_CAN2EN_Pos)  /*!< 0x04000000 */
 #define RCC_APB1ENR_CAN2EN                   RCC_APB1ENR_CAN2EN_Msk            /*!< CAN2 clock enable */

 #define RCC_APB1ENR_DACEN_Pos                (29U)
 #define RCC_APB1ENR_DACEN_Msk                (0x1U << RCC_APB1ENR_DACEN_Pos)   /*!< 0x20000000 */
 #define RCC_APB1ENR_DACEN                    RCC_APB1ENR_DACEN_Msk             /*!< DAC interface clock enable */

 /*******************  Bit definition for RCC_BDCR register  *******************/
 #define RCC_BDCR_LSEON_Pos                   (0U)
 #define RCC_BDCR_LSEON_Msk                   (0x1U << RCC_BDCR_LSEON_Pos)      /*!< 0x00000001 */
 #define RCC_BDCR_LSEON                       RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
 #define RCC_BDCR_LSERDY_Pos                  (1U)
 #define RCC_BDCR_LSERDY_Msk                  (0x1U << RCC_BDCR_LSERDY_Pos)     /*!< 0x00000002 */
 #define RCC_BDCR_LSERDY                      RCC_BDCR_LSERDY_Msk               /*!< External Low Speed oscillator Ready */
 #define RCC_BDCR_LSEBYP_Pos                  (2U)
 #define RCC_BDCR_LSEBYP_Msk                  (0x1U << RCC_BDCR_LSEBYP_Pos)     /*!< 0x00000004 */
 #define RCC_BDCR_LSEBYP                      RCC_BDCR_LSEBYP_Msk               /*!< External Low Speed oscillator Bypass */

 #define RCC_BDCR_RTCSEL_Pos                  (8U)
 #define RCC_BDCR_RTCSEL_Msk                  (0x3U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000300 */
 #define RCC_BDCR_RTCSEL                      RCC_BDCR_RTCSEL_Msk               /*!< RTCSEL[1:0] bits (RTC clock source selection) */
 #define RCC_BDCR_RTCSEL_0                    (0x1U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000100 */
 #define RCC_BDCR_RTCSEL_1                    (0x2U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000200 */

 /*!< RTC congiguration */
 #define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000U                       /*!< No clock */
 #define RCC_BDCR_RTCSEL_LSE                  0x00000100U                       /*!< LSE oscillator clock used as RTC clock */
 #define RCC_BDCR_RTCSEL_LSI                  0x00000200U                       /*!< LSI oscillator clock used as RTC clock */
 #define RCC_BDCR_RTCSEL_HSE                  0x00000300U                       /*!< HSE oscillator clock divided by 128 used as RTC clock */

 #define RCC_BDCR_RTCEN_Pos                   (15U)
 #define RCC_BDCR_RTCEN_Msk                   (0x1U << RCC_BDCR_RTCEN_Pos)      /*!< 0x00008000 */
 #define RCC_BDCR_RTCEN                       RCC_BDCR_RTCEN_Msk                /*!< RTC clock enable */
 #define RCC_BDCR_BDRST_Pos                   (16U)
 #define RCC_BDCR_BDRST_Msk                   (0x1U << RCC_BDCR_BDRST_Pos)      /*!< 0x00010000 */
 #define RCC_BDCR_BDRST                       RCC_BDCR_BDRST_Msk                /*!< Backup domain software reset  */

 /*******************  Bit definition for RCC_CSR register  ********************/
 #define RCC_CSR_LSION_Pos                    (0U)
 #define RCC_CSR_LSION_Msk                    (0x1U << RCC_CSR_LSION_Pos)       /*!< 0x00000001 */
 #define RCC_CSR_LSION                        RCC_CSR_LSION_Msk                 /*!< Internal Low Speed oscillator enable */
 #define RCC_CSR_LSIRDY_Pos                   (1U)
 #define RCC_CSR_LSIRDY_Msk                   (0x1U << RCC_CSR_LSIRDY_Pos)      /*!< 0x00000002 */
 #define RCC_CSR_LSIRDY                       RCC_CSR_LSIRDY_Msk                /*!< Internal Low Speed oscillator Ready */
 #define RCC_CSR_RMVF_Pos                     (24U)
 #define RCC_CSR_RMVF_Msk                     (0x1U << RCC_CSR_RMVF_Pos)        /*!< 0x01000000 */
 #define RCC_CSR_RMVF                         RCC_CSR_RMVF_Msk                  /*!< Remove reset flag */
 #define RCC_CSR_PINRSTF_Pos                  (26U)
 #define RCC_CSR_PINRSTF_Msk                  (0x1U << RCC_CSR_PINRSTF_Pos)     /*!< 0x04000000 */
 #define RCC_CSR_PINRSTF                      RCC_CSR_PINRSTF_Msk               /*!< PIN reset flag */
 #define RCC_CSR_PORRSTF_Pos                  (27U)
 #define RCC_CSR_PORRSTF_Msk                  (0x1U << RCC_CSR_PORRSTF_Pos)     /*!< 0x08000000 */
 #define RCC_CSR_PORRSTF                      RCC_CSR_PORRSTF_Msk               /*!< POR/PDR reset flag */
 #define RCC_CSR_SFTRSTF_Pos                  (28U)
 #define RCC_CSR_SFTRSTF_Msk                  (0x1U << RCC_CSR_SFTRSTF_Pos)     /*!< 0x10000000 */
 #define RCC_CSR_SFTRSTF                      RCC_CSR_SFTRSTF_Msk               /*!< Software Reset flag */
 #define RCC_CSR_IWDGRSTF_Pos                 (29U)
 #define RCC_CSR_IWDGRSTF_Msk                 (0x1U << RCC_CSR_IWDGRSTF_Pos)    /*!< 0x20000000 */
 #define RCC_CSR_IWDGRSTF                     RCC_CSR_IWDGRSTF_Msk              /*!< Independent Watchdog reset flag */
 #define RCC_CSR_WWDGRSTF_Pos                 (30U)
 #define RCC_CSR_WWDGRSTF_Msk                 (0x1U << RCC_CSR_WWDGRSTF_Pos)    /*!< 0x40000000 */
 #define RCC_CSR_WWDGRSTF                     RCC_CSR_WWDGRSTF_Msk              /*!< Window watchdog reset flag */
 #define RCC_CSR_LPWRRSTF_Pos                 (31U)
 #define RCC_CSR_LPWRRSTF_Msk                 (0x1U << RCC_CSR_LPWRRSTF_Pos)    /*!< 0x80000000 */
 #define RCC_CSR_LPWRRSTF                     RCC_CSR_LPWRRSTF_Msk              /*!< Low-Power reset flag */

 /*******************  Bit definition for RCC_AHBRSTR register  ****************/
 #define RCC_AHBRSTR_OTGFSRST_Pos             (12U)
 #define RCC_AHBRSTR_OTGFSRST_Msk             (0x1U << RCC_AHBRSTR_OTGFSRST_Pos) /*!< 0x00001000 */
 #define RCC_AHBRSTR_OTGFSRST                 RCC_AHBRSTR_OTGFSRST_Msk          /*!< USB OTG FS reset */
 #define RCC_AHBRSTR_ETHMACRST_Pos            (14U)
 #define RCC_AHBRSTR_ETHMACRST_Msk            (0x1U << RCC_AHBRSTR_ETHMACRST_Pos) /*!< 0x00004000 */
 #define RCC_AHBRSTR_ETHMACRST                RCC_AHBRSTR_ETHMACRST_Msk         /*!< ETHERNET MAC reset */

 /*******************  Bit definition for RCC_CFGR2 register  ******************/
 /*!< PREDIV1 configuration */
 #define RCC_CFGR2_PREDIV1_Pos                (0U)
 #define RCC_CFGR2_PREDIV1_Msk                (0xFU << RCC_CFGR2_PREDIV1_Pos)   /*!< 0x0000000F */
 #define RCC_CFGR2_PREDIV1                    RCC_CFGR2_PREDIV1_Msk             /*!< PREDIV1[3:0] bits */
 #define RCC_CFGR2_PREDIV1_0                  (0x1U << RCC_CFGR2_PREDIV1_Pos)   /*!< 0x00000001 */
 #define RCC_CFGR2_PREDIV1_1                  (0x2U << RCC_CFGR2_PREDIV1_Pos)   /*!< 0x00000002 */
 #define RCC_CFGR2_PREDIV1_2                  (0x4U << RCC_CFGR2_PREDIV1_Pos)   /*!< 0x00000004 */
 #define RCC_CFGR2_PREDIV1_3                  (0x8U << RCC_CFGR2_PREDIV1_Pos)   /*!< 0x00000008 */

 #define RCC_CFGR2_PREDIV1_DIV1               0x00000000U                       /*!< PREDIV1 input clock not divided */
 #define RCC_CFGR2_PREDIV1_DIV2_Pos           (0U)
 #define RCC_CFGR2_PREDIV1_DIV2_Msk           (0x1U << RCC_CFGR2_PREDIV1_DIV2_Pos) /*!< 0x00000001 */
 #define RCC_CFGR2_PREDIV1_DIV2               RCC_CFGR2_PREDIV1_DIV2_Msk        /*!< PREDIV1 input clock divided by 2 */
 #define RCC_CFGR2_PREDIV1_DIV3_Pos           (1U)
 #define RCC_CFGR2_PREDIV1_DIV3_Msk           (0x1U << RCC_CFGR2_PREDIV1_DIV3_Pos) /*!< 0x00000002 */
 #define RCC_CFGR2_PREDIV1_DIV3               RCC_CFGR2_PREDIV1_DIV3_Msk        /*!< PREDIV1 input clock divided by 3 */
 #define RCC_CFGR2_PREDIV1_DIV4_Pos           (0U)
 #define RCC_CFGR2_PREDIV1_DIV4_Msk           (0x3U << RCC_CFGR2_PREDIV1_DIV4_Pos) /*!< 0x00000003 */
 #define RCC_CFGR2_PREDIV1_DIV4               RCC_CFGR2_PREDIV1_DIV4_Msk        /*!< PREDIV1 input clock divided by 4 */
 #define RCC_CFGR2_PREDIV1_DIV5_Pos           (2U)
 #define RCC_CFGR2_PREDIV1_DIV5_Msk           (0x1U << RCC_CFGR2_PREDIV1_DIV5_Pos) /*!< 0x00000004 */
 #define RCC_CFGR2_PREDIV1_DIV5               RCC_CFGR2_PREDIV1_DIV5_Msk        /*!< PREDIV1 input clock divided by 5 */
 #define RCC_CFGR2_PREDIV1_DIV6_Pos           (0U)
 #define RCC_CFGR2_PREDIV1_DIV6_Msk           (0x5U << RCC_CFGR2_PREDIV1_DIV6_Pos) /*!< 0x00000005 */
 #define RCC_CFGR2_PREDIV1_DIV6               RCC_CFGR2_PREDIV1_DIV6_Msk        /*!< PREDIV1 input clock divided by 6 */
 #define RCC_CFGR2_PREDIV1_DIV7_Pos           (1U)
 #define RCC_CFGR2_PREDIV1_DIV7_Msk           (0x3U << RCC_CFGR2_PREDIV1_DIV7_Pos) /*!< 0x00000006 */
 #define RCC_CFGR2_PREDIV1_DIV7               RCC_CFGR2_PREDIV1_DIV7_Msk        /*!< PREDIV1 input clock divided by 7 */
 #define RCC_CFGR2_PREDIV1_DIV8_Pos           (0U)
 #define RCC_CFGR2_PREDIV1_DIV8_Msk           (0x7U << RCC_CFGR2_PREDIV1_DIV8_Pos) /*!< 0x00000007 */
 #define RCC_CFGR2_PREDIV1_DIV8               RCC_CFGR2_PREDIV1_DIV8_Msk        /*!< PREDIV1 input clock divided by 8 */
 #define RCC_CFGR2_PREDIV1_DIV9_Pos           (3U)
 #define RCC_CFGR2_PREDIV1_DIV9_Msk           (0x1U << RCC_CFGR2_PREDIV1_DIV9_Pos) /*!< 0x00000008 */
 #define RCC_CFGR2_PREDIV1_DIV9               RCC_CFGR2_PREDIV1_DIV9_Msk        /*!< PREDIV1 input clock divided by 9 */
 #define RCC_CFGR2_PREDIV1_DIV10_Pos          (0U)
 #define RCC_CFGR2_PREDIV1_DIV10_Msk          (0x9U << RCC_CFGR2_PREDIV1_DIV10_Pos) /*!< 0x00000009 */
 #define RCC_CFGR2_PREDIV1_DIV10              RCC_CFGR2_PREDIV1_DIV10_Msk       /*!< PREDIV1 input clock divided by 10 */
 #define RCC_CFGR2_PREDIV1_DIV11_Pos          (1U)
 #define RCC_CFGR2_PREDIV1_DIV11_Msk          (0x5U << RCC_CFGR2_PREDIV1_DIV11_Pos) /*!< 0x0000000A */
 #define RCC_CFGR2_PREDIV1_DIV11              RCC_CFGR2_PREDIV1_DIV11_Msk       /*!< PREDIV1 input clock divided by 11 */
 #define RCC_CFGR2_PREDIV1_DIV12_Pos          (0U)
 #define RCC_CFGR2_PREDIV1_DIV12_Msk          (0xBU << RCC_CFGR2_PREDIV1_DIV12_Pos) /*!< 0x0000000B */
 #define RCC_CFGR2_PREDIV1_DIV12              RCC_CFGR2_PREDIV1_DIV12_Msk       /*!< PREDIV1 input clock divided by 12 */
 #define RCC_CFGR2_PREDIV1_DIV13_Pos          (2U)
 #define RCC_CFGR2_PREDIV1_DIV13_Msk          (0x3U << RCC_CFGR2_PREDIV1_DIV13_Pos) /*!< 0x0000000C */
 #define RCC_CFGR2_PREDIV1_DIV13              RCC_CFGR2_PREDIV1_DIV13_Msk       /*!< PREDIV1 input clock divided by 13 */
 #define RCC_CFGR2_PREDIV1_DIV14_Pos          (0U)
 #define RCC_CFGR2_PREDIV1_DIV14_Msk          (0xDU << RCC_CFGR2_PREDIV1_DIV14_Pos) /*!< 0x0000000D */
 #define RCC_CFGR2_PREDIV1_DIV14              RCC_CFGR2_PREDIV1_DIV14_Msk       /*!< PREDIV1 input clock divided by 14 */
 #define RCC_CFGR2_PREDIV1_DIV15_Pos          (1U)
 #define RCC_CFGR2_PREDIV1_DIV15_Msk          (0x7U << RCC_CFGR2_PREDIV1_DIV15_Pos) /*!< 0x0000000E */
 #define RCC_CFGR2_PREDIV1_DIV15              RCC_CFGR2_PREDIV1_DIV15_Msk       /*!< PREDIV1 input clock divided by 15 */
 #define RCC_CFGR2_PREDIV1_DIV16_Pos          (0U)
 #define RCC_CFGR2_PREDIV1_DIV16_Msk          (0xFU << RCC_CFGR2_PREDIV1_DIV16_Pos) /*!< 0x0000000F */
 #define RCC_CFGR2_PREDIV1_DIV16              RCC_CFGR2_PREDIV1_DIV16_Msk       /*!< PREDIV1 input clock divided by 16 */

 /*!< PREDIV2 configuration */
 #define RCC_CFGR2_PREDIV2_Pos                (4U)
 #define RCC_CFGR2_PREDIV2_Msk                (0xFU << RCC_CFGR2_PREDIV2_Pos)   /*!< 0x000000F0 */
 #define RCC_CFGR2_PREDIV2                    RCC_CFGR2_PREDIV2_Msk             /*!< PREDIV2[3:0] bits */
 #define RCC_CFGR2_PREDIV2_0                  (0x1U << RCC_CFGR2_PREDIV2_Pos)   /*!< 0x00000010 */
 #define RCC_CFGR2_PREDIV2_1                  (0x2U << RCC_CFGR2_PREDIV2_Pos)   /*!< 0x00000020 */
 #define RCC_CFGR2_PREDIV2_2                  (0x4U << RCC_CFGR2_PREDIV2_Pos)   /*!< 0x00000040 */
 #define RCC_CFGR2_PREDIV2_3                  (0x8U << RCC_CFGR2_PREDIV2_Pos)   /*!< 0x00000080 */

 #define RCC_CFGR2_PREDIV2_DIV1               0x00000000U                       /*!< PREDIV2 input clock not divided */
 #define RCC_CFGR2_PREDIV2_DIV2_Pos           (4U)
 #define RCC_CFGR2_PREDIV2_DIV2_Msk           (0x1U << RCC_CFGR2_PREDIV2_DIV2_Pos) /*!< 0x00000010 */
 #define RCC_CFGR2_PREDIV2_DIV2               RCC_CFGR2_PREDIV2_DIV2_Msk        /*!< PREDIV2 input clock divided by 2 */
 #define RCC_CFGR2_PREDIV2_DIV3_Pos           (5U)
 #define RCC_CFGR2_PREDIV2_DIV3_Msk           (0x1U << RCC_CFGR2_PREDIV2_DIV3_Pos) /*!< 0x00000020 */
 #define RCC_CFGR2_PREDIV2_DIV3               RCC_CFGR2_PREDIV2_DIV3_Msk        /*!< PREDIV2 input clock divided by 3 */
 #define RCC_CFGR2_PREDIV2_DIV4_Pos           (4U)
 #define RCC_CFGR2_PREDIV2_DIV4_Msk           (0x3U << RCC_CFGR2_PREDIV2_DIV4_Pos) /*!< 0x00000030 */
 #define RCC_CFGR2_PREDIV2_DIV4               RCC_CFGR2_PREDIV2_DIV4_Msk        /*!< PREDIV2 input clock divided by 4 */
 #define RCC_CFGR2_PREDIV2_DIV5_Pos           (6U)
 #define RCC_CFGR2_PREDIV2_DIV5_Msk           (0x1U << RCC_CFGR2_PREDIV2_DIV5_Pos) /*!< 0x00000040 */
 #define RCC_CFGR2_PREDIV2_DIV5               RCC_CFGR2_PREDIV2_DIV5_Msk        /*!< PREDIV2 input clock divided by 5 */
 #define RCC_CFGR2_PREDIV2_DIV6_Pos           (4U)
 #define RCC_CFGR2_PREDIV2_DIV6_Msk           (0x5U << RCC_CFGR2_PREDIV2_DIV6_Pos) /*!< 0x00000050 */
 #define RCC_CFGR2_PREDIV2_DIV6               RCC_CFGR2_PREDIV2_DIV6_Msk        /*!< PREDIV2 input clock divided by 6 */
 #define RCC_CFGR2_PREDIV2_DIV7_Pos           (5U)
 #define RCC_CFGR2_PREDIV2_DIV7_Msk           (0x3U << RCC_CFGR2_PREDIV2_DIV7_Pos) /*!< 0x00000060 */
 #define RCC_CFGR2_PREDIV2_DIV7               RCC_CFGR2_PREDIV2_DIV7_Msk        /*!< PREDIV2 input clock divided by 7 */
 #define RCC_CFGR2_PREDIV2_DIV8_Pos           (4U)
 #define RCC_CFGR2_PREDIV2_DIV8_Msk           (0x7U << RCC_CFGR2_PREDIV2_DIV8_Pos) /*!< 0x00000070 */
 #define RCC_CFGR2_PREDIV2_DIV8               RCC_CFGR2_PREDIV2_DIV8_Msk        /*!< PREDIV2 input clock divided by 8 */
 #define RCC_CFGR2_PREDIV2_DIV9_Pos           (7U)
 #define RCC_CFGR2_PREDIV2_DIV9_Msk           (0x1U << RCC_CFGR2_PREDIV2_DIV9_Pos) /*!< 0x00000080 */
 #define RCC_CFGR2_PREDIV2_DIV9               RCC_CFGR2_PREDIV2_DIV9_Msk        /*!< PREDIV2 input clock divided by 9 */
 #define RCC_CFGR2_PREDIV2_DIV10_Pos          (4U)
 #define RCC_CFGR2_PREDIV2_DIV10_Msk          (0x9U << RCC_CFGR2_PREDIV2_DIV10_Pos) /*!< 0x00000090 */
 #define RCC_CFGR2_PREDIV2_DIV10              RCC_CFGR2_PREDIV2_DIV10_Msk       /*!< PREDIV2 input clock divided by 10 */
 #define RCC_CFGR2_PREDIV2_DIV11_Pos          (5U)
 #define RCC_CFGR2_PREDIV2_DIV11_Msk          (0x5U << RCC_CFGR2_PREDIV2_DIV11_Pos) /*!< 0x000000A0 */
 #define RCC_CFGR2_PREDIV2_DIV11              RCC_CFGR2_PREDIV2_DIV11_Msk       /*!< PREDIV2 input clock divided by 11 */
 #define RCC_CFGR2_PREDIV2_DIV12_Pos          (4U)
 #define RCC_CFGR2_PREDIV2_DIV12_Msk          (0xBU << RCC_CFGR2_PREDIV2_DIV12_Pos) /*!< 0x000000B0 */
 #define RCC_CFGR2_PREDIV2_DIV12              RCC_CFGR2_PREDIV2_DIV12_Msk       /*!< PREDIV2 input clock divided by 12 */
 #define RCC_CFGR2_PREDIV2_DIV13_Pos          (6U)
 #define RCC_CFGR2_PREDIV2_DIV13_Msk          (0x3U << RCC_CFGR2_PREDIV2_DIV13_Pos) /*!< 0x000000C0 */
 #define RCC_CFGR2_PREDIV2_DIV13              RCC_CFGR2_PREDIV2_DIV13_Msk       /*!< PREDIV2 input clock divided by 13 */
 #define RCC_CFGR2_PREDIV2_DIV14_Pos          (4U)
 #define RCC_CFGR2_PREDIV2_DIV14_Msk          (0xDU << RCC_CFGR2_PREDIV2_DIV14_Pos) /*!< 0x000000D0 */
 #define RCC_CFGR2_PREDIV2_DIV14              RCC_CFGR2_PREDIV2_DIV14_Msk       /*!< PREDIV2 input clock divided by 14 */
 #define RCC_CFGR2_PREDIV2_DIV15_Pos          (5U)
 #define RCC_CFGR2_PREDIV2_DIV15_Msk          (0x7U << RCC_CFGR2_PREDIV2_DIV15_Pos) /*!< 0x000000E0 */
 #define RCC_CFGR2_PREDIV2_DIV15              RCC_CFGR2_PREDIV2_DIV15_Msk       /*!< PREDIV2 input clock divided by 15 */
 #define RCC_CFGR2_PREDIV2_DIV16_Pos          (4U)
 #define RCC_CFGR2_PREDIV2_DIV16_Msk          (0xFU << RCC_CFGR2_PREDIV2_DIV16_Pos) /*!< 0x000000F0 */
 #define RCC_CFGR2_PREDIV2_DIV16              RCC_CFGR2_PREDIV2_DIV16_Msk       /*!< PREDIV2 input clock divided by 16 */

 /*!< PLL2MUL configuration */
 #define RCC_CFGR2_PLL2MUL_Pos                (8U)
 #define RCC_CFGR2_PLL2MUL_Msk                (0xFU << RCC_CFGR2_PLL2MUL_Pos)   /*!< 0x00000F00 */
 #define RCC_CFGR2_PLL2MUL                    RCC_CFGR2_PLL2MUL_Msk             /*!< PLL2MUL[3:0] bits */
 #define RCC_CFGR2_PLL2MUL_0                  (0x1U << RCC_CFGR2_PLL2MUL_Pos)   /*!< 0x00000100 */
 #define RCC_CFGR2_PLL2MUL_1                  (0x2U << RCC_CFGR2_PLL2MUL_Pos)   /*!< 0x00000200 */
 #define RCC_CFGR2_PLL2MUL_2                  (0x4U << RCC_CFGR2_PLL2MUL_Pos)   /*!< 0x00000400 */
 #define RCC_CFGR2_PLL2MUL_3                  (0x8U << RCC_CFGR2_PLL2MUL_Pos)   /*!< 0x00000800 */

 #define RCC_CFGR2_PLL2MUL8_Pos               (9U)
 #define RCC_CFGR2_PLL2MUL8_Msk               (0x3U << RCC_CFGR2_PLL2MUL8_Pos)  /*!< 0x00000600 */
 #define RCC_CFGR2_PLL2MUL8                   RCC_CFGR2_PLL2MUL8_Msk            /*!< PLL2 input clock * 8 */
 #define RCC_CFGR2_PLL2MUL9_Pos               (8U)
 #define RCC_CFGR2_PLL2MUL9_Msk               (0x7U << RCC_CFGR2_PLL2MUL9_Pos)  /*!< 0x00000700 */
 #define RCC_CFGR2_PLL2MUL9                   RCC_CFGR2_PLL2MUL9_Msk            /*!< PLL2 input clock * 9 */
 #define RCC_CFGR2_PLL2MUL10_Pos              (11U)
 #define RCC_CFGR2_PLL2MUL10_Msk              (0x1U << RCC_CFGR2_PLL2MUL10_Pos) /*!< 0x00000800 */
 #define RCC_CFGR2_PLL2MUL10                  RCC_CFGR2_PLL2MUL10_Msk           /*!< PLL2 input clock * 10 */
 #define RCC_CFGR2_PLL2MUL11_Pos              (8U)
 #define RCC_CFGR2_PLL2MUL11_Msk              (0x9U << RCC_CFGR2_PLL2MUL11_Pos) /*!< 0x00000900 */
 #define RCC_CFGR2_PLL2MUL11                  RCC_CFGR2_PLL2MUL11_Msk           /*!< PLL2 input clock * 11 */
 #define RCC_CFGR2_PLL2MUL12_Pos              (9U)
 #define RCC_CFGR2_PLL2MUL12_Msk              (0x5U << RCC_CFGR2_PLL2MUL12_Pos) /*!< 0x00000A00 */
 #define RCC_CFGR2_PLL2MUL12                  RCC_CFGR2_PLL2MUL12_Msk           /*!< PLL2 input clock * 12 */
 #define RCC_CFGR2_PLL2MUL13_Pos              (8U)
 #define RCC_CFGR2_PLL2MUL13_Msk              (0xBU << RCC_CFGR2_PLL2MUL13_Pos) /*!< 0x00000B00 */
 #define RCC_CFGR2_PLL2MUL13                  RCC_CFGR2_PLL2MUL13_Msk           /*!< PLL2 input clock * 13 */
 #define RCC_CFGR2_PLL2MUL14_Pos              (10U)
 #define RCC_CFGR2_PLL2MUL14_Msk              (0x3U << RCC_CFGR2_PLL2MUL14_Pos) /*!< 0x00000C00 */
 #define RCC_CFGR2_PLL2MUL14                  RCC_CFGR2_PLL2MUL14_Msk           /*!< PLL2 input clock * 14 */
 #define RCC_CFGR2_PLL2MUL16_Pos              (9U)
 #define RCC_CFGR2_PLL2MUL16_Msk              (0x7U << RCC_CFGR2_PLL2MUL16_Pos) /*!< 0x00000E00 */
 #define RCC_CFGR2_PLL2MUL16                  RCC_CFGR2_PLL2MUL16_Msk           /*!< PLL2 input clock * 16 */
 #define RCC_CFGR2_PLL2MUL20_Pos              (8U)
 #define RCC_CFGR2_PLL2MUL20_Msk              (0xFU << RCC_CFGR2_PLL2MUL20_Pos) /*!< 0x00000F00 */
 #define RCC_CFGR2_PLL2MUL20                  RCC_CFGR2_PLL2MUL20_Msk           /*!< PLL2 input clock * 20 */

 /*!< PLL3MUL configuration */
 #define RCC_CFGR2_PLL3MUL_Pos                (12U)
 #define RCC_CFGR2_PLL3MUL_Msk                (0xFU << RCC_CFGR2_PLL3MUL_Pos)   /*!< 0x0000F000 */
 #define RCC_CFGR2_PLL3MUL                    RCC_CFGR2_PLL3MUL_Msk             /*!< PLL3MUL[3:0] bits */
 #define RCC_CFGR2_PLL3MUL_0                  (0x1U << RCC_CFGR2_PLL3MUL_Pos)   /*!< 0x00001000 */
 #define RCC_CFGR2_PLL3MUL_1                  (0x2U << RCC_CFGR2_PLL3MUL_Pos)   /*!< 0x00002000 */
 #define RCC_CFGR2_PLL3MUL_2                  (0x4U << RCC_CFGR2_PLL3MUL_Pos)   /*!< 0x00004000 */
 #define RCC_CFGR2_PLL3MUL_3                  (0x8U << RCC_CFGR2_PLL3MUL_Pos)   /*!< 0x00008000 */

 #define RCC_CFGR2_PLL3MUL8_Pos               (13U)
 #define RCC_CFGR2_PLL3MUL8_Msk               (0x3U << RCC_CFGR2_PLL3MUL8_Pos)  /*!< 0x00006000 */
 #define RCC_CFGR2_PLL3MUL8                   RCC_CFGR2_PLL3MUL8_Msk            /*!< PLL3 input clock * 8 */
 #define RCC_CFGR2_PLL3MUL9_Pos               (12U)
 #define RCC_CFGR2_PLL3MUL9_Msk               (0x7U << RCC_CFGR2_PLL3MUL9_Pos)  /*!< 0x00007000 */
 #define RCC_CFGR2_PLL3MUL9                   RCC_CFGR2_PLL3MUL9_Msk            /*!< PLL3 input clock * 9 */
 #define RCC_CFGR2_PLL3MUL10_Pos              (15U)
 #define RCC_CFGR2_PLL3MUL10_Msk              (0x1U << RCC_CFGR2_PLL3MUL10_Pos) /*!< 0x00008000 */
 #define RCC_CFGR2_PLL3MUL10                  RCC_CFGR2_PLL3MUL10_Msk           /*!< PLL3 input clock * 10 */
 #define RCC_CFGR2_PLL3MUL11_Pos              (12U)
 #define RCC_CFGR2_PLL3MUL11_Msk              (0x9U << RCC_CFGR2_PLL3MUL11_Pos) /*!< 0x00009000 */
 #define RCC_CFGR2_PLL3MUL11                  RCC_CFGR2_PLL3MUL11_Msk           /*!< PLL3 input clock * 11 */
 #define RCC_CFGR2_PLL3MUL12_Pos              (13U)
 #define RCC_CFGR2_PLL3MUL12_Msk              (0x5U << RCC_CFGR2_PLL3MUL12_Pos) /*!< 0x0000A000 */
 #define RCC_CFGR2_PLL3MUL12                  RCC_CFGR2_PLL3MUL12_Msk           /*!< PLL3 input clock * 12 */
 #define RCC_CFGR2_PLL3MUL13_Pos              (12U)
 #define RCC_CFGR2_PLL3MUL13_Msk              (0xBU << RCC_CFGR2_PLL3MUL13_Pos) /*!< 0x0000B000 */
 #define RCC_CFGR2_PLL3MUL13                  RCC_CFGR2_PLL3MUL13_Msk           /*!< PLL3 input clock * 13 */
 #define RCC_CFGR2_PLL3MUL14_Pos              (14U)
 #define RCC_CFGR2_PLL3MUL14_Msk              (0x3U << RCC_CFGR2_PLL3MUL14_Pos) /*!< 0x0000C000 */
 #define RCC_CFGR2_PLL3MUL14                  RCC_CFGR2_PLL3MUL14_Msk           /*!< PLL3 input clock * 14 */
 #define RCC_CFGR2_PLL3MUL16_Pos              (13U)
 #define RCC_CFGR2_PLL3MUL16_Msk              (0x7U << RCC_CFGR2_PLL3MUL16_Pos) /*!< 0x0000E000 */
 #define RCC_CFGR2_PLL3MUL16                  RCC_CFGR2_PLL3MUL16_Msk           /*!< PLL3 input clock * 16 */
 #define RCC_CFGR2_PLL3MUL20_Pos              (12U)
 #define RCC_CFGR2_PLL3MUL20_Msk              (0xFU << RCC_CFGR2_PLL3MUL20_Pos) /*!< 0x0000F000 */
 #define RCC_CFGR2_PLL3MUL20                  RCC_CFGR2_PLL3MUL20_Msk           /*!< PLL3 input clock * 20 */

 #define RCC_CFGR2_PREDIV1SRC_Pos             (16U)
 #define RCC_CFGR2_PREDIV1SRC_Msk             (0x1U << RCC_CFGR2_PREDIV1SRC_Pos) /*!< 0x00010000 */
 #define RCC_CFGR2_PREDIV1SRC                 RCC_CFGR2_PREDIV1SRC_Msk          /*!< PREDIV1 entry clock source */
 #define RCC_CFGR2_PREDIV1SRC_PLL2_Pos        (16U)
 #define RCC_CFGR2_PREDIV1SRC_PLL2_Msk        (0x1U << RCC_CFGR2_PREDIV1SRC_PLL2_Pos) /*!< 0x00010000 */
 #define RCC_CFGR2_PREDIV1SRC_PLL2            RCC_CFGR2_PREDIV1SRC_PLL2_Msk     /*!< PLL2 selected as PREDIV1 entry clock source */
 #define RCC_CFGR2_PREDIV1SRC_HSE             0x00000000U                       /*!< HSE selected as PREDIV1 entry clock source */
 #define RCC_CFGR2_I2S2SRC_Pos                (17U)
 #define RCC_CFGR2_I2S2SRC_Msk                (0x1U << RCC_CFGR2_I2S2SRC_Pos)   /*!< 0x00020000 */
 #define RCC_CFGR2_I2S2SRC                    RCC_CFGR2_I2S2SRC_Msk             /*!< I2S2 entry clock source */
 #define RCC_CFGR2_I2S3SRC_Pos                (18U)
 #define RCC_CFGR2_I2S3SRC_Msk                (0x1U << RCC_CFGR2_I2S3SRC_Pos)   /*!< 0x00040000 */
 #define RCC_CFGR2_I2S3SRC                    RCC_CFGR2_I2S3SRC_Msk             /*!< I2S3 clock source */


/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup RCC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RCC_Exported_Types RCC Exported Types
  * @{
  */

/** 
  * @brief  RCC PLL configuration structure definition  
  */
typedef struct
{
  uint32_t PLLState;      /*!< PLLState: The new state of the PLL.
                              This parameter can be a value of @ref RCC_PLL_Config */

  uint32_t PLLSource;     /*!< PLLSource: PLL entry clock source.
                              This parameter must be a value of @ref RCC_PLL_Clock_Source */          

  uint32_t PLLMUL;        /*!< PLLMUL: Multiplication factor for PLL VCO input clock
                              This parameter must be a value of @ref RCCEx_PLL_Multiplication_Factor */
} RCC_PLLInitTypeDef;
   
/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition  
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
} RCC_ClkInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_Exported_Constants RCC Exported Constants
  * @{
  */

/** @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */

#define RCC_PLLSOURCE_HSI_DIV2      0x00000000U     /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC            /*!< HSE clock selected as PLL entry clock source */

/**
  * @}
  */

/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U
/**
  * @}
  */

/** @defgroup RCC_HSE_Config HSE Config
  * @{
  */
#define RCC_HSE_OFF                      0x00000000U                                /*!< HSE clock deactivation */
#define RCC_HSE_ON                       RCC_CR_HSEON                               /*!< HSE clock activation */
#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON)) /*!< External clock source for HSE clock */
/**
  * @}
  */

/** @defgroup RCC_LSE_Config LSE Config
  * @{
  */
#define RCC_LSE_OFF                      0x00000000U                                    /*!< LSE clock deactivation */
#define RCC_LSE_ON                       RCC_BDCR_LSEON                                 /*!< LSE clock activation */
#define RCC_LSE_BYPASS                   ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)) /*!< External clock source for LSE clock */

/**
  * @}
  */

/** @defgroup RCC_HSI_Config HSI Config
  * @{
  */
#define RCC_HSI_OFF                      0x00000000U                      /*!< HSI clock deactivation */
#define RCC_HSI_ON                       RCC_CR_HSION                     /*!< HSI clock activation */

#define RCC_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */

/**
  * @}
  */

/** @defgroup RCC_LSI_Config LSI Config
  * @{
  */
#define RCC_LSI_OFF                      0x00000000U              /*!< LSI clock deactivation */
#define RCC_LSI_ON                       RCC_CSR_LSION            /*!< LSI clock activation */

/**
  * @}
  */

/** @defgroup RCC_PLL_Config PLL Config
  * @{
  */
#define RCC_PLL_NONE                      0x00000000U  /*!< PLL is not configured */
#define RCC_PLL_OFF                       0x00000001U  /*!< PLL deactivation */
#define RCC_PLL_ON                        0x00000002U  /*!< PLL activation */

/**
  * @}
  */

/** @defgroup RCC_System_Clock_Type System Clock Type
  * @{
  */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */

/**
  * @}
  */

/** @defgroup RCC_System_Clock_Source System Clock Source
  * @{
  */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI /*!< HSI selected as system clock */
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */

/**
  * @}
  */

/** @defgroup RCC_System_Clock_Source_Status System Clock Source Status
  * @{
  */
#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */

/**
  * @}
  */

/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */

/**
  * @}
  */
  
/** @defgroup RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
  * @{
  */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */

/**
  * @}
  */

/** @defgroup RCC_RTC_Clock_Source RTC Clock Source
  * @{
  */
#define RCC_RTCCLKSOURCE_NO_CLK          0x00000000U                 /*!< No clock */
#define RCC_RTCCLKSOURCE_LSE             RCC_BDCR_RTCSEL_LSE                  /*!< LSE oscillator clock used as RTC clock */
#define RCC_RTCCLKSOURCE_LSI             RCC_BDCR_RTCSEL_LSI                  /*!< LSI oscillator clock used as RTC clock */
#define RCC_RTCCLKSOURCE_HSE_DIV128      RCC_BDCR_RTCSEL_HSE                    /*!< HSE oscillator clock divided by 128 used as RTC clock */
/**
  * @}
  */


/** @defgroup RCC_MCO_Index MCO Index
  * @{
  */
#define RCC_MCO1                         0x00000000U
#define RCC_MCO                          RCC_MCO1               /*!< MCO1 to be compliant with other families with 2 MCOs*/

/**
  * @}
  */

/** @defgroup RCC_MCOx_Clock_Prescaler MCO Clock Prescaler
  * @{
  */
#define RCC_MCODIV_1                    0x00000000U

/**
  * @}
  */

/** @defgroup RCC_Interrupt Interrupts
  * @{
  */
#define RCC_IT_LSIRDY                    ((uint8_t)RCC_CIR_LSIRDYF)   /*!< LSI Ready Interrupt flag */
#define RCC_IT_LSERDY                    ((uint8_t)RCC_CIR_LSERDYF)   /*!< LSE Ready Interrupt flag */
#define RCC_IT_HSIRDY                    ((uint8_t)RCC_CIR_HSIRDYF)   /*!< HSI Ready Interrupt flag */
#define RCC_IT_HSERDY                    ((uint8_t)RCC_CIR_HSERDYF)   /*!< HSE Ready Interrupt flag */
#define RCC_IT_PLLRDY                    ((uint8_t)RCC_CIR_PLLRDYF)   /*!< PLL Ready Interrupt flag */
#define RCC_IT_CSS                       ((uint8_t)RCC_CIR_CSSF)      /*!< Clock Security System Interrupt flag */
/**
  * @}
  */ 
  
/** @defgroup RCC_Flag Flags
  *        Elements values convention: XXXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - XXX  : Register index
  *                 - 001: CR register
  *                 - 010: BDCR register
  *                 - 011: CSR register
  * @{
  */
/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos))   /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_PINRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_PINRSTF_Pos))  /*!< PIN reset flag */
#define RCC_FLAG_PORRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_PORRSTF_Pos))  /*!< POR/PDR reset flag */
#define RCC_FLAG_SFTRST                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_SFTRSTF_Pos))  /*!< Software Reset flag */
#define RCC_FLAG_IWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_IWDGRSTF_Pos)) /*!< Independent Watchdog reset flag */
#define RCC_FLAG_WWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_WWDGRSTF_Pos)) /*!< Window watchdog reset flag */
#define RCC_FLAG_LPWRRST                 ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LPWRRSTF_Pos)) /*!< Low-Power reset flag */

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup RCC_Exported_Macros RCC Exported Macros
  * @{
  */

/** @defgroup RCC_Peripheral_Clock_Enable_Disable Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  * @{
  */
#define __HAL_RCC_DMA1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_SRAM_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_SRAMEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_SRAMEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_FLITF_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_FLITFEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FLITFEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_CRC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_CRCEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_CRCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_DMA1_CLK_DISABLE()      (RCC->AHBENR &= ~(RCC_AHBENR_DMA1EN))
#define __HAL_RCC_SRAM_CLK_DISABLE()      (RCC->AHBENR &= ~(RCC_AHBENR_SRAMEN))
#define __HAL_RCC_FLITF_CLK_DISABLE()     (RCC->AHBENR &= ~(RCC_AHBENR_FLITFEN))
#define __HAL_RCC_CRC_CLK_DISABLE()       (RCC->AHBENR &= ~(RCC_AHBENR_CRCEN))

/**
  * @}
  */

/** @defgroup RCC_AHB_Peripheral_Clock_Enable_Disable_Status AHB Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_DMA1_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_DMA1EN)) != RESET)
#define __HAL_RCC_DMA1_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_DMA1EN)) == RESET)
#define __HAL_RCC_SRAM_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_SRAMEN)) != RESET)
#define __HAL_RCC_SRAM_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_SRAMEN)) == RESET)
#define __HAL_RCC_FLITF_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_FLITFEN)) != RESET)
#define __HAL_RCC_FLITF_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_FLITFEN)) == RESET)
#define __HAL_RCC_CRC_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_CRCEN)) != RESET)
#define __HAL_RCC_CRC_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_CRCEN)) == RESET)

/**
  * @}
  */

/** @defgroup RCC_APB1_Clock_Enable_Disable APB1 Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @{   
  */
#define __HAL_RCC_TIM2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM3_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_WWDG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USART2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_I2C1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_BKP_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_PWR_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM2_CLK_DISABLE()      (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN))
#define __HAL_RCC_TIM3_CLK_DISABLE()      (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN))
#define __HAL_RCC_WWDG_CLK_DISABLE()      (RCC->APB1ENR &= ~(RCC_APB1ENR_WWDGEN))
#define __HAL_RCC_USART2_CLK_DISABLE()    (RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN))
#define __HAL_RCC_I2C1_CLK_DISABLE()      (RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN))

#define __HAL_RCC_BKP_CLK_DISABLE()       (RCC->APB1ENR &= ~(RCC_APB1ENR_BKPEN))
#define __HAL_RCC_PWR_CLK_DISABLE()       (RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN))

/**
  * @}
  */

/** @defgroup RCC_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_TIM2_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM2EN)) != RESET)
#define __HAL_RCC_TIM2_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM2EN)) == RESET)
#define __HAL_RCC_TIM3_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM3EN)) != RESET)
#define __HAL_RCC_TIM3_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM3EN)) == RESET)
#define __HAL_RCC_WWDG_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_WWDGEN)) != RESET)
#define __HAL_RCC_WWDG_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_WWDGEN)) == RESET)
#define __HAL_RCC_USART2_IS_CLK_ENABLED()     ((RCC->APB1ENR & (RCC_APB1ENR_USART2EN)) != RESET)
#define __HAL_RCC_USART2_IS_CLK_DISABLED()    ((RCC->APB1ENR & (RCC_APB1ENR_USART2EN)) == RESET)
#define __HAL_RCC_I2C1_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_I2C1EN)) != RESET)
#define __HAL_RCC_I2C1_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_I2C1EN)) == RESET)
#define __HAL_RCC_BKP_IS_CLK_ENABLED()        ((RCC->APB1ENR & (RCC_APB1ENR_BKPEN)) != RESET)
#define __HAL_RCC_BKP_IS_CLK_DISABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_BKPEN)) == RESET)
#define __HAL_RCC_PWR_IS_CLK_ENABLED()        ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) != RESET)
#define __HAL_RCC_PWR_IS_CLK_DISABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == RESET)

/**
  * @}
  */

/** @defgroup RCC_APB2_Clock_Enable_Disable APB2 Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{   
  */
#define __HAL_RCC_AFIO_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOD_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_ADC1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_SPI1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USART1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_AFIO_CLK_DISABLE()      (RCC->APB2ENR &= ~(RCC_APB2ENR_AFIOEN))
#define __HAL_RCC_GPIOA_CLK_DISABLE()     (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPAEN))
#define __HAL_RCC_GPIOB_CLK_DISABLE()     (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPBEN))
#define __HAL_RCC_GPIOC_CLK_DISABLE()     (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPCEN))
#define __HAL_RCC_GPIOD_CLK_DISABLE()     (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPDEN))
#define __HAL_RCC_ADC1_CLK_DISABLE()      (RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN))

#define __HAL_RCC_TIM1_CLK_DISABLE()      (RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN))
#define __HAL_RCC_SPI1_CLK_DISABLE()      (RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN))
#define __HAL_RCC_USART1_CLK_DISABLE()    (RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN))

/**
  * @}
  */
  
/** @defgroup RCC_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_AFIO_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_AFIOEN)) != RESET)
#define __HAL_RCC_AFIO_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_AFIOEN)) == RESET)
#define __HAL_RCC_GPIOA_IS_CLK_ENABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPAEN)) != RESET)
#define __HAL_RCC_GPIOA_IS_CLK_DISABLED()     ((RCC->APB2ENR & (RCC_APB2ENR_IOPAEN)) == RESET)
#define __HAL_RCC_GPIOB_IS_CLK_ENABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPBEN)) != RESET)
#define __HAL_RCC_GPIOB_IS_CLK_DISABLED()     ((RCC->APB2ENR & (RCC_APB2ENR_IOPBEN)) == RESET)
#define __HAL_RCC_GPIOC_IS_CLK_ENABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPCEN)) != RESET)
#define __HAL_RCC_GPIOC_IS_CLK_DISABLED()     ((RCC->APB2ENR & (RCC_APB2ENR_IOPCEN)) == RESET)
#define __HAL_RCC_GPIOD_IS_CLK_ENABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPDEN)) != RESET)
#define __HAL_RCC_GPIOD_IS_CLK_DISABLED()     ((RCC->APB2ENR & (RCC_APB2ENR_IOPDEN)) == RESET)
#define __HAL_RCC_ADC1_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_ADC1EN)) != RESET)
#define __HAL_RCC_ADC1_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_ADC1EN)) == RESET)
#define __HAL_RCC_TIM1_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_TIM1EN)) != RESET)
#define __HAL_RCC_TIM1_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_TIM1EN)) == RESET)
#define __HAL_RCC_SPI1_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_SPI1EN)) != RESET)
#define __HAL_RCC_SPI1_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_SPI1EN)) == RESET)
#define __HAL_RCC_USART1_IS_CLK_ENABLED()     ((RCC->APB2ENR & (RCC_APB2ENR_USART1EN)) != RESET)
#define __HAL_RCC_USART1_IS_CLK_DISABLED()    ((RCC->APB2ENR & (RCC_APB2ENR_USART1EN)) == RESET)

/**
  * @}
  */

/** @defgroup RCC_APB1_Force_Release_Reset APB1 Force Release Reset
  * @brief  Force or release APB1 peripheral reset.
  * @{   
  */
#define __HAL_RCC_APB1_FORCE_RESET()       (RCC->APB2RSTR = 0xFFFFFFFFU)  
#define __HAL_RCC_TIM2_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM2RST))
#define __HAL_RCC_TIM3_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM3RST))
#define __HAL_RCC_WWDG_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_WWDGRST))
#define __HAL_RCC_USART2_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_USART2RST))
#define __HAL_RCC_I2C1_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST))

#define __HAL_RCC_BKP_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_BKPRST))
#define __HAL_RCC_PWR_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_PWRRST))

#define __HAL_RCC_APB1_RELEASE_RESET()      (RCC->APB1RSTR = 0x00)  
#define __HAL_RCC_TIM2_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST))
#define __HAL_RCC_TIM3_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST))
#define __HAL_RCC_WWDG_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_WWDGRST))
#define __HAL_RCC_USART2_RELEASE_RESET()     (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART2RST))
#define __HAL_RCC_I2C1_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST))

#define __HAL_RCC_BKP_RELEASE_RESET()        (RCC->APB1RSTR &= ~(RCC_APB1RSTR_BKPRST))
#define __HAL_RCC_PWR_RELEASE_RESET()        (RCC->APB1RSTR &= ~(RCC_APB1RSTR_PWRRST))

/**
  * @}
  */

/** @defgroup RCC_APB2_Force_Release_Reset APB2 Force Release Reset
  * @brief  Force or release APB2 peripheral reset.
  * @{   
  */
#define __HAL_RCC_APB2_FORCE_RESET()       (RCC->APB2RSTR = 0xFFFFFFFFU)  
#define __HAL_RCC_AFIO_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_AFIORST))
#define __HAL_RCC_GPIOA_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPARST))
#define __HAL_RCC_GPIOB_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPBRST))
#define __HAL_RCC_GPIOC_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPCRST))
#define __HAL_RCC_GPIOD_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPDRST))
#define __HAL_RCC_ADC1_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_ADC1RST))

#define __HAL_RCC_TIM1_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM1RST))
#define __HAL_RCC_SPI1_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST))
#define __HAL_RCC_USART1_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_USART1RST))

#define __HAL_RCC_APB2_RELEASE_RESET()      (RCC->APB2RSTR = 0x00)  
#define __HAL_RCC_AFIO_RELEASE_RESET()       (RCC->APB2RSTR &= ~(RCC_APB2RSTR_AFIORST))
#define __HAL_RCC_GPIOA_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPARST))
#define __HAL_RCC_GPIOB_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPBRST))
#define __HAL_RCC_GPIOC_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPCRST))
#define __HAL_RCC_GPIOD_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPDRST))
#define __HAL_RCC_ADC1_RELEASE_RESET()       (RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADC1RST))

#define __HAL_RCC_TIM1_RELEASE_RESET()       (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST))
#define __HAL_RCC_SPI1_RELEASE_RESET()       (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST))
#define __HAL_RCC_USART1_RELEASE_RESET()     (RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST))

/**
  * @}
  */

/** @defgroup RCC_HSI_Configuration HSI Configuration
  * @{   
  */

/** @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  * @note   HSI can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the HSI.  
  * @note   After enabling the HSI, the application software should wait on HSIRDY
  *         flag to be set indicating that HSI clock is stable and can be used as
  *         system clock source.  
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.  
  */
#define __HAL_RCC_HSI_ENABLE()  (*(__IO uint32_t *) RCC_CR_HSION_BB = ENABLE)
#define __HAL_RCC_HSI_DISABLE() (*(__IO uint32_t *) RCC_CR_HSION_BB = DISABLE)

/** @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  _HSICALIBRATIONVALUE_ specifies the calibration trimming value.
  *         (default is RCC_HSICALIBRATION_DEFAULT).
  *         This parameter must be a number between 0 and 0x1F.
  */  
#define __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) \
          (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << RCC_CR_HSITRIM_Pos))

/**
  * @}
  */

/** @defgroup RCC_LSI_Configuration  LSI Configuration
  * @{   
  */

/** @brief Macro to enable the Internal Low Speed oscillator (LSI).
  * @note   After enabling the LSI, the application software should wait on 
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDG and/or the RTC.
  */
#define __HAL_RCC_LSI_ENABLE()  (*(__IO uint32_t *) RCC_CSR_LSION_BB = ENABLE)

/** @brief Macro to disable the Internal Low Speed oscillator (LSI).
  * @note   LSI can not be disabled if the IWDG is running.  
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles. 
  */
#define __HAL_RCC_LSI_DISABLE() (*(__IO uint32_t *) RCC_CSR_LSION_BB = DISABLE)

/**
  * @}
  */

/** @defgroup RCC_HSE_Configuration HSE Configuration
  * @{   
  */

/**
  * @brief  Macro to configure the External High Speed oscillator (HSE).
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this macro. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__ specifies the new state of the HSE.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_HSE_OFF turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg @ref RCC_HSE_ON turn ON the HSE oscillator
  *            @arg @ref RCC_HSE_BYPASS HSE oscillator bypassed with external clock
  */
#define __HAL_RCC_HSE_CONFIG(__STATE__)                                     \
                    do{                                                     \
                      if ((__STATE__) == RCC_HSE_ON)                        \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_OFF)                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_BYPASS)               \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEBYP);                    \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else                                                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                    }while(0U)

/**
  * @}
  */

/** @defgroup RCC_LSE_Configuration LSE Configuration
  * @{   
  */

/**
  * @brief  Macro to configure the External Low Speed oscillator (LSE).
  * @note Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro. 
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using 
  *         @ref HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).  
  * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
  *         software should wait on LSERDY flag to be set indicating that LSE clock
  *         is stable and can be used to clock the RTC.
  * @param  __STATE__ specifies the new state of the LSE.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_LSE_OFF turn OFF the LSE oscillator, LSERDY flag goes low after
  *                              6 LSE oscillator clock cycles.
  *            @arg @ref RCC_LSE_ON turn ON the LSE oscillator.
  *            @arg @ref RCC_LSE_BYPASS LSE oscillator bypassed with external clock.
  */
#define __HAL_RCC_LSE_CONFIG(__STATE__)                                     \
                    do{                                                     \
                      if ((__STATE__) == RCC_LSE_ON)                        \
                      {                                                     \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);                   \
                      }                                                     \
                      else if ((__STATE__) == RCC_LSE_OFF)                  \
                      {                                                     \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);                 \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                \
                      }                                                     \
                      else if ((__STATE__) == RCC_LSE_BYPASS)               \
                      {                                                     \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                  \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);                   \
                      }                                                     \
                      else                                                  \
                      {                                                     \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);                 \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                \
                      }                                                     \
                    }while(0U)

/**
  * @}
  */

/** @defgroup RCC_PLL_Configuration PLL Configuration
  * @{   
  */

/** @brief Macro to enable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on 
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  */
#define __HAL_RCC_PLL_ENABLE()          (*(__IO uint32_t *) RCC_CR_PLLON_BB = ENABLE)

/** @brief Macro to disable the main PLL.
  * @note   The main PLL can not be disabled if it is used as system clock source
  */
#define __HAL_RCC_PLL_DISABLE()         (*(__IO uint32_t *) RCC_CR_PLLON_BB = DISABLE)

/** @brief Macro to configure the main PLL clock source and multiplication factors.
  * @note   This function must be used only when the main PLL is disabled.
  *  
  * @param  __RCC_PLLSOURCE__ specifies the PLL entry clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_PLLSOURCE_HSI_DIV2 HSI oscillator clock selected as PLL clock entry
  *            @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL clock entry
  * @param  __PLLMUL__ specifies the multiplication factor for PLL VCO output clock
  *          This parameter can be one of the following values:
  *             @arg @ref RCC_PLL_MUL4   PLLVCO = PLL clock entry x 4
  *             @arg @ref RCC_PLL_MUL6   PLLVCO = PLL clock entry x 6
  @if STM32F105xC
  *             @arg @ref RCC_PLL_MUL6_5 PLLVCO = PLL clock entry x 6.5
  @elseif STM32F107xC
  *             @arg @ref RCC_PLL_MUL6_5 PLLVCO = PLL clock entry x 6.5
  @else
  *             @arg @ref RCC_PLL_MUL2   PLLVCO = PLL clock entry x 2
  *             @arg @ref RCC_PLL_MUL3   PLLVCO = PLL clock entry x 3
  *             @arg @ref RCC_PLL_MUL10  PLLVCO = PLL clock entry x 10
  *             @arg @ref RCC_PLL_MUL11  PLLVCO = PLL clock entry x 11
  *             @arg @ref RCC_PLL_MUL12  PLLVCO = PLL clock entry x 12
  *             @arg @ref RCC_PLL_MUL13  PLLVCO = PLL clock entry x 13
  *             @arg @ref RCC_PLL_MUL14  PLLVCO = PLL clock entry x 14
  *             @arg @ref RCC_PLL_MUL15  PLLVCO = PLL clock entry x 15
  *             @arg @ref RCC_PLL_MUL16  PLLVCO = PLL clock entry x 16
  @endif
  *             @arg @ref RCC_PLL_MUL8   PLLVCO = PLL clock entry x 8
  *             @arg @ref RCC_PLL_MUL9   PLLVCO = PLL clock entry x 9
  *   
  */
#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__)\
          MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL),((__RCC_PLLSOURCE__) | (__PLLMUL__) ))

/** @brief  Get oscillator clock selected as PLL input clock
  * @retval The clock source used for PLL entry. The returned value can be one
  *         of the following:
  *             @arg @ref RCC_PLLSOURCE_HSI_DIV2 HSI oscillator clock selected as PLL input clock
  *             @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL input clock
  */
#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

/**
  * @}
  */

/** @defgroup RCC_Get_Clock_source Get Clock source
  * @{   
  */

/**
  * @brief  Macro to configure the system clock source.
  * @param  __SYSCLKSOURCE__ specifies the system clock source.
  *          This parameter can be one of the following values:
  *              @arg @ref RCC_SYSCLKSOURCE_HSI HSI oscillator is used as system clock source.
  *              @arg @ref RCC_SYSCLKSOURCE_HSE HSE oscillator is used as system clock source.
  *              @arg @ref RCC_SYSCLKSOURCE_PLLCLK PLL output is used as system clock source.
  */
#define __HAL_RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

/** @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSI HSI used as system clock
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSE HSE used as system clock
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_PLLCLK PLL used as system clock
  */
#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

/**
  * @}
  */

/** @defgroup RCCEx_MCOx_Clock_Config RCC Extended MCOx Clock Config
  * @{   
  */ 

#if   defined(RCC_CFGR_MCO_3)
/** @brief  Macro to configure the MCO clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_MCO1SOURCE_NOCLOCK      No clock selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_SYSCLK       System clock (SYSCLK) selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_HSI          HSI selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_HSE          HSE selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_PLLCLK       PLL clock divided by 2 selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_PLL2CLK      PLL2 clock selected by 2 selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_PLL3CLK_DIV2 PLL3 clock divided by 2 selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_EXT_HSE XT1  external 3-25 MHz oscillator clock selected (for Ethernet) as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_PLL3CLK      PLL3 clock selected (for Ethernet) as MCO clock
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_MCODIV_1 No division applied on MCO clock source
  */
#else
/** @brief  Macro to configure the MCO clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_MCO1SOURCE_NOCLOCK No clock selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_SYSCLK  System clock (SYSCLK) selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_HSI HSI selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_HSE HSE selected as MCO clock
  *            @arg @ref RCC_MCO1SOURCE_PLLCLK  PLL clock divided by 2 selected as MCO clock
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_MCODIV_1 No division applied on MCO clock source
  */
#endif

#define __HAL_RCC_MCO1_CONFIG(__MCOCLKSOURCE__, __MCODIV__) \
                 MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, (__MCOCLKSOURCE__))


/**
  * @}
  */

  /** @defgroup RCC_RTC_Clock_Configuration RCC RTC Clock Configuration
  * @{   
  */

/** @brief Macro to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).    
  * @note   Once the RTC clock is configured it can't be changed unless the  
  *         Backup domain is reset using @ref __HAL_RCC_BACKUPRESET_FORCE() macro, or by
  *         a Power On Reset (POR).
  *
  * @param  __RTC_CLKSOURCE__ specifies the RTC clock source.
  *          This parameter can be one of the following values:
  *             @arg @ref RCC_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_LSE LSE selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_LSI LSI selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV128 HSE divided by 128 selected as RTC clock
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wakeup source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.    
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  */
#define __HAL_RCC_RTC_CONFIG(__RTC_CLKSOURCE__) MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, (__RTC_CLKSOURCE__))
                                                   
/** @brief Macro to get the RTC clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
  *            @arg @ref RCC_RTCCLKSOURCE_LSE LSE selected as RTC clock
  *            @arg @ref RCC_RTCCLKSOURCE_LSI LSI selected as RTC clock
  *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIV128 HSE divided by 128 selected as RTC clock
  */
#define __HAL_RCC_GET_RTC_SOURCE() (READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL))

/** @brief Macro to enable the the RTC clock.
  * @note   These macros must be used only after the RTC clock source was selected.
  */
#define __HAL_RCC_RTC_ENABLE()          (*(__IO uint32_t *) RCC_BDCR_RTCEN_BB = ENABLE)

/** @brief Macro to disable the the RTC clock.
  * @note  These macros must be used only after the RTC clock source was selected.
  */
#define __HAL_RCC_RTC_DISABLE()         (*(__IO uint32_t *) RCC_BDCR_RTCEN_BB = DISABLE)

/** @brief  Macro to force the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_BDCR register.
  */
#define __HAL_RCC_BACKUPRESET_FORCE()   (*(__IO uint32_t *) RCC_BDCR_BDRST_BB = ENABLE)

/** @brief  Macros to release the Backup domain reset.
  */
#define __HAL_RCC_BACKUPRESET_RELEASE() (*(__IO uint32_t *) RCC_BDCR_BDRST_BB = DISABLE)

/**
  * @}
  */

/** @defgroup RCC_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @{
  */

/** @brief Enable RCC interrupt.
  * @param  __INTERRUPT__ specifies the RCC interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg @ref RCC_IT_LSIRDY LSI ready interrupt
  *            @arg @ref RCC_IT_LSERDY LSE ready interrupt
  *            @arg @ref RCC_IT_HSIRDY HSI ready interrupt
  *            @arg @ref RCC_IT_HSERDY HSE ready interrupt
  *            @arg @ref RCC_IT_PLLRDY main PLL ready interrupt
  @if STM32F105xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @elsif STM32F107xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @endif
  */
#define __HAL_RCC_ENABLE_IT(__INTERRUPT__) (*(__IO uint8_t *) RCC_CIR_BYTE1_ADDRESS |= (__INTERRUPT__))

/** @brief Disable RCC interrupt.
  * @param  __INTERRUPT__ specifies the RCC interrupt sources to be disabled.
  *          This parameter can be any combination of the following values:
  *            @arg @ref RCC_IT_LSIRDY LSI ready interrupt
  *            @arg @ref RCC_IT_LSERDY LSE ready interrupt
  *            @arg @ref RCC_IT_HSIRDY HSI ready interrupt
  *            @arg @ref RCC_IT_HSERDY HSE ready interrupt
  *            @arg @ref RCC_IT_PLLRDY main PLL ready interrupt
  @if STM32F105xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @elsif STM32F107xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @endif
  */
#define __HAL_RCC_DISABLE_IT(__INTERRUPT__) (*(__IO uint8_t *) RCC_CIR_BYTE1_ADDRESS &= (uint8_t)(~(__INTERRUPT__)))

/** @brief Clear the RCC's interrupt pending bits.
  * @param  __INTERRUPT__ specifies the interrupt pending bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg @ref RCC_IT_LSIRDY LSI ready interrupt.
  *            @arg @ref RCC_IT_LSERDY LSE ready interrupt.
  *            @arg @ref RCC_IT_HSIRDY HSI ready interrupt.
  *            @arg @ref RCC_IT_HSERDY HSE ready interrupt.
  *            @arg @ref RCC_IT_PLLRDY Main PLL ready interrupt.
  @if STM32F105xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @elsif STM32F107xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @endif
  *            @arg @ref RCC_IT_CSS Clock Security System interrupt
  */
#define __HAL_RCC_CLEAR_IT(__INTERRUPT__) (*(__IO uint8_t *) RCC_CIR_BYTE2_ADDRESS = (__INTERRUPT__))

/** @brief Check the RCC's interrupt has occurred or not.
  * @param  __INTERRUPT__ specifies the RCC interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_IT_LSIRDY LSI ready interrupt.
  *            @arg @ref RCC_IT_LSERDY LSE ready interrupt.
  *            @arg @ref RCC_IT_HSIRDY HSI ready interrupt.
  *            @arg @ref RCC_IT_HSERDY HSE ready interrupt.
  *            @arg @ref RCC_IT_PLLRDY Main PLL ready interrupt.
  @if STM32F105xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @elsif STM32F107xx
  *            @arg @ref RCC_IT_PLL2RDY Main PLL2 ready interrupt.
  *            @arg @ref RCC_IT_PLLI2S2RDY Main PLLI2S ready interrupt.
  @endif
  *            @arg @ref RCC_IT_CSS Clock Security System interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_RCC_GET_IT(__INTERRUPT__) ((RCC->CIR & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief Set RMVF bit to clear the reset flags.
  *         The reset flags are RCC_FLAG_PINRST, RCC_FLAG_PORRST, RCC_FLAG_SFTRST,
  *         RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST
  */
#define __HAL_RCC_CLEAR_RESET_FLAGS() (*(__IO uint32_t *)RCC_CSR_RMVF_BB = ENABLE)

/** @brief  Check RCC flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_FLAG_HSIRDY HSI oscillator clock ready.
  *            @arg @ref RCC_FLAG_HSERDY HSE oscillator clock ready.
  *            @arg @ref RCC_FLAG_PLLRDY Main PLL clock ready.
  @if STM32F105xx
  *            @arg @ref RCC_FLAG_PLL2RDY Main PLL2 clock ready.
  *            @arg @ref RCC_FLAG_PLLI2SRDY Main PLLI2S clock ready.
  @elsif STM32F107xx
  *            @arg @ref RCC_FLAG_PLL2RDY Main PLL2 clock ready.
  *            @arg @ref RCC_FLAG_PLLI2SRDY Main PLLI2S clock ready.
  @endif
  *            @arg @ref RCC_FLAG_LSERDY LSE oscillator clock ready.
  *            @arg @ref RCC_FLAG_LSIRDY LSI oscillator clock ready.
  *            @arg @ref RCC_FLAG_PINRST  Pin reset.
  *            @arg @ref RCC_FLAG_PORRST  POR/PDR reset.
  *            @arg @ref RCC_FLAG_SFTRST  Software reset.
  *            @arg @ref RCC_FLAG_IWDGRST Independent Watchdog reset.
  *            @arg @ref RCC_FLAG_WWDGRST Window Watchdog reset.
  *            @arg @ref RCC_FLAG_LPWRRST Low Power reset.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))

/**
  * @}
  */

/**
  * @}
  */

/* Include RCC HAL Extension module */
#include "stm32f1xx_hal_rcc_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCC_Exported_Functions
  * @{
  */

/** @addtogroup RCC_Exported_Functions_Group1
  * @{
  */

/* Initialization and de-initialization functions  ******************************/
void              HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);

/**
  * @}
  */

/** @addtogroup RCC_Exported_Functions_Group2
  * @{
  */

/* Peripheral Control functions  ************************************************/
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
void              HAL_RCC_DisableCSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency);

/* CSS NMI IRQ handler */
void              HAL_RCC_NMI_IRQHandler(void);

/* User Callbacks in non blocking mode (IT mode) */
void              HAL_RCC_CSSCallback(void);

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup RCC_Private_Constants
  * @{
  */

/** @defgroup RCC_Timeout RCC Timeout
  * @{
  */ 
  
/* Disable Backup domain write protection state change timeout */
#define RCC_DBP_TIMEOUT_VALUE          100U    /* 100 ms */
/* LSE state change timeout */
#define RCC_LSE_TIMEOUT_VALUE          LSE_STARTUP_TIMEOUT
#define CLOCKSWITCH_TIMEOUT_VALUE      5000    /* 5 s    */
#define HSE_TIMEOUT_VALUE              HSE_STARTUP_TIMEOUT
#define HSI_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */

/**
  * @}
  */
  
/** @defgroup RCC_Register_Offset Register offsets
  * @{
  */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
#define RCC_CR_OFFSET             0x00U
#define RCC_CFGR_OFFSET           0x04U
#define RCC_CIR_OFFSET            0x08U
#define RCC_BDCR_OFFSET           0x20U
#define RCC_CSR_OFFSET            0x24U

/**
  * @}
  */

/** @defgroup RCC_BitAddress_AliasRegion BitAddress AliasRegion
  * @brief RCC registers bit address in the alias region
  * @{
  */
#define RCC_CR_OFFSET_BB          (RCC_OFFSET + RCC_CR_OFFSET)
#define RCC_CFGR_OFFSET_BB        (RCC_OFFSET + RCC_CFGR_OFFSET)
#define RCC_CIR_OFFSET_BB         (RCC_OFFSET + RCC_CIR_OFFSET)
#define RCC_BDCR_OFFSET_BB        (RCC_OFFSET + RCC_BDCR_OFFSET)
#define RCC_CSR_OFFSET_BB         (RCC_OFFSET + RCC_CSR_OFFSET)

/* --- CR Register ---*/
/* Alias word address of HSION bit */
#define RCC_HSION_BIT_NUMBER      RCC_CR_HSION_Pos
#define RCC_CR_HSION_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSION_BIT_NUMBER * 4U)))
/* Alias word address of HSEON bit */
#define RCC_HSEON_BIT_NUMBER      RCC_CR_HSEON_Pos
#define RCC_CR_HSEON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSEON_BIT_NUMBER * 4U)))
/* Alias word address of CSSON bit */
#define RCC_CSSON_BIT_NUMBER      RCC_CR_CSSON_Pos
#define RCC_CR_CSSON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_CSSON_BIT_NUMBER * 4U)))
/* Alias word address of PLLON bit */
#define RCC_PLLON_BIT_NUMBER      RCC_CR_PLLON_Pos
#define RCC_CR_PLLON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))

/* --- CSR Register ---*/
/* Alias word address of LSION bit */
#define RCC_LSION_BIT_NUMBER      RCC_CSR_LSION_Pos
#define RCC_CSR_LSION_BB          ((uint32_t)(PERIPH_BB_BASE + (RCC_CSR_OFFSET_BB * 32U) + (RCC_LSION_BIT_NUMBER * 4U)))

/* Alias word address of RMVF bit */
#define RCC_RMVF_BIT_NUMBER       RCC_CSR_RMVF_Pos
#define RCC_CSR_RMVF_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CSR_OFFSET_BB * 32U) + (RCC_RMVF_BIT_NUMBER * 4U)))

/* --- BDCR Registers ---*/
/* Alias word address of LSEON bit */
#define RCC_LSEON_BIT_NUMBER      RCC_BDCR_LSEON_Pos
#define RCC_BDCR_LSEON_BB          ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_LSEON_BIT_NUMBER * 4U)))

/* Alias word address of LSEON bit */
#define RCC_LSEBYP_BIT_NUMBER     RCC_BDCR_LSEBYP_Pos
#define RCC_BDCR_LSEBYP_BB         ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_LSEBYP_BIT_NUMBER * 4U)))

/* Alias word address of RTCEN bit */
#define RCC_RTCEN_BIT_NUMBER      RCC_BDCR_RTCEN_Pos
#define RCC_BDCR_RTCEN_BB          ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_RTCEN_BIT_NUMBER * 4U)))

/* Alias word address of BDRST bit */
#define RCC_BDRST_BIT_NUMBER      RCC_BDCR_BDRST_Pos
#define RCC_BDCR_BDRST_BB         ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_BDRST_BIT_NUMBER * 4U)))

/**
  * @}
  */
  
/* CR register byte 2 (Bits[23:16]) base address */
#define RCC_CR_BYTE2_ADDRESS          ((uint32_t)(RCC_BASE + RCC_CR_OFFSET + 0x02U))

/* CIR register byte 1 (Bits[15:8]) base address */
#define RCC_CIR_BYTE1_ADDRESS     ((uint32_t)(RCC_BASE + RCC_CIR_OFFSET + 0x01U))

/* CIR register byte 2 (Bits[23:16]) base address */
#define RCC_CIR_BYTE2_ADDRESS     ((uint32_t)(RCC_BASE + RCC_CIR_OFFSET + 0x02U))

/* Defines used for Flags */
#define CR_REG_INDEX                     ((uint8_t)1)
#define BDCR_REG_INDEX                   ((uint8_t)2)
#define CSR_REG_INDEX                    ((uint8_t)3)

#define RCC_FLAG_MASK                    ((uint8_t)0x1F)

/**
  * @}
  */

/** @addtogroup RCC_Private_Macros
  * @{
  */
/** @defgroup RCC_Alias_For_Legacy Alias define maintained for legacy
  * @{
  */
#define __HAL_RCC_SYSCFG_CLK_DISABLE    __HAL_RCC_AFIO_CLK_DISABLE
#define __HAL_RCC_SYSCFG_CLK_ENABLE     __HAL_RCC_AFIO_CLK_ENABLE
#define __HAL_RCC_SYSCFG_FORCE_RESET    __HAL_RCC_AFIO_FORCE_RESET
#define __HAL_RCC_SYSCFG_RELEASE_RESET  __HAL_RCC_AFIO_RELEASE_RESET
/**
  * @}
  */

#define IS_RCC_PLLSOURCE(__SOURCE__) (((__SOURCE__) == RCC_PLLSOURCE_HSI_DIV2) || \
                                      ((__SOURCE__) == RCC_PLLSOURCE_HSE))
#define IS_RCC_OSCILLATORTYPE(__OSCILLATOR__) (((__OSCILLATOR__) == RCC_OSCILLATORTYPE_NONE)                           || \
                                               (((__OSCILLATOR__) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE) || \
                                               (((__OSCILLATOR__) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI) || \
                                               (((__OSCILLATOR__) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI) || \
                                               (((__OSCILLATOR__) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE))
#define IS_RCC_HSE(__HSE__) (((__HSE__) == RCC_HSE_OFF) || ((__HSE__) == RCC_HSE_ON) || \
                             ((__HSE__) == RCC_HSE_BYPASS))
#define IS_RCC_LSE(__LSE__) (((__LSE__) == RCC_LSE_OFF) || ((__LSE__) == RCC_LSE_ON) || \
                             ((__LSE__) == RCC_LSE_BYPASS))
#define IS_RCC_HSI(__HSI__) (((__HSI__) == RCC_HSI_OFF) || ((__HSI__) == RCC_HSI_ON))
#define IS_RCC_CALIBRATION_VALUE(__VALUE__) ((__VALUE__) <= 0x1FU)
#define IS_RCC_LSI(__LSI__) (((__LSI__) == RCC_LSI_OFF) || ((__LSI__) == RCC_LSI_ON))
#define IS_RCC_PLL(__PLL__) (((__PLL__) == RCC_PLL_NONE) || ((__PLL__) == RCC_PLL_OFF) || \
                             ((__PLL__) == RCC_PLL_ON))

#define IS_RCC_CLOCKTYPE(CLK) ((((CLK) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK) || \
                               (((CLK) & RCC_CLOCKTYPE_HCLK)   == RCC_CLOCKTYPE_HCLK)   || \
                               (((CLK) & RCC_CLOCKTYPE_PCLK1)  == RCC_CLOCKTYPE_PCLK1)  || \
                               (((CLK) & RCC_CLOCKTYPE_PCLK2)  == RCC_CLOCKTYPE_PCLK2))
#define IS_RCC_SYSCLKSOURCE(__SOURCE__) (((__SOURCE__) == RCC_SYSCLKSOURCE_HSI) || \
                                         ((__SOURCE__) == RCC_SYSCLKSOURCE_HSE) || \
                                         ((__SOURCE__) == RCC_SYSCLKSOURCE_PLLCLK))
#define IS_RCC_SYSCLKSOURCE_STATUS(__SOURCE__) (((__SOURCE__) == RCC_SYSCLKSOURCE_STATUS_HSI) || \
                                                ((__SOURCE__) == RCC_SYSCLKSOURCE_STATUS_HSE) || \
                                                ((__SOURCE__) == RCC_SYSCLKSOURCE_STATUS_PLLCLK))
#define IS_RCC_HCLK(__HCLK__) (((__HCLK__) == RCC_SYSCLK_DIV1) || ((__HCLK__) == RCC_SYSCLK_DIV2) || \
                               ((__HCLK__) == RCC_SYSCLK_DIV4) || ((__HCLK__) == RCC_SYSCLK_DIV8) || \
                               ((__HCLK__) == RCC_SYSCLK_DIV16) || ((__HCLK__) == RCC_SYSCLK_DIV64) || \
                               ((__HCLK__) == RCC_SYSCLK_DIV128) || ((__HCLK__) == RCC_SYSCLK_DIV256) || \
                               ((__HCLK__) == RCC_SYSCLK_DIV512))
#define IS_RCC_PCLK(__PCLK__) (((__PCLK__) == RCC_HCLK_DIV1) || ((__PCLK__) == RCC_HCLK_DIV2) || \
                               ((__PCLK__) == RCC_HCLK_DIV4) || ((__PCLK__) == RCC_HCLK_DIV8) || \
                               ((__PCLK__) == RCC_HCLK_DIV16))
#define IS_RCC_MCO(__MCO__)  ((__MCO__) == RCC_MCO)
#define IS_RCC_MCODIV(__DIV__) (((__DIV__) == RCC_MCODIV_1)) 
#define IS_RCC_RTCCLKSOURCE(__SOURCE__)  (((__SOURCE__) == RCC_RTCCLKSOURCE_NO_CLK) || \
                                          ((__SOURCE__) == RCC_RTCCLKSOURCE_LSE) || \
                                          ((__SOURCE__) == RCC_RTCCLKSOURCE_LSI) || \
                                          ((__SOURCE__) == RCC_RTCCLKSOURCE_HSE_DIV128))

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_RCC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

