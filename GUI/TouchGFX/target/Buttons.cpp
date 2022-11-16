/******************************************************************************
 *
 * @brief     This file is part of the TouchGFX 4.8.0 evaluation distribution.
 *
 * @author    Draupner Graphics A/S <http://www.touchgfx.com>
 *
 ******************************************************************************
 *
 * @section Copyright
 *
 * Copyright (C) 2014-2016 Draupner Graphics A/S <http://www.touchgfx.com>.
 * All rights reserved.
 *
 * TouchGFX is protected by international copyright laws and the knowledge of
 * this source code may not be used to write a similar product. This file may
 * only be used in accordance with a license and should not be re-
 * distributed in any way without the prior permission of Draupner Graphics.
 *
 * This is licensed software for evaluation use, any use must strictly comply
 * with the evaluation license agreement provided with delivery of the
 * TouchGFX software.
 *
 * The evaluation license agreement can be seen on www.touchgfx.com
 *
 * @section Disclaimer
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Draupner Graphics A/S has
 * no obligation to support this software. Draupner Graphics A/S is providing
 * the software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Draupner Graphics A/S can not be held liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this software.
 *
 *****************************************************************************/
#include <touchgfx/hal/Buttons.hpp>
#include "stm32l4xx_hal.h"

using namespace touchgfx;

extern "C"
{
    /** @defgroup STM32L496G_DISCOVERY_BUTTON  BUTTON Constants
      * @{
      */
#define JOYn                              5

    /**
    * @brief Joystick Right push-button
    */
#define RIGHT_JOY_PIN                     GPIO_PIN_11  /* PF.11 */
#define RIGHT_JOY_GPIO_PORT               GPIOF
#define RIGHT_JOY_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn               EXTI15_10_IRQn

    /**
    * @brief Joystick Left push-button
    */
#define LEFT_JOY_PIN                      GPIO_PIN_9  /* PI.09 */
#define LEFT_JOY_GPIO_PORT                GPIOI
#define LEFT_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOI_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                EXTI9_5_IRQn

    /**
    * @brief Joystick Up push-button
    */
#define UP_JOY_PIN                        GPIO_PIN_8  /* PI.08 */
#define UP_JOY_GPIO_PORT                  GPIOI
#define UP_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOI_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOI_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                  EXTI9_5_IRQn

    /**
     * @brief Joystick Down push-button
     */
#define DOWN_JOY_PIN                      GPIO_PIN_10   /* PI.10 */
#define DOWN_JOY_GPIO_PORT                GPIOI
#define DOWN_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()
#define DOWN_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOI_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                EXTI15_10_IRQn

    /**
     * @brief Joystick Sel push-button
     */
#define SEL_JOY_PIN                       GPIO_PIN_13   /* PC.13 */
#define SEL_JOY_GPIO_PORT                 GPIOC
#define SEL_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                 EXTI15_10_IRQn

#define JOYx_GPIO_CLK_ENABLE(__JOY__)     do { if((__JOY__) == JOY_SEL)   { SEL_JOY_GPIO_CLK_ENABLE();   } else \
                                                 if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_ENABLE();  } else \
                                                 if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_ENABLE();  } else \
                                                 if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_ENABLE(); } else \
                                                 if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_ENABLE(); }  } while(0)

#define JOYx_GPIO_CLK_DISABLE(__JOY__)    do { if((__JOY__) == JOY_SEL)   { SEL_JOY_GPIO_CLK_DISABLE();   } else \
                                                 if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_DISABLE();  } else \
                                                 if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_DISABLE();  } else \
                                                 if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_DISABLE(); } else \
                                                 if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_DISABLE(); }  } while(0)

#define JOY_ALL_PINS                      (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN | SEL_JOY_PIN)

    /**
     * @brief JOYSTICK Types Definition
     */
    typedef enum
    {
        JOY_SEL   = 0,
        JOY_LEFT  = 1,
        JOY_RIGHT = 2,
        JOY_DOWN  = 3,
        JOY_UP    = 4,
        JOY_NONE  = 5
    } JOYState_TypeDef;

    typedef enum
    {
        JOY_MODE_GPIO = 0,
        JOY_MODE_EXTI = 1
    } JOYMode_TypeDef;

    /**
     * @brief JOYSTICK variables
     */
    static GPIO_TypeDef* JOY_PORT[JOYn] = {SEL_JOY_GPIO_PORT,
                                           LEFT_JOY_GPIO_PORT,
                                           RIGHT_JOY_GPIO_PORT,
                                           DOWN_JOY_GPIO_PORT,
                                           UP_JOY_GPIO_PORT
                                          };

    static const uint16_t JOY_PIN[JOYn] = { SEL_JOY_PIN,
                                            LEFT_JOY_PIN,
                                            RIGHT_JOY_PIN,
                                            DOWN_JOY_PIN,
                                            UP_JOY_PIN
                                          };

    static const uint8_t JOY_IRQn[JOYn] = { SEL_JOY_EXTI_IRQn,
                                            LEFT_JOY_EXTI_IRQn,
                                            RIGHT_JOY_EXTI_IRQn,
                                            DOWN_JOY_EXTI_IRQn,
                                            UP_JOY_EXTI_IRQn
                                          };

    uint8_t          BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
    JOYState_TypeDef BSP_JOY_GetState(void);
}

void Buttons::init()
{
    BSP_JOY_Init(JOY_MODE_GPIO);
}

unsigned int Buttons::sample()
{
    JOYState_TypeDef buttonValue = BSP_JOY_GetState();

    if (buttonValue == JOY_NONE)
    {
        return 0;
    }

    return static_cast<unsigned int>(JOY_NONE - buttonValue);
}

extern "C"
{
    /**
      * @brief  Configures all buttons of the joystick in GPIO or EXTI modes.
      * @param  Joy_Mode: Joystick mode.
      *    This parameter can be one of the following values:
      *     @arg  JOY_MODE_GPIO: Joystick pins will be used as simple IOs
      *     @arg  JOY_MODE_EXTI: Joystick pins will be connected to EXTI line
      *                                 with interrupt generation capability
      * @retval HAL_OK: if all initializations are OK. Other value if error.
      */
    __weak uint8_t BSP_JOY_Init(JOYMode_TypeDef Joy_Mode)
    {
        JOYState_TypeDef joykey;
        GPIO_InitTypeDef GPIO_InitStruct;

        /* Initialized the Joystick. */
        for (int joykey_id = JOY_SEL; joykey_id < (JOY_SEL + JOYn) ; joykey_id++)
        {
            joykey = static_cast<JOYState_TypeDef>(joykey_id);

            /* Enable the JOY clock */
            JOYx_GPIO_CLK_ENABLE(joykey);

            GPIO_InitStruct.Pin = JOY_PIN[joykey];
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

            if (Joy_Mode == JOY_MODE_GPIO)
            {
                /* Configure Joy pin as input */
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);
            }
            else if (Joy_Mode == JOY_MODE_EXTI)
            {
                /* Configure Joy pin as input with External interrupt */
                GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
                HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);
                /* Enable and set Joy EXTI Interrupt to the lowest priority */
                HAL_NVIC_SetPriority((IRQn_Type)(JOY_IRQn[joykey]), 0x0F, 0x00);
                HAL_NVIC_EnableIRQ((IRQn_Type)(JOY_IRQn[joykey]));
            }
        }

        return HAL_OK;
    }

    /**
    * @brief  Returns the current joystick status.
    * @retval Code of the joystick key pressed
    *          This code can be one of the following values:
    *            @arg  JOY_NONE
    *            @arg  JOY_SEL
    *            @arg  JOY_DOWN
    *            @arg  JOY_LEFT
    *            @arg  JOY_RIGHT
    *            @arg  JOY_UP
    */
    __weak JOYState_TypeDef BSP_JOY_GetState(void)
    {
        JOYState_TypeDef joykey;

        for (int joykey_id = JOY_SEL; joykey_id < (JOY_SEL + JOYn) ; joykey_id++)
        {
            joykey = static_cast<JOYState_TypeDef>(joykey_id);
            if (HAL_GPIO_ReadPin(JOY_PORT[joykey], JOY_PIN[joykey]) == GPIO_PIN_SET)
            {
                /* Return Code Joystick key pressed */
                return joykey;
            }
        }

        /* No Joystick key pressed */
        return JOY_NONE;
    }
}
