/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "lidar.h"
#include "lidar_mode_filter.h"

#define lidar_SCALE	1000.0f
#define lidar_MIN	0.12f		/** 0.12m lidar minimum distance */
#define lidar_MAX	3.5f		/** 3.50m lidar maximum distance */

#define atoi(nptr)  strtol((nptr), NULL, 10)
extern uint32_t get_boot_time_us(void);

static char data_buffer[5]; // array for collecting decoded data

static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t lidar_measure_time_interrupt = 0;
static volatile uint32_t lidar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float lidar_raw = 0.0f;  // m

float lidar_mode = 0.0f;
bool lidar_valid = false;				/**< the mode of all lidar measurements */

/**
  * @brief  Triggers the lidar to measure the next value
  *
  * see datasheet for more info
  */
void lidar_trigger(){
	GPIO_SetBits(GPIOE, GPIO_Pin_8);
}

/**
  * @brief  lidar interrupt handler
  */
void TIM4_IRQHandler(void)
{
  /* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM4);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;

    /* Frequency computation */
    Frequency = SystemCoreClock / IC2Value;
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
}

/**
  * @brief  Basic Kalman filter
  */
static void lidar_filter(void)
{
	/* no data for long time */
	if (dt > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = lidar_mode;
	lidar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_lidar_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_lidar_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest lidar data
  *
  * @param  lidar_value_filtered Filtered return value
  * @param  lidar_value_raw Raw return value
  */
bool lidar_read(float* lidar_value_filtered, float* lidar_value_raw)
{
	/* getting new data with only around 10Hz */
	if (new_value) {
		lidar_filter();
		new_value = 0;
		lidar_measure_time = get_boot_time_us();
	}

	/* catch post-filter out of band values */
	if (x_post < lidar_MIN || x_post > lidar_MAX) {
		lidar_valid = false;
	}

	*lidar_value_filtered = x_post;
	*lidar_value_raw = lidar_raw;

	return lidar_valid;
}

/**
 * @brief  Configures the lidar sensor Peripheral.
 */
void lidar_config(void)
{
	valid_data = 0;

  	/* Configure system clocks */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure trigger pin in output pushpull mode */
	GPIO_InitTypeDef GPIOE_InitStructure;
	GPIOE_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIOE_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIOE_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIOE_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOE_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIOE_InitStructure);

	/* Configure PW pin in input floating mode */
	GPIO_InitTypeDef GPIOD_InitStructure;
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIOD_InitStructure);

	/* Timer 4 PWM Input configuration */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

	/* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);


	/* Enable the TIM4 global Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

uint32_t get_lidar_measure_time()
{
    return lidar_measure_time;
}

uint32_t get_lidar_measure_time_interrupt()
{
    return lidar_measure_time_interrupt;
}