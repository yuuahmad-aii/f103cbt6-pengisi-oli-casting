/*
 * File: HCSR04_cfg.c
 * Driver Name: [[ HC-SR04 Ultrasonic Sensor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
	{
		// 3 buah sensor yang digunakan  
		{GPIOB, GPIO_PIN_3, TIM2, TIM_CHANNEL_1, 48},
		{GPIOB, GPIO_PIN_5, TIM3, TIM_CHANNEL_1, 48},
		{GPIOB, GPIO_PIN_7, TIM4, TIM_CHANNEL_1, 48},
};
