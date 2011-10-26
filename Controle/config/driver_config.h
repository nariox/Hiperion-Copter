/*
 * driver_config.h
 *
 *  Created on: 25/10/2011
 *      Author: Danilo Luvizotto
 */

#ifndef DRIVER_CONFIG_H_
#define DRIVER_CONFIG_H_

//#ifdef __USE_CMSIS
#include "LPC11xx.h"
//#endif

#define CONFIG_ENABLE_DRIVER_TIMER16                    1
#define CONFIG_TIMER16_DEFAULT_TIMER16_0_IRQHANDLER     0

#define CONFIG_ENABLE_DRIVER_I2C						1
#define CONFIG_I2C_DEFAULT_I2C_IRQHANDLER				1

#define CONFIG_ENABLE_DRIVER_GPIO                       1

 /* DRIVER_CONFIG_H_ */
#endif
