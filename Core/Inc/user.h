/*
 * user.h
 *
 *  Created on: Jul 9, 2024
 *      Author: Nate Hunter
 */

#pragma once

#define ERR_LORA (1<<0)
#define ERR_USB (1<<1)
#define INIT_ATTEMPS 10

void USER_Init();
void USER_Loop();
