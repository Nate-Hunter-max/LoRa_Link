/*
 * user.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Nate Hunter
 */
#include "user.h"
#include "main.h"
#include "LoRa.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern uint8_t UserRxBufferFS[];
extern uint8_t UserTxBufferFS[];
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t currFlash[4] = { 0, 0, 0, 0 };
USBD_CDC_HandleTypeDef *hcdc;
LoRa_HandleTypeDef lora;
LoRaConfig loraCfg;

typedef enum {
	INITIAL,
	USB_CONNECTED,
	USB_DISCONNECTED,
	LORA_RECEIVE,
	LORA_SEND,
	CONFIG_MODE
} SystemState;

SystemState currentState = INITIAL;
SystemState lastState = -1;

/*-----------------------------------------------STATE_MACHINE----------------------------------------------*/

static inline void init_state() {
	static uint8_t i;
	static uint8_t errCode;
	static uint32_t currTick;

	loraCfg.frequency = 433;
	loraCfg.bandwidth = 0x08;
	loraCfg.spreadingFactor = 7;
	loraCfg.codingRate = 0b001;
	loraCfg.headerMode = 0;
	loraCfg.crcEnabled = 1;
	loraCfg.lowDataRateOptimize = 0;
	loraCfg.preambleLength = 6;
	loraCfg.payloadLength = 180;
	loraCfg.txAddr = 255;
	loraCfg.rxAddr = 0;
	loraCfg.txPower = 0x03;
	lora.config = loraCfg;
	lora.spi = &hspi2;
	lora.NSS_Port = LORA_NSS_GPIO_Port;
	lora.NSS_Pin = LORA_NSS_Pin;

	if (currentState != lastState) {
		lastState = currentState;
		errCode = ERR_LORA | ERR_USB;
		currTick = 0;
		i = INIT_ATTEMPS;
		htim2.Instance->CCR3 = htim2.Init.Period;
		htim2.Instance->CCR2 = htim2.Init.Period;
	}

	if (HAL_GetTick() - currTick >= 150) {
		currTick = HAL_GetTick();
		if (errCode & ERR_LORA && LoRa_Init(&lora))
			errCode &= ~ERR_LORA;
		if (errCode & ERR_USB && (hUsbDeviceFS.ep_in[CDC_IN_EP & 0xFU].is_used))
			errCode &= ~ERR_USB;
		i--;
	}

	if (!errCode) {
		hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
		htim2.Instance->CCR1 = htim2.Init.Period;
		htim2.Instance->CCR3 = 0;
		htim2.Instance->CCR2 = 0;
		currentState = USB_CONNECTED;
	}

	if (i == 0) {
		htim2.Instance->CCR3 = 0;
		htim2.Instance->CCR2 = 0;
		if (errCode & ERR_LORA) {
			htim2.Instance->CCR4 = htim2.Init.Period;
			while (1){}
		} else if (errCode & ERR_USB) {
			currFlash[0] = 1;
			HAL_TIM_Base_Start_IT(&htim4);
			currentState = USB_DISCONNECTED;
		}
	}
}

static inline void usb_connected_state() {
	static uint8_t rxLen;
	if (currentState != lastState) {
		lastState = currentState;
	}

	if (hcdc->RxLength > 0) {
		htim2.Instance->CCR3 = htim2.Init.Period;
		LoRa_Transmit(&lora, UserRxBufferFS, hcdc->RxLength);
		hcdc->RxLength = 0;
		htim2.Instance->CCR3 = 0;
	}

	if (LoRa_Receive(&lora, UserTxBufferFS, &rxLen)) {
		htim2.Instance->CCR2 = htim2.Init.Period;
		CDC_Transmit_FS(UserTxBufferFS, rxLen);
		htim2.Instance->CCR2 = 0;
	}
}

static inline void usb_disconnected_state() {
	if (currentState != lastState) {
		lastState = currentState;
	}
}

static inline void lora_receive_state() {
	if (currentState != lastState) {
		lastState = currentState;
	}
}

static inline void lora_send_state() {
	if (currentState != lastState) {
		lastState = currentState;
	}
}

static inline void config_mode_state() {
	if (currentState != lastState) {
		lastState = currentState;
	}
}

/*-----------------------------------------------ADDITIONAL_LOGIC----------------------------------------------*/

void FadeLedPWM(uint32_t *pwm) {
	static uint8_t direction = 1;
	if (direction) {
		(*pwm)++;
		if (*pwm >= htim2.Init.Period)
			direction = 0;
	} else {
		(*pwm)--;
		if (*pwm == 0)
			direction = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint32_t pwmValue = 0;

	if (htim->Instance == TIM4) {
		if (currFlash[0]) {
			htim2.Instance->CCR1 = pwmValue;
			FadeLedPWM(&pwmValue);
		}
		if (currFlash[1]) {
			htim2.Instance->CCR2 = pwmValue;
			FadeLedPWM(&pwmValue);
		}
		if (currFlash[2]) {
			htim2.Instance->CCR3 = pwmValue;
			FadeLedPWM(&pwmValue);
		}
		if (currFlash[3]) {
			htim2.Instance->CCR4 = pwmValue;
			FadeLedPWM(&pwmValue);
		}
	}
}

/*-----------------------------------------------USER_FUNCS----------------------------------------------*/

void USER_Init() {
	htim2.Instance->CCR1 = 0;
	htim2.Instance->CCR2 = 0;
	htim2.Instance->CCR3 = 0;
	htim2.Instance->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void USER_Loop() {
	switch (currentState) {
	case INITIAL:
		init_state();
		break;
	case USB_CONNECTED:
		usb_connected_state();
		break;
	case USB_DISCONNECTED:
		usb_disconnected_state();
		break;
	case LORA_RECEIVE:
		lora_receive_state();
		break;
	case LORA_SEND:
		lora_send_state();
		break;
	case CONFIG_MODE:
		config_mode_state();
		break;
	default:
		break;
	}
}

