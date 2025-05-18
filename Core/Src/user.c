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
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern uint8_t UserRxBufferFS[];
extern uint8_t UserTxBufferFS[];
char formatedMessage[256];
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t currFlash[4] = { 0, 0, 0, 0 };
USBD_CDC_HandleTypeDef *hcdc;
LoRa_HandleTypeDef lora;
LoRaConfig loraCfg;

/** @brief IMU data structure */
typedef struct {
	uint32_t time;          ///< Milliseconds from start
	int32_t temp;           ///< MS56 temperature (centigrade*10e2)
	uint32_t press;         ///< MS56 pressure (Pa)
	float magData[3];       ///< LIS3 mag (mG)
	float accelData[3];     ///< LSM6 accel (mG)
	float gyroData[3];      ///< LSM6 gyro (mdps)
	int32_t altitude;       ///< Altitude (zero at start, cm)
	float lat;              ///< Latitude from GPS
	float lon;              ///< Longitude from GPS
	uint32_t flags;         ///< Flags (0|0|0|0|Land|ResSys|Eject|Start)
	//*_____64 bytes - frame threshold_____*/
	uint32_t press0;        ///< MS56 pressure at 0 Alt (Pa)
	float vectAbs;          ///< Absolute value of accel vector
	uint32_t wqAdr;         ///< WQ address
} ImuData;

typedef enum {
	INITIAL, USB_CONNECTED, USB_DISCONNECTED, LORA_RECEIVE, LORA_SEND, CONFIG_MODE
} SystemState;

SystemState currentState = INITIAL;
SystemState lastState = -1;

/**
 * @brief Calculate checksum by XOR-ing all bytes in the buffer.
 *
 * @param data Pointer to the buffer.
 * @param length Number of bytes to include in the checksum.
 * @return uint8_t Calculated checksum.
 */
uint8_t calculateChecksum(const uint8_t *data, size_t length) {
	uint8_t crc = 0;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
	}
	return crc;
}

/**
 * @brief Write uint16 or uint32 into buffer in LSBF (little-endian) format.
 */
void writeUint16LSBF(uint8_t *buffer, uint16_t value) {
	buffer[0] = (uint8_t) (value & 0xFF);
	buffer[1] = (uint8_t) ((value >> 8) & 0xFF);
}

void writeUint32LSBF(uint8_t *buffer, uint32_t value) {
	buffer[0] = (uint8_t) (value & 0xFF);
	buffer[1] = (uint8_t) ((value >> 8) & 0xFF);
	buffer[2] = (uint8_t) ((value >> 16) & 0xFF);
	buffer[3] = (uint8_t) ((value >> 24) & 0xFF);
}

/**
 * @brief Pack IMU data into a binary packet for radio transmission.
 *
 * @param data Pointer to ImuData structure.
 * @param teamId 16-bit team identifier.
 * @param outBuffer Output buffer to write the 28+ byte packet. Must be at least 28 bytes.
 * @return size_t Total packet length in bytes (minimum 27+1).
 */
size_t packImuDataPacket(const ImuData *data, uint16_t teamId, uint8_t *outBuffer) {
	uint8_t *ptr = outBuffer;

	// Header
	writeUint16LSBF(ptr, 0xAAAA);
	ptr += 2;                   // Start mark
	writeUint16LSBF(ptr, teamId);
	ptr += 2;                   // Team ID
	writeUint32LSBF(ptr, data->time);
	ptr += 4;               // Time
	writeUint16LSBF(ptr, (uint16_t) (data->temp));
	ptr += 2;   // Temperature (int16)
	writeUint32LSBF(ptr, data->press);
	ptr += 4;              // Pressure

	// Accel: convert to int16 (x1000, LSBF)
	for (int i = 0; i < 3; i++) {
		int16_t acc = (int16_t) (data->accelData[i]);
		writeUint16LSBF(ptr, acc);
		ptr += 2;
	}

	// Gyro: convert to int16 (x1, LSBF)
	for (int i = 0; i < 3; i++) {
		int16_t gyro = (int16_t) (data->gyroData[i]);
		writeUint16LSBF(ptr, gyro);
		ptr += 2;
	}

	// CRC for first 26 bytes
	uint8_t crc = calculateChecksum(outBuffer, 26);
	*ptr++ = crc;

	// Optional user-defined data can go here (padding/future use)

	return (size_t) (ptr - outBuffer); // Total length of the packet
}

/**
 * @brief Parse IMU structure into a human-readable string.
 *
 * @param data Pointer to ImuData structure.
 * @param buffer Pointer to destination buffer.
 * @param bufferSize Size of the destination buffer in bytes.
 */
void formatImuData(const ImuData *data, char *buffer, size_t bufferSize) {
	size_t offset = 0;

	offset += snprintf(buffer + offset, bufferSize - offset, "/*******Begin*Frame*******/\n");
	offset += snprintf(buffer + offset, bufferSize - offset, "Time: %lu\n", (unsigned long) data->time);
	offset += snprintf(buffer + offset, bufferSize - offset, "Temp: %.2f\n", data->temp / 100.0f);
	offset += snprintf(buffer + offset, bufferSize - offset, "Pressure: %lu\n", (unsigned long) data->press);
	offset += snprintf(buffer + offset, bufferSize - offset, "Altitude: %ld\n", (long) data->altitude);
	offset += snprintf(buffer + offset, bufferSize - offset, "Flags: 0x%08lX\n", (unsigned long) data->flags);

	offset += snprintf(buffer + offset, bufferSize - offset, "Mag: %.2f %.2f %.2f\n", data->magData[0], data->magData[1],
			data->magData[2]);

	offset += snprintf(buffer + offset, bufferSize - offset, "Accel: %.2f %.2f %.2f\n", data->accelData[0], data->accelData[1],
			data->accelData[2]);

	offset += snprintf(buffer + offset, bufferSize - offset, "Gyro: %.2f %.2f %.2f\n", data->gyroData[0], data->gyroData[1],
			data->gyroData[2]);

	offset += snprintf(buffer + offset, bufferSize - offset, "/********End*Frame********/\n");
}

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
			while (1) {
			}
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
		if (rxLen == 64) {
			ImuData *data = (ImuData*) UserTxBufferFS;
			formatImuData(data, formatedMessage, sizeof formatedMessage);
			CDC_Transmit_FS((uint8_t*) formatedMessage, strlen(formatedMessage));
//			rxLen = packImuDataPacket(data, 0xFAAF, (uint8_t*) formatedMessage);
//			CDC_Transmit_FS((uint8_t*) formatedMessage, rxLen);
		} else
			CDC_Transmit_FS((uint8_t*) UserTxBufferFS, rxLen);
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

