/**
 * @file menu.c
 * @brief Implements terminal menu system and input processing
 */

#include "menu.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

#define INPUT_BUFFER_SIZE 64

static char inputBuffer[INPUT_BUFFER_SIZE];
static uint8_t inputIndex = 0;
static MenuMode currentMode = MENU_IDLE;

/**
 * @brief Append incoming characters to input buffer and process full lines.
 *
 * @param data Pointer to received characters
 * @param len Number of characters received
 */
void menu_append_input(uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        char c = (char)data[i];
        if (c == '\r') continue; // ignore CR

        if (c == '\n') {
            inputBuffer[inputIndex] = '\0';
            menu_process_line(inputBuffer);
            inputIndex = 0;
        } else {
            if (inputIndex < INPUT_BUFFER_SIZE - 1) {
                inputBuffer[inputIndex++] = c;
            }
        }
    }
}

/**
 * @brief Process full input line depending on menu state
 *
 * @param line Zero-terminated input string
 */
void menu_process_line(const char* line) {
    switch (currentMode) {
    case MENU_SETUP_USERNAME:
        strncpy(config.username, line, sizeof(config.username));
        menu_prompt("Enter password:");
        currentMode = MENU_SETUP_PASSWORD;
        break;

    case MENU_SETUP_PASSWORD:
        strncpy(config.password, line, sizeof(config.password));
        menu_prompt("Set LoRa frequency [Hz]:");
        currentMode = MENU_SETUP_FREQ;
        break;

    case MENU_SETUP_FREQ:
        config.lora_freq = (uint32_t)atoi(line);
        menu_prompt("Set spreading factor [7-12]:");
        currentMode = MENU_SETUP_SF;
        break;

    case MENU_SETUP_SF:
        config.lora_sf = (uint8_t)atoi(line);
        config.is_initialized = 1;
        config_save(&config);
        menu_prompt("Setup complete. Rebooting...");
        HAL_NVIC_SystemReset();
        break;

    case MENU_MAIN:
        if (strcmp(line, "1") == 0) {
            selectedProgram = PROGRAM_LORA_TERMINAL;
        } else if (strcmp(line, "2") == 0) {
            selectedProgram = PROGRAM_CANSAT_RECEIVER;
        } else {
            menu_prompt("Invalid choice. Try again:");
            menu_show_main();
        }
        break;

    default:
        break;
    }
}

/**
 * @brief Display a message in the USB terminal.
 *
 * @param msg String to send
 */
void menu_prompt(const char* msg) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%s\r\n> ", msg);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

/**
 * @brief Start first-time setup process
 */
void menu_start_setup(void) {
    currentMode = MENU_SETUP_USERNAME;
    menu_prompt("Welcome to Osprey Chimera!\nFirst-time setup required.\nEnter username:");
}

/**
 * @brief Show main menu
 */
void menu_show_main(void) {
    currentMode = MENU_MAIN;
    menu_prompt("==== MAIN MENU ===="
                "\n1. LoRa Terminal"
                "\n2. CanSat Receiver");
}

/**
 * @brief Return currently selected program
 *
 * @return ProgramType enum
 */
ProgramType menu_get_selected_program(void) {
    return selectedProgram;
}
