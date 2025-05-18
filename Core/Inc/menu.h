/**
 * @file menu.h
 * @brief Terminal menu system for LoraLink user interaction
 *
 * This file contains functions and enums for handling terminal-based configuration
 * and program selection via USB CDC interface.
 *
 * @author [Nate Hunter]
 * @date [18.05.2025]
 * @version [1.0]
 */

#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>

/**
 * @brief Enum for internal menu states
 */
typedef enum {
    MENU_IDLE = 0,
    MENU_SETUP_USERNAME,
    MENU_SETUP_PASSWORD,
    MENU_SETUP_FREQ,
    MENU_SETUP_SF,
    MENU_MAIN
} MenuMode;

/**
 * @brief Enum for selectable operating programs
 */
typedef enum {
    PROGRAM_NONE = 0,
    PROGRAM_LORA_TERMINAL,
    PROGRAM_CANSAT_RECEIVER
} ProgramType;

/**
 * @brief Append received input to internal buffer and process full line if complete
 *
 * @param data Pointer to received data
 * @param len Number of bytes
 */
void menu_append_input(uint8_t* data, uint32_t len);

/**
 * @brief Process a complete input line based on current menu state
 *
 * @param line Null-terminated input string
 */
void menu_process_line(const char* line);

/**
 * @brief Print a message and prompt to terminal
 *
 * @param msg Null-terminated string to send
 */
void menu_prompt(const char* msg);

/**
 * @brief Begin interactive first-time setup
 */
void menu_start_setup(void);

/**
 * @brief Show the main menu after setup
 */
void menu_show_main(void);

/**
 * @brief Get selected program after user input
 *
 * @return ProgramType enum value
 */
ProgramType menu_get_selected_program(void);

#endif /* __MENU_H */
