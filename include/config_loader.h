/**
 * @file config_loader.h
 * @brief JSON configuration loading for loop_charger_inverter (Inverter-focused).
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "modbus_defines.h"

#define LOOP_CHARGER_INVERTER_VERSION "0.0.1"
#define MAX_INVERTER_COUNT            10

/** Receive buffer size for module I/O. */
#ifndef MODBUS_MAX_BUFFER_SIZE
#define MODBUS_MAX_BUFFER_SIZE        MODBUS_RTU_MAX_ADU_LENGTH
#endif

typedef enum {
    CONNECTION_DISCONNECTED = 0,
    CONNECTION_CONNECTED    = 1,
    CONNECTION_UNKNOWN      = 99
} connection_state_t;

typedef enum {
    MODBUS_FORMAT_RTU,
    MODBUS_FORMAT_TCP,
} modbus_format_t;

/**
 * @brief Per-module runtime configuration.
 *
 * Register pool placement is handled entirely by the per-unit mapping tables
 * in inverter_map.c (absolute pool_address values).  No pool_base field is
 * needed here.
 */
typedef struct {
    char            name[64];
    bool            enabled;
    bool            is_server;
    int             fd;
    int             modbus_uid;
    modbus_format_t format;
    char            path[256];
    char            gpio[16];
    int             baud_rate;
    char            modbus_role[16];
    char            recv_buffer[MODBUS_MAX_BUFFER_SIZE];
    size_t          recv_index;
    connection_state_t  connection_state;
    double              disconnect_timeout;
    uint32_t            rtu_poll_interval_ms;
} module_config_t;

typedef struct {
    module_config_t inverter[MAX_INVERTER_COUNT];
    int             inverter_count;
} system_config_t;

extern system_config_t global_config;
extern char            global_config_path[256];

typedef struct {
    int (*init_callback)(module_config_t *config);
    int (*start_callback)(const module_config_t *config);
    int (*process_callback)(module_config_t *config);
    int (*error_callback)(module_config_t *config, int connection_state);
    int (*msg_callback)(module_config_t *config, uint16_t addr,
                        uint16_t *values, size_t count);
} module_callbacks_t;

void load_json_config(const char *json_path);

#endif /* CONFIG_LOADER_H */
