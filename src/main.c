/**
 * @file main.c
 * @brief Entry point for the loop_charger_inverter daemon.
 *
 * Responsibilities
 * ────────────────
 *  1. Parse command-line arguments.
 *  2. Set log level.
 *  3. Load JSON configuration.
 *  4. Initialise shared subsystems (register pool).
 *  5. Start device modules (Inverter, Modbus RTU).
 *  6. Register alarm contexts.
 *  7. Start alarm bridge.
 *  8. Wait for SIGTERM / SIGINT, then shut down cleanly.
 *
 * Adding a new device type
 * ────────────────────────
 *  1. Create include/<name>_module.h and src/<name>_module.c.
 *  2. Add #include and start_<name>_modules() / stop_<name>_modules() calls here.
 *  3. Add config.json entries.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

#include "config_loader.h"
#include "device_register_map.h"
#include "inverter_module.h"
#include "inverter_alarm.h"
#include "inverter_alarm_manager.h"
#include "alarm_bridge.h"
#include "log.h"

/* ── Globals ──────────────────────────────────────────────────────────── */
extern system_config_t global_config;

static volatile sig_atomic_t g_running = 1;

/* ── Forward declarations ─────────────────────────────────────────────── */
static void parse_arguments(int argc, char **argv,
                             char **config_path, log_level_t *log_level);
static void setup_signal_handlers(void);
static void signal_handler(int sig);

/* ── Signal handling ──────────────────────────────────────────────────── */

static void signal_handler(int sig)
{
    LOG_INFO("Received signal %d. Shutting down …", sig);
    g_running = 0;
}

static void setup_signal_handlers(void)
{
    struct sigaction sa = {
        .sa_handler = signal_handler,
        .sa_flags   = 0,
    };
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

/* ── Argument parsing ─────────────────────────────────────────────────── */

/**
 * @brief Parse command-line arguments.
 *
 * @param argc         Argument count.
 * @param argv         Argument vector.
 * @param config_path  Out: path to config JSON.
 * @param log_level    Out: initial log level.
 */
static void parse_arguments(int argc, char **argv,
                             char **config_path, log_level_t *log_level)
{
    int opt;

    while ((opt = getopt(argc, argv, "c:l:h")) != -1) {
        switch (opt) {
        case 'c':
            *config_path = optarg;
            break;
        case 'l':
            *log_level = (log_level_t)atoi(optarg);
            break;
        case 'h':
        default:
            fprintf(stderr,
                    "Usage: %s [-c config.json] [-l log_level]\n"
                    "  -c  Path to JSON configuration file\n"
                    "  -l  Log level: 0=VERBOSE 1=DEBUG 2=INFO 3=WARN 4=ERROR\n",
                    argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}

/* ── Main ─────────────────────────────────────────────────────────────── */

/**
 * @brief Application entry point.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on clean exit, 1 on initialisation failure.
 */
int main(int argc, char **argv)
{
    char        *config_path = "./config/config.json";
    log_level_t  log_level   = LOG_LEVEL_INFO;

    LOG_INFO("***************************************");
    LOG_INFO(" loop_charger_inverter  version: %s",
             LOOP_CHARGER_INVERTER_VERSION);
    LOG_INFO("***************************************");

    parse_arguments(argc, argv, &config_path, &log_level);
    set_log_level(log_level);

    LOG_INFO("Loading configuration: %s", config_path);
    load_json_config(config_path);

    device_register_map_init();

    setup_signal_handlers();

    if (start_inverter_modules(global_config.inverter,
                               global_config.inverter_count) != 0) {
        LOG_ERROR("One or more Inverter modules failed to start.");
    }

    inverter_alarm_register_all();

    if (alarm_bridge_start() != 0) {
        LOG_ERROR("Alarm bridge failed to start.");
    }

    LOG_INFO("All modules started. Waiting for shutdown signal …");

    while (g_running) {
        pause();
    }

    alarm_bridge_stop();
    stop_inverter_modules();
    inverter_alarm_manager_close();

    LOG_INFO("Shutdown complete.");
    return 0;
}
