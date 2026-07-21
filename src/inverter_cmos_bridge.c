/**
 * @file inverter_cmos_bridge.c
 * @brief CMOS ↔ Inverter Modbus bridge.
 *
 * Two threads are started by inverter_cmos_bridge_start():
 *
 *  1. cmos_sub_thread  – runs cmos_sub_spin_ctx() (blocking).
 *       The HMI only sends cmd + value (no uid/addr).  Each HMI command has
 *       its own on_xxx_cmd handler (e.g. on_output_voltage_set_cmd) that owns
 *       the register address, converts value to uint16_t, and delegates the
 *       shared validate/enqueue (or validate/read) flow to on_write() /
 *       on_read().
 *
 *  2. cmos_pub_poll_thread – calls cmos_pub_poll() every 10 ms.
 *       Keeps the read-response publisher alive for new subscriber connections.
 *
 * cmos_sub_spin_ctx() cannot be stopped gracefully (it loops on while(1)).
 * pthread_cancel() is used; epoll_wait() is a POSIX cancellation point so the
 * thread exits cleanly, and the cleanup handler calls cmos_sub_destroy().
 */

#include "inverter_cmos_bridge.h"
#include "inverter_module.h"
#include "cmos.h"
#include "cmos_util.h"
#include "device_register_map.h"
#include "inverter/inverter_map.h"
#include "log.h"

#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/* ── CMOS connection constants ────────────────────────────────────────── */
#define BRIDGE_MASTER_IP        "127.0.0.1"
#define BRIDGE_MASTER_PORT      10000
#define BRIDGE_PUB_PORT         13000           /* listen port for read responses  */
#define BRIDGE_NODE_NAME        "inverter_node"  /* node name for master registration */
#define BRIDGE_SUB_HMI_TOPIC    "control_output"   /* topic for write/read requests   */
#define BRIDGE_PUB_TOPIC        "inverter"       /* topic for read responses        */
#define BRIDGE_PUB_POLL_US      500000u           /* 500 ms poll interval             */

/* this project currently only wires up Inverter#1, so uid is fixed */
#define INVERTER_BRIDGE_DEFAULT_UID  1u

/* ── Module-level state ───────────────────────────────────────────────── */
static pthread_t        g_sub_thread;
static pthread_t        g_pub_poll_thread;
static volatile int     g_bridge_running = 0;

/* ── Register access core (shared by every on_xxx_cmd handler) ───────── */

/**
 * @brief Validate and enqueue a single-register write.
 *
 * Looks up the profile for uid, checks that addr is mapped and not
 * ACCESS_RO, then pushes the write command via inverter_cmd_push().
 *
 * Every on_xxx_cmd handler should call this instead of repeating the
 * lookup/validate/enqueue sequence itself.  Must stay non-blocking.
 *
 * @return 0 on success, -1 on validation failure or queue-full.
 */
static int on_write(uint8_t uid, uint16_t addr, uint16_t val)
{
    const device_map_profile_t *profile = inverter_find_profile_by_uid(uid);
    if (!profile) {
        LOG_WARNING("[CMOS Bridge] write: unknown uid=%u", uid);
        return -1;
    }

    const device_register_mapping_t *entry = device_find_slot(profile, addr);
    if (!entry) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return -1;
    }

    if (entry->access == ACCESS_RO) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X is ACCESS_RO (uid=%u) "
                    "– rejected", addr, uid);
        return -1;
    }

    if (inverter_cmd_push(uid, addr, &val, 1, INVERTER_WRITE_MODE_AUTO) != 0) {
        LOG_ERROR("[CMOS Bridge] write: command queue full for uid=%u "
                  "addr=0x%04X", uid, addr);
        return -1;
    }

    LOG_DEBUG("[CMOS Bridge] write queued: uid=%u addr=0x%04X val=%u",
              uid, addr, val);
    return 0;
}

/**
 * @brief Validate and read a single register from the cached pool.
 *
 * Looks up the profile for uid, checks that addr is mapped and not
 * ACCESS_WO, then reads the cached pool value (updated continuously by
 * process_callback) into *out_val.
 *
 * Every on_xxx_cmd handler should call this instead of repeating the
 * lookup/validate/read sequence itself.  Must stay non-blocking.
 * Publishing the result (if needed) is left to the caller, since the
 * response topic/key format may differ per command.
 *
 * @return 0 on success (*out_val filled), -1 on validation or read failure.
 */
static int on_read(uint8_t uid, uint16_t addr, uint16_t *out_val)
{
    const device_map_profile_t *profile = inverter_find_profile_by_uid(uid);
    if (!profile) {
        LOG_WARNING("[CMOS Bridge] read: unknown uid=%u", uid);
        return -1;
    }

    const device_register_mapping_t *entry = device_find_slot(profile, addr);
    if (!entry) {
        LOG_WARNING("[CMOS Bridge] read: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return -1;
    }

    if (entry->access == ACCESS_WO) {
        LOG_WARNING("[CMOS Bridge] read: addr 0x%04X is ACCESS_WO (uid=%u) "
                    "– rejected", addr, uid);
        return -1;
    }

    if (!pool_read_register(entry->pool_address, out_val)) {
        LOG_ERROR("[CMOS Bridge] read: pool_read failed pool_addr=0x%04X "
                  "(uid=%u addr=0x%04X)", entry->pool_address, uid, addr);
        return -1;
    }

    LOG_DEBUG("[CMOS Bridge] read uid=%u addr=0x%04X pool[0x%04X]=%u",
              uid, addr, entry->pool_address, *out_val);
    return 0;
}

/* ── CMOS callbacks ───────────────────────────────────────────────────── */

static void on_main_pump_duty_cmd(const char *topic, const char *value)
{
    (void)topic;
    uint16_t frequency_register = dev_operation_cmd_reg;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);

    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",frequency_register, value);

    on_write(INVERTER_BRIDGE_DEFAULT_UID, frequency_register, val);
}

static void on_operation_cmd(const char *topic, const char *value)
{
    (void)topic;
    uint16_t addr = int_operation_cmd_reg;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);

    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",addr, value);

    on_write(INVERTER_BRIDGE_DEFAULT_UID, addr, val);
}

static void on_frequency_write_cmd(const char *topic, const char *value)
{
    (void)topic;
    uint16_t addr = int_frequency_write_cmd_reg;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);
    
    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",addr, value);

    on_write(INVERTER_BRIDGE_DEFAULT_UID, addr, val);
}

static void on_fault_control_cmd(const char *topic, const char *value)
{
    (void)topic;
    uint16_t addr = int_fault_control_cmd_reg;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);
    
    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",addr, value);

    on_write(INVERTER_BRIDGE_DEFAULT_UID, addr, val);
}

/* ── Thread cleanup handler ───────────────────────────────────────────── */

/**
 * @brief pthread cleanup handler – destroys the subscriber context on cancel.
 */
static void cleanup_sub_ctx(void *arg)
{
    cmos_sub_ctx_t *ctx = (cmos_sub_ctx_t *)arg;
    cmos_sub_destroy(ctx);
    LOG_INFO("[CMOS Bridge] subscriber context destroyed.");
}

/* ── Thread functions ─────────────────────────────────────────────────── */

/**
 * @brief CMOS subscriber thread.
 *
 * Subscribes to BRIDGE_SUB_HMI_TOPIC once per HMI command (type="command",
 * key=<cmd name>), each routed to its own on_xxx_cmd handler, then enters
 * cmos_sub_spin_ctx() which blocks indefinitely.  The thread is stopped via
 * pthread_cancel(); epoll_wait() inside spin_ctx is a cancellation point so
 * the thread exits cleanly.
 */
static void *cmos_sub_thread(void *arg)
{
    (void)arg;

    cmos_sub_ctx_t *ctx = cmos_sub_create(BRIDGE_MASTER_IP,
                                           BRIDGE_MASTER_PORT,
                                           "inverter_sub");
    if (!ctx) {
        LOG_ERROR("[CMOS Bridge] failed to create subscriber context.");
        return NULL;
    }

    pthread_cleanup_push(cleanup_sub_ctx, ctx);

    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "inv_operation_commands", on_operation_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "inv_frequency_write_commands", on_frequency_write_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "inv_fault_Control_commands", on_fault_control_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "ctrl", "main_pump", on_main_pump_duty_cmd);

    LOG_INFO("[CMOS Bridge] subscriber thread ready, "
             "topic='%s' (write + read).", BRIDGE_SUB_HMI_TOPIC);

    cmos_sub_spin_ctx(ctx);  /* blocks – exited only by pthread_cancel() */

    pthread_cleanup_pop(1);
    return NULL;
}

/**
 * @brief Read one pool register and publish its value to BRIDGE_PUB_TOPIC.
 *
 * Converts the cached uint16_t pool value to a decimal string and forwards
 * it to cmos_publish().  Intended for periodic push from cmos_pub_poll_thread.
 *
 * @param cfg   Module configuration (used for connection-state string).
 * @param type  CMOS message type field.
 * @param key   CMOS message key field (e.g. register address string).
 * @param pool_address  Absolute index into internal_pool[].
 */
static void publish_pool_register(const module_config_t *cfg,
                                  const char *type,
                                  const char *key,
                                  uint16_t    pool_address)
{
    uint16_t val = 0;

    const char *state = (cfg->connection_state == CONNECTION_CONNECTED)
                        ? "Alive"
                        : "Disconnect";

    if (!pool_read_register(pool_address, &val)) {
        LOG_WARNING("[CMOS Bridge] publish_pool_register: "
                    "pool_address 0x%04X out of range.", pool_address);
        return;
    }

    char val_str[8];
    snprintf(val_str, sizeof(val_str), "%u", val);

    cmos_publish(state, type, key, val_str);
}

/**
 * @brief Publisher poll thread.
 *
 * cmos_pub_poll() is non-blocking (epoll_wait timeout=0) so calling it at
 * 10 ms intervals is sufficient to accept new read-response subscribers.
 * Exits when g_bridge_running is cleared by inverter_cmos_bridge_stop().
 */
static void *cmos_pub_poll_thread(void *arg)
{
    module_config_t *inverters = (module_config_t *)arg;

    LOG_INFO("[CMOS Bridge] publisher poll thread started, "
             "resp topic='%s'.", BRIDGE_PUB_TOPIC);

    while (g_bridge_running) {
        cmos_pub_poll();

        for (int i = 0; i < global_config.inverter_count; i++) {
            publish_pool_register(&inverters[i], NULL, "fault_warning_code", int_fault_warning_code_reg);
            publish_pool_register(&inverters[i], NULL, "inverter_operating_status", int_operation_status_reg);
            publish_pool_register(&inverters[i], NULL, "frequency_read_command", int_frequency_read_cmd_reg);
            publish_pool_register(&inverters[i], NULL, "output_frequency", int_out_frequency_reg);
            publish_pool_register(&inverters[i], NULL, "output_current", int_out_current_reg);
            publish_pool_register(&inverters[i], NULL, "dc_bus_voltage", int_dc_bus_voltage_reg);
            publish_pool_register(&inverters[i], NULL, "motor_actual_speed", int_motor_actual_speed_reg);
        }

        usleep(BRIDGE_PUB_POLL_US);
    }

    return NULL;
}

/* ── Public API ───────────────────────────────────────────────────────── */

int inverter_cmos_bridge_start(void)
{
    if (cmos_pub_init(BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT,
                      BRIDGE_NODE_NAME, BRIDGE_PUB_TOPIC,
                      BRIDGE_PUB_PORT) != 0) {
        LOG_ERROR("[CMOS Bridge] publisher init failed "
                  "(master=%s:%d port=%d).",
                  BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT, BRIDGE_PUB_PORT);
        return -1;
    }

    g_bridge_running = 1;

    if (pthread_create(&g_pub_poll_thread, NULL,
                       cmos_pub_poll_thread, global_config.inverter) != 0) {
        LOG_ERROR("[CMOS Bridge] failed to start publisher poll thread.");
        g_bridge_running = 0;
        cmos_pub_close();
        return -1;
    }

    if (pthread_create(&g_sub_thread, NULL, cmos_sub_thread, NULL) != 0) {
        LOG_ERROR("[CMOS Bridge] failed to start subscriber thread.");
        g_bridge_running = 0;
        pthread_join(g_pub_poll_thread, NULL);
        cmos_pub_close();
        return -1;
    }

    LOG_INFO("[CMOS Bridge] started (pub port=%d).", BRIDGE_PUB_PORT);
    return 0;
}

void inverter_cmos_bridge_stop(void)
{
    g_bridge_running = 0;

    pthread_cancel(g_sub_thread);
    pthread_join(g_sub_thread, NULL);

    pthread_join(g_pub_poll_thread, NULL);

    cmos_pub_close();

    LOG_INFO("[CMOS Bridge] stopped.");
}
