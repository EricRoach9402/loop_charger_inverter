/**
 * @file inverter_cmos_bridge.c
 * @brief CMOS ↔ Inverter Modbus RTU bridge.
 *
 * Two threads are started by inverter_cmos_bridge_start():
 *
 *  1. cmos_sub_thread  – runs cmos_sub_spin_ctx() (blocking).
 *       • on_write_cmd: parse → access-check → inverter_cmd_push()
 *       • on_read_cmd:  parse → pool_read_register()
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
#define BRIDGE_PUB_PORT         12001       /* listen port for read responses  */
#define BRIDGE_NODE_NAME        "INV_NODE"  /* node name for master registration */
#define BRIDGE_SUB_TOPIC        "INV_SUB"   /* topic for write/read requests   */
#define BRIDGE_PUB_TOPIC        "INV"       /* topic for read responses        */
#define BRIDGE_PUB_POLL_US      10000u      /* 10 ms poll interval             */

/* ── Module-level state ───────────────────────────────────────────────── */
static pthread_t        g_sub_thread;
static pthread_t        g_pub_poll_thread;
static volatile int     g_bridge_running = 0;

/* ── Internal helpers ─────────────────────────────────────────────────── */

/**
 * @brief Extract a value from a comma-delimited key=value string.
 *
 * Searches for "key=" token (delimited by ',' or end-of-string) and copies
 * the corresponding value into dest.
 *
 * Example src: "uid=1,addr=0x0100,val=220"
 *   parse_token(src, "addr", out, size)  →  out = "0x0100"
 *
 * @return 0 on success, -1 if key not found.
 */
static int parse_token(const char *src, const char *key,
                       char *dest, size_t dest_size)
{
    char   buf[256];
    char   search[64];
    char  *p;
    size_t key_len;
    size_t i;

    strncpy(buf, src, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    snprintf(search, sizeof(search), "%s=", key);
    key_len = strlen(search);

    p = buf;
    while (*p) {
        if (strncmp(p, search, key_len) == 0) {
            p += key_len;
            i = 0;
            while (*p && *p != ',' && i < dest_size - 1) {
                dest[i++] = *p++;
            }
            dest[i] = '\0';
            return 0;
        }
        while (*p && *p != ',') {
            p++;
        }
        if (*p == ',') {
            p++;
        }
    }

    return -1;
}

/* ── CMOS callbacks ───────────────────────────────────────────────────── */

/**
 * @brief CMOS callback for write commands.
 *
 * Called by cmos_sub_spin_ctx() when a message arrives with
 * type=modbus key=write.  The callback receives only the value field:
 *   "uid=<n>,addr=<hex>,val=<dec>"
 *
 * Validates register access (rejects ACCESS_RO registers) and enqueues the
 * write command via inverter_cmd_push().  Must stay non-blocking.
 */
static void on_write_cmd(const char *topic, const char *value)
{
    char     uid_str[16];
    char     addr_str[16];
    char     val_str[16];
    uint8_t  uid;
    uint16_t addr;
    uint16_t val;

    (void)topic;

    if (parse_token(value, "uid",  uid_str,  sizeof(uid_str))  != 0 ||
        parse_token(value, "addr", addr_str, sizeof(addr_str)) != 0 ||
        parse_token(value, "val",  val_str,  sizeof(val_str))  != 0) {
        LOG_WARNING("[CMOS Bridge] write: malformed value – expected "
                    "uid=<n>,addr=<hex>,val=<dec>  got: '%s'", value);
        return;
    }

    uid  = (uint8_t)atoi(uid_str);
    addr = (uint16_t)strtoul(addr_str, NULL, 0);
    val  = (uint16_t)strtoul(val_str,  NULL, 0);

    const device_map_profile_t *profile = inverter_find_profile_by_uid(uid);
    if (!profile) {
        LOG_WARNING("[CMOS Bridge] write: unknown uid=%u", uid);
        return;
    }

    const device_register_mapping_t *entry = device_find_slot(profile, addr);
    if (!entry) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return;
    }

    if (entry->access == ACCESS_RO) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X is ACCESS_RO (uid=%u) "
                    "– rejected", addr, uid);
        return;
    }

    if (inverter_cmd_push(uid, addr, &val, 1, INVERTER_WRITE_MODE_AUTO) != 0) {
        LOG_ERROR("[CMOS Bridge] write: command queue full for uid=%u "
                  "addr=0x%04X", uid, addr);
    }

    LOG_DEBUG("[CMOS Bridge] write queued: uid=%u addr=0x%04X val=%u",
              uid, addr, val);
}

/**
 * @brief CMOS callback for read commands.
 *
 * Called by cmos_sub_spin_ctx() when a message arrives with
 * type=modbus key=read.  The callback receives only the value field:
 *   "uid=<n>,addr=<hex>"
 *
 * Reads the cached pool value (updated continuously by process_callback) and
 * logs the result.  Must stay non-blocking.
 */
static void on_read_cmd(const char *topic, const char *value)
{
    char     uid_str[16];
    char     addr_str[16];
    uint8_t  uid;
    uint16_t addr;
    uint16_t pool_val = 0;

    (void)topic;

    if (parse_token(value, "uid",  uid_str,  sizeof(uid_str))  != 0 ||
        parse_token(value, "addr", addr_str, sizeof(addr_str)) != 0) {
        LOG_WARNING("[CMOS Bridge] read: malformed value – expected "
                    "uid=<n>,addr=<hex>  got: '%s'", value);
        return;
    }

    uid  = (uint8_t)atoi(uid_str);
    addr = (uint16_t)strtoul(addr_str, NULL, 0);

    const device_map_profile_t *profile = inverter_find_profile_by_uid(uid);
    if (!profile) {
        LOG_WARNING("[CMOS Bridge] read: unknown uid=%u", uid);
        return;
    }

    const device_register_mapping_t *entry = device_find_slot(profile, addr);
    if (!entry) {
        LOG_WARNING("[CMOS Bridge] read: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return;
    }

    if (entry->access == ACCESS_WO) {
        LOG_WARNING("[CMOS Bridge] read: addr 0x%04X is ACCESS_WO (uid=%u) "
                    "– rejected", addr, uid);
        return;
    }

    if (!pool_read_register(entry->pool_address, &pool_val)) {
        LOG_ERROR("[CMOS Bridge] read: pool_read failed pool_addr=0x%04X "
                  "(uid=%u addr=0x%04X)", entry->pool_address, uid, addr);
        return;
    }

    /*
     * Publish result to BRIDGE_PUB_TOPIC.
     * Uncomment when read-response publishing is required:
     *
     *   char result_str[16];
     *   snprintf(result_str, sizeof(result_str), "%u", pool_val);
     *   cmos_publish(NULL, "modbus_resp", addr_str, result_str);
     */

    LOG_DEBUG("[CMOS Bridge] read uid=%u addr=0x%04X pool[0x%04X]=%u",
              uid, addr, entry->pool_address, pool_val);
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
 * Subscribes to BRIDGE_SUB_TOPIC with two filter slots (write / read) and
 * enters cmos_sub_spin_ctx() which blocks indefinitely.  The thread is
 * stopped via pthread_cancel(); epoll_wait() inside spin_ctx is a
 * cancellation point so the thread exits cleanly.
 */
static void *cmos_sub_thread(void *arg)
{
    (void)arg;

    cmos_sub_ctx_t *ctx = cmos_sub_create(BRIDGE_MASTER_IP,
                                           BRIDGE_MASTER_PORT,
                                           "inv_sub");
    if (!ctx) {
        LOG_ERROR("[CMOS Bridge] failed to create subscriber context.");
        return NULL;
    }

    pthread_cleanup_push(cleanup_sub_ctx, ctx);

    cmos_sub_add(ctx, BRIDGE_SUB_TOPIC, NULL, "modbus", "write", on_write_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_TOPIC, NULL, "modbus", "read",  on_read_cmd);

    LOG_INFO("[CMOS Bridge] subscriber thread ready, "
             "topic='%s' (write + read).", BRIDGE_SUB_TOPIC);

    cmos_sub_spin_ctx(ctx);  /* blocks – exited only by pthread_cancel() */

    pthread_cleanup_pop(1);
    return NULL;
}

/**
 * @brief Read one pool register and publish its value to BRIDGE_PUB_TOPIC.
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
            publish_pool_register(&inverters[i], "ac_output_voltage", NULL, 0x0003);
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
                       cmos_pub_poll_thread,
                       global_config.inverter) != 0) {
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
