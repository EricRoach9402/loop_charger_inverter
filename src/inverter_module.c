/**
 * @file inverter_module.c
 * @brief Inverter polling module – Modbus RTU client implementation.
 *
 * Callback design
 * ───────────────
 *  inverter_rtu_init_callback    – open and configure the serial port.
 *  inverter_rtu_process_callback – drain write queue, then segment-read all
 *                                  registers → write to pool.
 *  inverter_rtu_error_callback   – close serial port and schedule reconnect.
 *  inverter_msg_callback         – execute FC06 / FC16 write on an Inverter unit.
 *
 * CMOS bridge integration
 * ───────────────────────
 *  A dedicated CMOS subscriber thread (inverter_cmos_bridge.c) receives write /
 *  read commands and enqueues writes via inverter_cmd_push().
 *  process_callback drains the queue before each FC03 scan.
 *
 * Thread model
 * ────────────
 *  One pthread per enabled Inverter unit.  Each thread calls:
 *    init_callback  → success  → loop: process_callback
 *                   → failure  → retry after INVERTER_RECONNECT_DELAY_MS
 *
 * Profile selection
 * ─────────────────
 *  Each Inverter unit's register profile is looked up by modbus_uid at startup.
 *  Profiles are registered in devices/inverter/inverter_map.c; order in
 *  config.json has no effect on which profile is selected.
 *
 * Adding a new Inverter unit
 * ──────────────────────────
 *  1. Add table + profile in devices/inverter/inverter_map.c.
 *  2. Declare the profile extern in devices/inverter/inverter_map.h.
 */

#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

#include "inverter_module.h"
#include "inverter_cmos_bridge.h"
#include "device_register_map.h"
#include "modbus_rtu_client.h"
#include "inverter/inverter_map.h"
#include "log.h"

/* ── Tunable constants ────────────────────────────────────────────────── */
#define INVERTER_RECONNECT_DELAY_MS         5000u
#define INVERTER_COMM_FAIL_THRESHOLD        5
#define INVERTER_INTER_SEGMENT_DELAY_US     40000u   /* 40 ms between FC03 frames */
#define INVERTER_SHUTDOWN_CHECK_INTERVAL_MS 100u     /* granularity for interruptible sleeps */

/* ── Command queue constants ──────────────────────────────────────────── */
#define INVERTER_CMD_QUEUE_CAPACITY         16u

/* ── Write command entry ──────────────────────────────────────────────── */

typedef struct {
    uint16_t              addr;
    uint16_t              values[MODBUS_MAX_WRITE_REGISTERS];
    uint16_t              count;
    inverter_write_mode_t mode;
} inverter_write_cmd_t;

/* ── Per-unit command queue ───────────────────────────────────────────── */
typedef struct {
    inverter_write_cmd_t entries[INVERTER_CMD_QUEUE_CAPACITY];
    unsigned int         head;
    unsigned int         tail;
    unsigned int         count;
    pthread_mutex_t      lock;
} inverter_cmd_queue_t;

/* ── Per-unit runtime state ───────────────────────────────────────────── */
typedef struct {
    module_config_t            *cfg;
    const device_map_profile_t *profile;
    mb_rtu_client_ctx_t         rtu_ctx;
    module_callbacks_t          callbacks;
    pthread_t                   thread;
    volatile sig_atomic_t       running;
    int                         comm_fail_count;
    inverter_cmd_queue_t        cmd_queue;
} inverter_unit_t;

static inverter_unit_t  inverter_units[MAX_INVERTER_COUNT];
static int              inverter_unit_count = 0;

/* ── Queue helpers (static) ───────────────────────────────────────────── */

static void queue_init(inverter_cmd_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
}

static void queue_destroy(inverter_cmd_queue_t *q)
{
    pthread_mutex_destroy(&q->lock);
}

/**
 * @brief Push one write command onto the tail of the queue.
 * @return 0 on success, -1 if the queue is full or count is out of range.
 */
static int queue_push(inverter_cmd_queue_t *q, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode)
{
    if (count == 0 || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    pthread_mutex_lock(&q->lock);

    if (q->count >= INVERTER_CMD_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    inverter_write_cmd_t *entry = &q->entries[q->tail];
    entry->addr  = addr;
    entry->count = count;
    entry->mode  = mode;
    memcpy(entry->values, values, count * sizeof(uint16_t));

    q->tail  = (q->tail + 1u) % INVERTER_CMD_QUEUE_CAPACITY;
    q->count++;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/**
 * @brief Pop one write command from the head of the queue.
 * @return 0 on success, -1 if the queue is empty.
 */
static int queue_pop(inverter_cmd_queue_t *q, inverter_write_cmd_t *out)
{
    pthread_mutex_lock(&q->lock);

    if (q->count == 0) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    *out    = q->entries[q->head];
    q->head = (q->head + 1u) % INVERTER_CMD_QUEUE_CAPACITY;
    q->count--;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/* ── Internal helpers ─────────────────────────────────────────────────── */

static inverter_unit_t *inverter_unit_from_config(const module_config_t *cfg)
{
    for (int i = 0; i < inverter_unit_count; i++) {
        if (inverter_units[i].cfg == cfg) {
            return &inverter_units[i];
        }
    }
    return NULL;
}

static inverter_unit_t *inverter_unit_from_uid(uint8_t uid)
{
    for (int i = 0; i < inverter_unit_count; i++) {
        if (inverter_units[i].cfg &&
            (uint8_t)inverter_units[i].cfg->modbus_uid == uid) {
            return &inverter_units[i];
        }
    }
    return NULL;
}

/**
 * @brief Sleep for duration_ms, waking early if the unit is signalled to stop.
 */
static void interruptible_sleep_ms(const inverter_unit_t *unit,
                                   uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms && unit->running) {
        usleep(INVERTER_SHUTDOWN_CHECK_INTERVAL_MS * 1000u);
        elapsed_ms += INVERTER_SHUTDOWN_CHECK_INTERVAL_MS;
    }
}

/**
 * @brief Execute a Modbus write (FC06 or FC16) directly on a unit.
 *
 * Shared by inverter_msg_callback() and the queue-drain path in
 * process_callback.  On failure, logs the error and returns -1 (does NOT
 * increment comm_fail_count – a write failure is an operational error, not a
 * connectivity loss).
 */
static int write_registers_to_device(inverter_unit_t *unit,
                                     uint16_t         addr,
                                     const uint16_t  *values,
                                     uint16_t         count,
                                     inverter_write_mode_t mode)
{
    int result;

    switch (mode) {
    case INVERTER_WRITE_MODE_FC06:
        if (count != 1) {
            LOG_ERROR("[Inverter RTU] %s: FC06 requires count=1 (got %u) at 0x%04X.",
                      unit->cfg->name, count, addr);
            return -1;
        }
        result = mb_rtu_client_write_single_register(
                     &unit->rtu_ctx, addr, values[0]);
        break;
    case INVERTER_WRITE_MODE_FC16:
        result = mb_rtu_client_write_multiple_registers(
                     &unit->rtu_ctx, addr, count, values);
        break;
    case INVERTER_WRITE_MODE_AUTO:
    default:
        if (count == 1) {
            result = mb_rtu_client_write_single_register(
                         &unit->rtu_ctx, addr, values[0]);
        } else {
            result = mb_rtu_client_write_multiple_registers(
                         &unit->rtu_ctx, addr, count, values);
        }
        break;
    }

    if (result != MB_RTU_CLIENT_OK) {
        LOG_ERROR("[Inverter RTU] %s: write to 0x%04X failed (err %d).",
                  unit->cfg->name, addr, result);
        return -1;
    }

    LOG_DEBUG("[Inverter RTU] %s: wrote %u register(s) at 0x%04X.",
              unit->cfg->name, count, addr);
    return 0;
}

/* ── Callbacks ────────────────────────────────────────────────────────── */

/**
 * @brief init_callback – open and configure the serial port.
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on failure.
 */
int inverter_rtu_init_callback(module_config_t *cfg)
{
    if (!cfg) {
        LOG_ERROR("[Inverter RTU] Invalid configuration.");
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[Inverter RTU] %s: unit not found.", cfg->name);
        return -1;
    }

    mb_rtu_client_config_t rtu_cfg = {
        .serial_path          = cfg->path,
        .baud_rate            = (uint32_t)cfg->baud_rate,
        .unit_id              = (uint8_t)cfg->modbus_uid,
        .response_timeout_ms  = 1000u,
    };

    LOG_INFO("[Inverter RTU] %s: opening %s @ %u baud uid=%d",
             cfg->name, cfg->path, cfg->baud_rate, cfg->modbus_uid);

    if (mb_rtu_client_connect(&unit->rtu_ctx, &rtu_cfg) != 0) {
        LOG_ERROR("[Inverter RTU] %s: failed to open serial port %s.",
                  cfg->name, cfg->path);
        return -1;
    }

    unit->comm_fail_count = 0;
    cfg->connection_state = CONNECTION_CONNECTED;

    LOG_INFO("[Inverter RTU] %s: serial port open.", cfg->name);
    return 0;
}

/**
 * @brief process_callback – drain write queue, then read all mapped registers.
 *
 * Phase 1: drain the per-unit write queue.  Each queued command is executed
 *          as FC06 (single) or FC16 (multi) before the read scan begins.
 *          A write failure is logged but does not abort the read phase.
 *
 * Phase 2: consecutive device addresses are batched into one FC03 request.
 *          On any read failure the fail counter increments; after
 *          INVERTER_COMM_FAIL_THRESHOLD consecutive failures the callback
 *          returns -1 to trigger error_callback.
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on communication failure.
 */
int inverter_rtu_process_callback(module_config_t *cfg)
{
    if (!cfg) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        return -1;
    }

    /* ── Phase 1: drain the write command queue ─────────────────────── */
    inverter_write_cmd_t cmd;
    while (queue_pop(&unit->cmd_queue, &cmd) == 0) {
        if (write_registers_to_device(unit, cmd.addr, cmd.values,
                                      cmd.count, cmd.mode) != 0) {
            LOG_WARNING("[Inverter RTU] %s: queued write to 0x%04X failed, "
                        "continuing scan.", cfg->name, cmd.addr);
        }
    }

    /* ── Phase 2: segmented FC03 read scan ──────────────────────────── */
    const device_map_profile_t *profile = unit->profile;

    if (profile->table_count == 0) {
        return 0;
    }

    uint16_t buf[MODBUS_MAX_READ_REGISTERS] = {0};

    size_t seg_start = 0;
    size_t seg_len   = 1;

    for (size_t i = 1; i <= profile->table_count; i++) {

        bool contiguous = (i < profile->table_count) &&
                          (profile->table[i].device_address ==
                           profile->table[i - 1].device_address + 1);
        bool chunk_full = (seg_len >= profile->read_chunk);

        if (contiguous && !chunk_full) {
            seg_len++;
            continue;
        }

        uint16_t start = profile->table[seg_start].device_address;
        uint16_t count = (uint16_t)seg_len;

        int result = mb_rtu_client_read_holding_registers(
                         &unit->rtu_ctx, start, count, buf);

        if (result != MB_RTU_CLIENT_OK) {
            unit->comm_fail_count++;
            LOG_WARNING("[Inverter RTU] %s read 0x%04X len %u failed "
                        "(err %d, fail %d/%d)",
                        cfg->name, start, count, result,
                        unit->comm_fail_count, INVERTER_COMM_FAIL_THRESHOLD);

            if (unit->comm_fail_count >= INVERTER_COMM_FAIL_THRESHOLD) {
                return -1;
            }
        } else {
            unit->comm_fail_count = 0;
            device_map_read_to_pool(profile, buf, start, (int)count);
        }

        usleep(INVERTER_INTER_SEGMENT_DELAY_US);

        seg_start = i;
        seg_len   = 1;
    }

    usleep((useconds_t)(cfg->rtu_poll_interval_ms * 1000u));
    return 0;
}

/**
 * @brief error_callback – close serial port and wait before the next reconnect.
 *
 * @param cfg               Module configuration.
 * @param connection_state  New connection state (CONNECTION_DISCONNECTED).
 * @return 0 (reconnect is managed by the thread loop).
 */
int inverter_rtu_error_callback(module_config_t *cfg, int connection_state)
{
    if (!cfg) {
        return 0;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (unit) {
        mb_rtu_client_disconnect(&unit->rtu_ctx);
        unit->comm_fail_count = 0;
    }

    cfg->connection_state = (connection_state_t)connection_state;

    LOG_WARNING("[Inverter RTU] %s: disconnected (state=%d). "
                "Retrying in %u ms …",
                cfg->name, connection_state, INVERTER_RECONNECT_DELAY_MS);

    if (unit) {
        interruptible_sleep_ms(unit, INVERTER_RECONNECT_DELAY_MS);
    } else {
        usleep(INVERTER_RECONNECT_DELAY_MS * 1000u);
    }
    return 0;
}

/**
 * @brief msg_callback – execute a Modbus write on an Inverter unit.
 *
 * Retained for framework completeness.  All writes are normally routed
 * through inverter_cmd_push() → queue drain inside process_callback.
 *
 * @param cfg    Module configuration of the target Inverter.
 * @param addr   Device register address to write.
 * @param values Values to write (host byte order).
 * @param count  Number of registers (1 → FC06, >1 → FC16).
 * @return 0 on success, -1 on failure.
 */
int inverter_msg_callback(module_config_t *cfg,
                           uint16_t addr, uint16_t *values, size_t count)
{
    if (!cfg || !values || count == 0) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[Inverter RTU] %s: msg_callback – unit not found.",
                  cfg->name);
        return -1;
    }

    return write_registers_to_device(unit, addr, values,
                                     (uint16_t)count,
                                     INVERTER_WRITE_MODE_AUTO);
}

/* ── Thread function ──────────────────────────────────────────────────── */

/**
 * @brief Poll thread for one Inverter unit.
 *
 *  open serial → segment reads → write pool → wait for next poll interval
 *              → on failure: close → wait → reopen
 */
static void *inverter_rtu_thread(void *arg)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;
    module_config_t *cfg  = unit->cfg;

    LOG_INFO("[Inverter RTU] Thread started: %s (profile=%s uid=%d)",
             cfg->name, unit->profile->name, cfg->modbus_uid);

    while (unit->running) {

        /* init --------------------------------------------------------- */
        if (unit->callbacks.init_callback(cfg) != 0) {
            LOG_ERROR("[Inverter RTU] %s: init failed, will retry.", cfg->name);
            interruptible_sleep_ms(unit, INVERTER_RECONNECT_DELAY_MS);
            continue;
        }

        /* process loop ------------------------------------------------- */
        while (unit->running) {
            if (unit->callbacks.process_callback(cfg) != 0) {
                if (unit->callbacks.error_callback) {
                    unit->callbacks.error_callback(cfg, CONNECTION_DISCONNECTED);
                }
                break; /* break inner loop → re-init */
            }
        }
    }

    mb_rtu_client_disconnect(&unit->rtu_ctx);
    LOG_INFO("[Inverter RTU] Thread stopped: %s", cfg->name);
    return NULL;
}

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Start all enabled Inverter modules.
 *
 * Selects each unit's register profile by modbus_uid, assigns callbacks,
 * spawns one polling thread per enabled Inverter unit, then starts the
 * CMOS bridge.
 *
 * @param inverters       Array of Inverter configurations from global_config.
 * @param inverter_count  Number of entries in the array.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_inverter_modules(module_config_t inverters[], int inverter_count)
{
    int status = 0;

    inverter_unit_count = 0;

    for (int i = 0; i < inverter_count; i++) {
        if (!inverters[i].enabled) {
            LOG_INFO("[Inverter] %s is disabled. Skipping.", inverters[i].name);
            continue;
        }

        const device_map_profile_t *profile =
            inverter_find_profile_by_uid((uint8_t)inverters[i].modbus_uid);

        if (!profile) {
            LOG_ERROR("[Inverter] %s: no profile registered for modbus_uid=%d.",
                      inverters[i].name, inverters[i].modbus_uid);
            status = -1;
            continue;
        }

        inverter_unit_t *unit      = &inverter_units[inverter_unit_count];
        unit->cfg                  = &inverters[i];
        unit->profile              = profile;
        unit->running              = 1;
        unit->comm_fail_count      = 0;
        memset(&unit->rtu_ctx, 0, sizeof(unit->rtu_ctx));
        unit->rtu_ctx.fd = -1;
        queue_init(&unit->cmd_queue);

        unit->callbacks.init_callback    = inverter_rtu_init_callback;
        unit->callbacks.process_callback = inverter_rtu_process_callback;
        unit->callbacks.error_callback   = inverter_rtu_error_callback;
        unit->callbacks.msg_callback     = inverter_msg_callback;
        unit->callbacks.start_callback   = NULL;

        inverter_unit_count++;

        LOG_INFO("[Inverter] Starting %s (profile=%s uid=%d).",
                 unit->cfg->name,
                 unit->profile->name,
                 unit->cfg->modbus_uid);

        if (pthread_create(&unit->thread, NULL,
                           inverter_rtu_thread, unit) != 0) {
            LOG_ERROR("[Inverter] Failed to create thread for %s.",
                      unit->cfg->name);
            queue_destroy(&unit->cmd_queue);
            unit->running = 0;
            inverter_unit_count--;
            status = -1;
        }
    }

    if (inverter_cmos_bridge_start() != 0) {
        LOG_ERROR("[Inverter] CMOS bridge failed to start.");
        status = -1;
    }

    return status;
}

/**
 * @brief Stop all running Inverter modules and join their threads.
 *
 * Stops the CMOS bridge first, then signals all unit threads to exit and
 * waits for them.
 *
 * @return 0 on success.
 */
int stop_inverter_modules(void)
{
    inverter_cmos_bridge_stop();

    for (int i = 0; i < inverter_unit_count; i++) {
        inverter_units[i].running = 0;
    }

    for (int i = 0; i < inverter_unit_count; i++) {
        pthread_join(inverter_units[i].thread, NULL);
        queue_destroy(&inverter_units[i].cmd_queue);
        LOG_INFO("[Inverter] %s stopped.", inverter_units[i].cfg->name);
    }

    inverter_unit_count = 0;
    return 0;
}

/**
 * @brief Enqueue a write command for the Inverter unit identified by uid.
 *
 * Called from the CMOS bridge thread.  Thread-safe via per-unit queue mutex.
 *
 * @return 0 on success, -1 if unit not found or queue is full.
 */
int inverter_cmd_push(uint8_t uid, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode)
{
    if (!values || count == 0 || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_uid(uid);
    if (!unit) {
        LOG_WARNING("[Inverter] inverter_cmd_push: uid=%u not found.", uid);
        return -1;
    }

    return queue_push(&unit->cmd_queue, addr, values, count, mode);
}
