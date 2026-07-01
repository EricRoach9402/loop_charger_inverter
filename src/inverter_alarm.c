/**
 * @file inverter_alarm.c
 * @brief Inverter alarm tables and alarm bridge registration.
 *
 * Owns: the alarm tables (what to monitor), per-unit runtime state, and
 * the wiring to alarm_bridge (inverter_alarm_register_all).
 *
 * Does NOT own alarm behaviour – every event is forwarded verbatim to
 * inverter_alarm_manager_handle_event(), which decides what to do.
 *
 * Table columns
 * ─────────────
 *  device_address  FC03 register address (must exist in inverter_map.c).
 *  condition       ALARM_COND_BITMASK / ALARM_COND_RANGE / ALARM_COND_CHANGE.
 *  lo_limit        Lower bound (RANGE) or ignored (BITMASK / CHANGE).
 *  hi_limit        Upper bound (RANGE) or bit mask (BITMASK) or ignored (CHANGE).
 *  error_code      Forwarded in the event; meaning defined in inverter_alarm.h.
 *  description     Human-readable label; forwarded in the event only.
 *
 * Condition quick reference
 * ─────────────────────────
 *  BITMASK  fires when (value & hi_limit) == hi_limit.
 *           Same register can appear N times for N independent bits.
 *
 *  RANGE    fires when value < lo_limit || value > hi_limit.
 *           Upper bound only: lo_limit = 0.
 *           Lower bound only: hi_limit = 0xFFFF.
 *
 *  CHANGE   fires when value != prev_value.
 *           No sticky suppression – every change is an independent event.
 *
 * Register address reference (from inverter_map.c)
 * ──────────────────────────────────────────────────
 *  0x0001  Fault_Code              0x0002  Warning_Code
 *  0x0010  AC_Output_Voltage       0x0015  AC_Output_Load_Percent
 *  0x0042  Battery_State_of_Charge 0x0043  Battery_Temperature
 *  0x0050  Heatsink_Temperature
 */

#include "inverter_alarm.h"
#include "inverter_alarm_manager.h"
#include "alarm_engine.h"
#include "alarm_bridge.h"
#include "device_register_map.h"
#include "config_loader.h"
#include "inverter/inverter_map.h"
#include "log.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* ═══════════════════════════════════════════════════════════════════════
 * Shared alarm table — same hardware model for all Inverter units.
 * Per-unit runtime state is kept separate so sticky flags and prev_value
 * are not shared across units.
 *
 *  device_addr  condition            lo      hi      error_code                   description
 * ═══════════════════════════════════════════════════════════════════════ */
static const alarm_entry_t inverter_alarm_table[] = {
    { 0x0001, ALARM_COND_CHANGE,  0x0000, 0x0000, INV_ERR_FAULT_CODE,         "fault code changed"              },
    { 0x0002, ALARM_COND_CHANGE,  0x0000, 0x0000, INV_ERR_WARNING_CODE,       "warning code changed"            },
    { 0x0010, ALARM_COND_RANGE,   0x0082, 0x00DC, INV_ERR_OUTPUT_VOLT_HIGH,   "AC output voltage out of range"  },
    { 0x0015, ALARM_COND_RANGE,   0x0000, 0x0064, INV_ERR_OVERLOAD,           "output load >= 100%"             },
    { 0x0042, ALARM_COND_RANGE,   0x0014, 0xFFFF, INV_ERR_BATT_SOC_LOW,       "battery SoC < 20%"               },
    { 0x0043, ALARM_COND_RANGE,   0x0000, 0x02BC, INV_ERR_BATT_TEMP_HIGH,     "battery temperature > 70 °C"     },
    { 0x0050, ALARM_COND_RANGE,   0x0000, 0x0320, INV_ERR_HEATSINK_TEMP_HIGH, "heatsink temperature > 80 °C"    },
};

/* Per-unit mutable runtime state – one slot set per Inverter unit. */
static alarm_state_t inverter1_alarm_states[ARRAY_SIZE(inverter_alarm_table)];
static alarm_state_t inverter2_alarm_states[ARRAY_SIZE(inverter_alarm_table)];
static alarm_state_t inverter3_alarm_states[ARRAY_SIZE(inverter_alarm_table)];

/* ── UID → alarm state registry ──────────────────────────────────────── */

typedef struct {
    uint8_t       uid;
    alarm_state_t *states;
    size_t         count;
} inverter_alarm_registry_t;

static const inverter_alarm_registry_t alarm_registry[] = {
    { 1u, inverter1_alarm_states, ARRAY_SIZE(inverter_alarm_table) },
    { 2u, inverter2_alarm_states, ARRAY_SIZE(inverter_alarm_table) },
    { 3u, inverter3_alarm_states, ARRAY_SIZE(inverter_alarm_table) },
};

/* Stable ctx storage – lifetime must outlast alarm_bridge_stop(). */
static alarm_engine_ctx_t inverter_alarm_ctxs[ARRAY_SIZE(alarm_registry)];
static size_t             inverter_alarm_ctx_count = 0u;

/* ── alarm_read_fn ────────────────────────────────────────────────────── */

/**
 * @brief Read one register from the internal pool by device_address.
 * userdata is a const device_map_profile_t *.
 */
static bool alarm_read_register(uint16_t  device_address,
                                uint16_t *out_value,
                                void     *userdata)
{
    const device_map_profile_t *profile = (const device_map_profile_t *)userdata;
    return pool_read_by_device_addr(profile, device_address, out_value);
}

/* ── alarm_event_fn ───────────────────────────────────────────────────── */

/**
 * @brief Thin forwarder – all events go straight to the Alarm Manager.
 */
static void forward_to_manager(const alarm_entry_t *entry,
                               uint16_t              value,
                               alarm_event_t         event,
                               void                 *userdata)
{
    inverter_alarm_manager_handle_event(entry, value, event, userdata);
}

/* ── Public API ───────────────────────────────────────────────────────── */

void inverter_alarm_register_all(void)
{
    inverter_alarm_ctx_count = 0u;

    for (size_t i = 0u; i < ARRAY_SIZE(alarm_registry); i++) {
        uint8_t uid = alarm_registry[i].uid;

        /* Locate the device profile for this uid. */
        const device_map_profile_t *profile =
            inverter_find_profile_by_uid(uid);
        if (!profile) {
            LOG_WARNING("[Alarm] inverter_alarm_register_all: "
                        "no device profile for uid=%u – skipped.", uid);
            continue;
        }

        /* Locate the module_config_t for this uid (event_data for manager). */
        module_config_t *cfg = NULL;
        for (int j = 0; j < global_config.inverter_count; j++) {
            if ((uint8_t)global_config.inverter[j].modbus_uid == uid) {
                cfg = &global_config.inverter[j];
                break;
            }
        }
        if (!cfg) {
            LOG_WARNING("[Alarm] inverter_alarm_register_all: "
                        "no module config for uid=%u – skipped.", uid);
            continue;
        }

        alarm_engine_ctx_t *ctx = &inverter_alarm_ctxs[inverter_alarm_ctx_count];

        ctx->table      = inverter_alarm_table;
        ctx->states     = alarm_registry[i].states;
        ctx->count      = alarm_registry[i].count;
        ctx->read_fn    = alarm_read_register;
        ctx->event_fn   = forward_to_manager;
        ctx->read_data  = (void *)profile;
        ctx->event_data = (void *)cfg;

        if (alarm_bridge_register_ctx(ctx) != 0) {
            LOG_ERROR("[Alarm] inverter_alarm_register_all: "
                      "failed to register ctx for uid=%u.", uid);
            continue;
        }

        inverter_alarm_ctx_count++;
        LOG_INFO("[Alarm] registered alarm context for uid=%u (%s).",
                 uid, cfg->name);
    }
}
