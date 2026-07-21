/**
 * @file inverter_map.h
 * @brief Inverter device register profiles – one profile per physical unit.
 *
 * Each profile corresponds to exactly one Inverter unit identified by its
 * modbus_uid in config.json.  Pool addresses are absolute and hardcoded
 * in each unit's mapping table in inverter_map.c.
 *
 * Pool layout
 * ───────────
 *  Inverter#1  (modbus_uid = 1):  pool 0x0000 – 0x001F  (32 registers)
 *  Inverter#2  (modbus_uid = 2):  pool 0x0020 – 0x003F  (32 registers)
 *  Inverter#3  (modbus_uid = 3):  pool 0x0040 – 0x005F  (32 registers)
 *
 * Adding a new Inverter unit
 * ──────────────────────────
 *  1. Add a mapping table in inverter_map.c with a unique pool_address range.
 *  2. Declare the new profile below (extern const device_map_profile_t …).
 *  3. Add an entry to the uid→profile lookup table in inverter_map.c.
 *
 * Adding a new hardware model
 * ───────────────────────────
 *  Same as above.  Each physical unit still gets its own table and profile
 *  regardless of whether the hardware model is shared with other units.
 */

#ifndef INVERTER_MAP_H
#define INVERTER_MAP_H

#include "device_register_map.h"

/** Inverter#1 – modbus_uid 1, pool region 0x0000–0x001F. */
extern const device_map_profile_t inverter1_profile;

/* ── UID → profile registry ───────────────────────────────────────────── */

/**
 * @brief Maps one modbus_uid to its per-unit register profile.
 *
 * The full table lives in inverter_map.c.  All additions of new Inverter
 * units are confined to inverter_map.c/h; inverter_module.c requires no
 * modification.
 */
typedef struct {
    uint8_t                     modbus_uid;
    const device_map_profile_t *profile;
} inverter_uid_profile_entry_t;

/**
 * @brief Look up the register profile for a given modbus_uid.
 *
 * @param uid  Modbus unit identifier from config.json.
 * @return Pointer to the matching profile, or NULL if not registered.
 */
const device_map_profile_t *inverter_find_profile_by_uid(uint8_t uid);

// RW
extern const uint16_t int_operation_cmd_reg;
extern const uint16_t int_frequency_write_cmd_reg;
extern const uint16_t int_fault_control_cmd_reg;

// RO
extern const uint16_t int_fault_warning_code_reg;
extern const uint16_t int_operation_status_reg;
extern const uint16_t int_frequency_read_cmd_reg;
extern const uint16_t int_out_frequency_reg;
extern const uint16_t int_out_current_reg;
extern const uint16_t int_dc_bus_voltage_reg;
extern const uint16_t int_motor_actual_speed_reg;
extern const uint16_t int_pid_feedback_value_reg;

//device
extern const uint16_t dev_operation_cmd_reg;

#endif /* INVERTER_MAP_H */
