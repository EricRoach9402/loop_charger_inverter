/**
 * @file inverter_map.c
 * @brief Inverter register mapping tables – one independent table per physical unit.
 *
 * Table format:  { device_address, pool_address, access, description }
 *
 *  device_address  – FC03 register address on the hardware.
 *  pool_address    – Absolute position in internal_pool[].  Unique per unit;
 *                    derived from the unit's pool base + its sequential offset.
 *  access          – ACCESS_RO / ACCESS_RW / ACCESS_WO for external requests.
 *
 * Pool regions
 * ────────────
 *  Inverter#1  (modbus_uid = 1):  0x0000 – 0x001F
 *
 * Pool stores raw register values.  No masking is applied here.
 * Consumers needing a masked view use pool_read_masked_by_device_addr().
 *
 * Rules
 * ─────
 *  • Rows MUST be sorted by device_address ascending (binary search).
 *  • pool_address values must be unique across ALL tables in the project.
 *  • Each table is self-contained; adding a unit never touches another table.
 *
 * Adding a new unit of the same hardware model
 * ────────────────────────────────────────────
 *  1. Copy an existing table, choose a new non-overlapping pool base.
 *  2. Update all pool_address values (new_base + sequential_offset).
 *  3. Add a new device_map_profile_t definition.
 *  4. Declare it extern in inverter_map.h.
 *  5. Register it in inverter_map.c uid→profile lookup table.
 */

#include "inverter_map.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* ═══════════════════════════════════════════════════════════════════════════
 * Inverter#1 – modbus_uid 1 – pool base 0x0000
 *
 * Typical solar / grid inverter Modbus RTU register map.
 *
 *  device addr   pool addr    access     description
 * ═══════════════════════════════════════════════════════════════════════════ */
static const device_register_mapping_t inverter1_mapping_table[] = {
    { 0x2000,  0xB000,  ACCESS_RW,  "operation_commands" },
    { 0x2001,  0xB001,  ACCESS_RW,  "frequency_write_commands" },
    { 0x2002,  0xB002,  ACCESS_RW,  "fault_Control_commands" },

    { 0x2100,  0xBB00,  ACCESS_RO,  "fault_warning_code" },
    { 0x2101,  0xBB01,  ACCESS_RO,  "inverter_operating_status" },
    { 0x2102,  0xBB02,  ACCESS_RO,  "frequency_read_command" },
    { 0x2103,  0xBB03,  ACCESS_RO,  "output_frequency" },
    { 0x2104,  0xBB04,  ACCESS_RO,  "output_current" },
    { 0x2105,  0xBB05,  ACCESS_RO,  "dc_Bus_voltage" },

    { 0x210C,  0xBB0C,  ACCESS_RO,  "motor_actual_speed" },

    { 0x220A,  0xBB0A,  ACCESS_RO,  "pid feedback value" },
};

const device_map_profile_t inverter1_profile = {
    .name        = "Inverter1",
    .table       = inverter1_mapping_table,
    .table_count = ARRAY_SIZE(inverter1_mapping_table),
    .read_chunk  = 20,
};

/* ═══════════════════════════════════════════════════════════════════════════
 * UID → profile registry
 *
 * Add one entry here when a new Inverter unit is commissioned.
 * inverter_module.c requires no modification.
 * ═══════════════════════════════════════════════════════════════════════════ */
static const inverter_uid_profile_entry_t inverter_uid_profile_map[] = {
    { 1u, &inverter1_profile }
};

#define INVERTER_UID_PROFILE_COUNT \
    (int)(sizeof(inverter_uid_profile_map) / sizeof(inverter_uid_profile_map[0]))

const device_map_profile_t *inverter_find_profile_by_uid(uint8_t uid)
{
    for (int i = 0; i < INVERTER_UID_PROFILE_COUNT; i++) {
        if (inverter_uid_profile_map[i].modbus_uid == uid) {
            return inverter_uid_profile_map[i].profile;
        }
    }
    return NULL;
}

// RW
const uint16_t int_operation_cmd_reg = 0xB000;
const uint16_t int_frequency_write_cmd_reg = 0xB001;
const uint16_t int_fault_control_cmd_reg = 0xB002;

// RO
const uint16_t int_fault_warning_code_reg = 0xBB00;
const uint16_t int_operation_status_reg = 0xBB01;
const uint16_t int_frequency_read_cmd_reg = 0xBB02;
const uint16_t int_out_frequency_reg = 0xBB03;
const uint16_t int_out_current_reg = 0xBB04;
const uint16_t int_dc_bus_voltage_reg = 0xBB05;
const uint16_t int_motor_actual_speed_reg = 0xBB0C;
const uint16_t int_pid_feedback_value_reg = 0xBB0A;

//device
const uint16_t dev_operation_cmd_reg = 0x2001;
