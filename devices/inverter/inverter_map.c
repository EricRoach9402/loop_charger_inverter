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
 *  Inverter#1  (modbus_uid = 1):  0x0000 – 0x001F  (32 registers)
 *  Inverter#2  (modbus_uid = 2):  0x0020 – 0x003F  (32 registers)
 *  Inverter#3  (modbus_uid = 3):  0x0040 – 0x005F  (32 registers)
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

    /* ── Status & fault registers ──────────────────────────────────────── */
    { 0x0000,  0xBB00,  ACCESS_RO,  "INV1_Device_Status"              },
    { 0x0001,  0xBB01,  ACCESS_RO,  "INV1_Fault_Code"                 },
    { 0x0002,  0xBB02,  ACCESS_RO,  "INV1_Warning_Code"               },
    { 0x0003,  0xBB03,  ACCESS_RO,  "INV1_Warning_Code1"              },

    /* ── AC output measurements ─────────────────────────────────────────── */
    { 0x0011,  0xBB04,  ACCESS_RO,  "INV1_AC_Output_Current"          },
    { 0x0012,  0xBB05,  ACCESS_RO,  "INV1_AC_Output_Frequency"        },
    { 0x0013,  0xBB06,  ACCESS_RO,  "INV1_AC_Output_Active_Power"     },
    { 0x0014,  0xBB07,  ACCESS_RO,  "INV1_AC_Output_Apparent_Power"   },
    { 0x0015,  0xBB08,  ACCESS_RO,  "INV1_AC_Output_Load_Percent"     },

    /* ── AC input / grid measurements ──────────────────────────────────── */
    { 0x0020,  0xBB09,  ACCESS_RO,  "INV1_Grid_Voltage"               },
    { 0x0021,  0xBB0A,  ACCESS_RO,  "INV1_Grid_Frequency"             },

    /* ── DC bus / PV input ──────────────────────────────────────────────── */
    { 0x0030,  0xBB0B,  ACCESS_RO,  "INV1_PV_Input_Voltage"           },
    { 0x0031,  0xBB0C,  ACCESS_RO,  "INV1_PV_Input_Current"           },
    { 0x0032,  0xBB0D,  ACCESS_RO,  "INV1_PV_Input_Power"             },
    { 0x0033,  0xBB0E,  ACCESS_RO,  "INV1_DC_Bus_Voltage"             },

    /* ── Battery measurements ───────────────────────────────────────────── */
    { 0x0040,  0xBB0F,  ACCESS_RO,  "INV1_Battery_Voltage"            },
    { 0x0041,  0xBB10,  ACCESS_RO,  "INV1_Battery_Current"            },
    { 0x0042,  0xBB11,  ACCESS_RO,  "INV1_Battery_State_of_Charge"    },
    { 0x0043,  0xBB12,  ACCESS_RO,  "INV1_Battery_Temperature"        },
    { 0x0044,  0xBB13,  ACCESS_RO,  "INV1_Battery_Charge_Mode"        },

    /* ── Temperature ────────────────────────────────────────────────────── */
    { 0x0050,  0xBB14,  ACCESS_RO,  "INV1_Heatsink_Temperature"       },
    { 0x0051,  0xBB15,  ACCESS_RO,  "INV1_Transformer_Temperature"    },

    /* ── Energy counters ────────────────────────────────────────────────── */
    { 0x0060,  0xBB16,  ACCESS_RO,  "INV1_Total_Output_Energy_Lo"     },
    { 0x0061,  0xBB17,  ACCESS_RO,  "INV1_Total_Output_Energy_Hi"     },
    { 0x0062,  0xBB18,  ACCESS_RO,  "INV1_Total_PV_Energy_Lo"         },
    { 0x0063,  0xBB19,  ACCESS_RO,  "INV1_Total_PV_Energy_Hi"         },

    /* ── Configuration (read/write) ─────────────────────────────────────── */
    { 0x0100,  0xBB1A,  ACCESS_RW,  "INV1_Output_Voltage_Set"         },
    { 0x0101,  0xBB1B,  ACCESS_RW,  "INV1_Output_Frequency_Set"       },
    { 0x0102,  0xBB1C,  ACCESS_RW,  "INV1_Battery_Low_Voltage_Set"    },
    { 0x0103,  0xBB1D,  ACCESS_RW,  "INV1_Battery_High_Voltage_Set"   },
    { 0x0104,  0xBB1E,  ACCESS_RW,  "INV1_Max_Charge_Current_Set"     },
    { 0x0105,  0xBB1F,  ACCESS_RW,  "INV1_Work_Mode_Set"              },
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
