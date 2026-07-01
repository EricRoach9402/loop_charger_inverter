/**
 * @file device_register_map.c
 * @brief Generic Modbus device register → internal pool mapping engine.
 *
 * Pool addresses are absolute: each per-unit mapping table hardcodes its
 * pool_address values.  No runtime pool-base arithmetic is performed here.
 *
 * The pool stores raw register values.  No masking is applied at this layer.
 * Consumers needing a masked view use pool_read_masked_by_device_addr().
 */

#include <string.h>

#include "device_register_map.h"
#include "log.h"

/* ── Shared internal pool ─────────────────────────────────────────────── */

uint16_t         internal_pool[INTERNAL_POOL_SIZE];
pthread_rwlock_t internal_pool_lock = PTHREAD_RWLOCK_INITIALIZER;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void device_register_map_init(void)
{
    memset(internal_pool, 0, sizeof(internal_pool));
}

/* ── Core mapping API ─────────────────────────────────────────────────── */

const device_register_mapping_t *device_find_slot(
    const device_map_profile_t *profile,
    uint16_t                    device_address)
{
    if (!profile || !profile->table || profile->table_count == 0) {
        return NULL;
    }

    size_t low  = 0;
    size_t high = profile->table_count;

    while (low < high) {
        size_t   mid  = low + (high - low) / 2;
        uint16_t addr = profile->table[mid].device_address;

        if (addr == device_address) {
            return &profile->table[mid];
        }
        if (addr < device_address) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }

    return NULL;
}

void device_map_read_to_pool(const device_map_profile_t *profile,
                             const uint16_t             *read_buffer,
                             uint16_t                    start_device_address,
                             int                         read_count)
{
    if (!profile || !read_buffer || read_count <= 0 ||
        !profile->table || profile->table_count == 0) {
        return;
    }

    for (int i = 0; i < read_count; i++) {
        uint16_t dev_addr = (uint16_t)(start_device_address + (uint16_t)i);

        const device_register_mapping_t *m =
            device_find_slot(profile, dev_addr);

        if (!m) {
            LOG_VERBOSE("[device_map] no mapping for dev 0x%04X", dev_addr);
            continue;
        }

        if (m->pool_address >= INTERNAL_POOL_SIZE) {
            LOG_WARNING("[device_map] pool_address 0x%04X out of bounds (dev 0x%04X)",
                        m->pool_address, dev_addr);
            continue;
        }

        /* Store the raw value; no masking at the ingest stage. */
        uint16_t raw = read_buffer[i];

        pthread_rwlock_wrlock(&internal_pool_lock);
        internal_pool[m->pool_address] = raw;
        pthread_rwlock_unlock(&internal_pool_lock);

        LOG_VERBOSE("[device_map] dev 0x%04X -> pool[0x%04X] = 0x%04X",
                    dev_addr, m->pool_address, raw);
    }
}

/* ── Direct pool access ───────────────────────────────────────────────── */

bool pool_read_register(uint16_t pool_address, uint16_t *out_value)
{
    if (!out_value || pool_address >= INTERNAL_POOL_SIZE) {
        return false;
    }

    pthread_rwlock_rdlock(&internal_pool_lock);
    *out_value = internal_pool[pool_address];
    pthread_rwlock_unlock(&internal_pool_lock);

    return true;
}

bool pool_write_register(uint16_t pool_address, uint16_t value)
{
    if (pool_address >= INTERNAL_POOL_SIZE) {
        return false;
    }

    pthread_rwlock_wrlock(&internal_pool_lock);
    internal_pool[pool_address] = value;
    pthread_rwlock_unlock(&internal_pool_lock);

    return true;
}

/* ── Profile-assisted pool access ─────────────────────────────────────── */

bool pool_read_by_device_addr(const device_map_profile_t *profile,
                              uint16_t                    device_address,
                              uint16_t                   *out_value)
{
    if (!profile || !out_value) {
        return false;
    }

    const device_register_mapping_t *m = device_find_slot(profile, device_address);
    if (!m) {
        return false;
    }

    return pool_read_register(m->pool_address, out_value);
}

bool pool_read_masked_by_device_addr(const device_map_profile_t *profile,
                                     uint16_t                    device_address,
                                     uint16_t                    mask,
                                     uint16_t                   *out_value)
{
    if (!profile || !out_value) {
        return false;
    }

    uint16_t raw = 0u;
    if (!pool_read_by_device_addr(profile, device_address, &raw)) {
        return false;
    }

    *out_value = (uint16_t)(raw & mask);
    return true;
}

bool pool_write_by_device_addr(const device_map_profile_t *profile,
                               uint16_t                    device_address,
                               uint16_t                    value)
{
    if (!profile) {
        return false;
    }

    const device_register_mapping_t *m = device_find_slot(profile, device_address);
    if (!m) {
        return false;
    }

    return pool_write_register(m->pool_address, value);
}

/* ── Access permission check ──────────────────────────────────────────── */

uint8_t check_register_access_range(const device_map_profile_t *profile,
                                    uint16_t                    pool_start,
                                    uint16_t                    count,
                                    register_access_t           required_access)
{
    if (!profile || !profile->table || count == 0) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    uint16_t pool_end = (uint16_t)(pool_start + count - 1u);

    for (uint16_t pa = pool_start; pa <= pool_end; pa++) {

        bool found = false;

        for (size_t i = 0; i < profile->table_count; i++) {
            if (profile->table[i].pool_address != pa) {
                continue;
            }

            found = true;
            register_access_t reg_access = profile->table[i].access;

            if (required_access == ACCESS_RO && reg_access == ACCESS_WO) {
                LOG_WARNING("[device_map] read rejected: pool 0x%04X is write-only (%s)",
                            pa, profile->table[i].description);
                return MODBUS_EX_ILLEGAL_FUNCTION;
            }

            if (required_access == ACCESS_WO && reg_access == ACCESS_RO) {
                LOG_WARNING("[device_map] write rejected: pool 0x%04X is read-only (%s)",
                            pa, profile->table[i].description);
                return MODBUS_EX_ILLEGAL_FUNCTION;
            }

            break;
        }

        if (!found) {
            LOG_WARNING("[device_map] access rejected: pool 0x%04X not mapped in profile '%s'",
                        pa, profile->name);
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    }

    return 0;
}
