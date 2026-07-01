/**
 * @file modbus_rtu_client.h
 * @brief Modbus RTU client (master) application-facing API.
 *
 * Provides blocking, thread-safe read/write functions for accessing registers
 * on a remote Modbus RTU slave device over a serial port.
 *
 * Relationship to other layers:
 *   modbus_defines.h      -- shared Modbus protocol constants and limits
 *   modbus_rtu_client     -- THIS FILE: serial transport + application API
 *
 * Thread safety:
 *   Multiple threads may share one mb_rtu_client_ctx_t.  Requests are
 *   serialized internally by a mutex: only one request is in flight at a time.
 *
 * Serial framing:
 *   Each request is a raw Modbus RTU ADU:
 *     [slave_addr (1)] [fc (1)] [payload...] [CRC_lo (1)] [CRC_hi (1)]
 *   CRC16 uses the standard Modbus polynomial (0xA001, LSB first).
 */

#ifndef MODBUS_RTU_CLIENT_H
#define MODBUS_RTU_CLIENT_H

#include <pthread.h>
#include <stdint.h>
#include "modbus_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Return codes ──────────────────────────────────────────────────────── */

/**
 * Return values for all mb_rtu_client_* request functions.
 *
 *   0          – success
 *   > 0        – Modbus exception code (MODBUS_EX_* from modbus_defines.h)
 *   < 0        – transport or framing error (one of the codes below)
 */
#define MB_RTU_CLIENT_OK                 0
#define MB_RTU_CLIENT_ERR_ARG           (-1)  /**< Invalid argument.                  */
#define MB_RTU_CLIENT_ERR_NOT_OPEN      (-2)  /**< Serial port not open; call connect. */
#define MB_RTU_CLIENT_ERR_TRANSPORT     (-3)  /**< Serial write/read error.            */
#define MB_RTU_CLIENT_ERR_TIMEOUT       (-4)  /**< Response not received in time.      */
#define MB_RTU_CLIENT_ERR_FRAME         (-5)  /**< Malformed or unexpected response.   */
#define MB_RTU_CLIENT_ERR_CRC           (-6)  /**< CRC mismatch in response.           */

/* ── Configuration ─────────────────────────────────────────────────────── */

/**
 * @brief Client configuration.
 *
 * All pointer fields must remain valid until mb_rtu_client_disconnect() returns.
 */
typedef struct mb_rtu_client_config {
    const char  *serial_path;           /**< Serial device path (e.g. "/dev/ttyS3"). */
    uint32_t     baud_rate;             /**< Baud rate (e.g. 9600, 19200, 115200).   */
    uint8_t      unit_id;               /**< Modbus slave address for every request. */
    uint32_t     response_timeout_ms;   /**< Per-request timeout; 0 = default (1 s). */
} mb_rtu_client_config_t;

/* ── Runtime context ───────────────────────────────────────────────────── */

/**
 * @brief Client runtime context.
 *
 * Zero-initialize before calling mb_rtu_client_connect().
 * Do not modify fields directly after connect.
 */
typedef struct mb_rtu_client_ctx {
    int                     fd;     /**< Serial port file descriptor; -1 when closed. */
    pthread_mutex_t         lock;   /**< Serializes concurrent requests.              */
    mb_rtu_client_config_t  cfg;    /**< Copy of configuration.                       */
} mb_rtu_client_ctx_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/**
 * @brief Open and configure the serial port.
 *
 * Initializes the mutex and opens the serial device in raw mode (8N1,
 * no flow control).  Sets the baud rate and per-request receive timeout.
 *
 * @param ctx  Zero-initialized context; must remain valid until disconnect.
 * @param cfg  Configuration; pointer fields are used by reference.
 * @return 0 on success, -1 on error.
 */
int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx,
                           const mb_rtu_client_config_t *cfg);

/**
 * @brief Close the serial port and release resources.
 *
 * Safe to call even if connect failed or was never called.
 */
void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx);

/* ── CRC utility ───────────────────────────────────────────────────────── */

/**
 * @brief Compute Modbus CRC16 over an arbitrary byte buffer.
 *
 * Uses the standard Modbus polynomial (0xA001, initial value 0xFFFF).
 * The two CRC bytes in a Modbus RTU ADU are stored LSB-first.
 *
 * @param data    Byte array.
 * @param length  Number of bytes.
 * @return 16-bit CRC value.
 */
uint16_t mb_rtu_compute_crc16(const uint8_t *data, size_t length);

/* ── Register access ───────────────────────────────────────────────────── */

/**
 * @brief FC03 – Read Holding Registers.
 *
 * @param ctx   Open context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to read (1 – MODBUS_MAX_READ_REGISTERS).
 * @param out   Caller-owned buffer; receives qty values in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                          uint16_t addr, uint16_t qty,
                                          uint16_t *out);

/**
 * @brief FC06 – Write Single Register.
 *
 * @param ctx    Open context.
 * @param addr   Register address (0-based).
 * @param value  Register value in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t value);

/**
 * @brief FC16 – Write Multiple Registers.
 *
 * @param ctx   Open context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to write (1 – MODBUS_MAX_WRITE_REGISTERS).
 * @param data  Register values in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                            uint16_t addr, uint16_t qty,
                                            const uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_CLIENT_H */
