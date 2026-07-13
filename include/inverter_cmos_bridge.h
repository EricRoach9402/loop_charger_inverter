/**
 * @file inverter_cmos_bridge.h
 * @brief CMOS subscriber bridge for Inverter Modbus write commands and pool reads.
 *
 * Responsibilities
 * ────────────────
 *  - Spawns a dedicated CMOS subscriber thread (cmos_sub_spin_ctx is blocking,
 *    so it must never run inside init_callback / process_callback).
 *  - Spawns a publisher poll thread that keeps the read-response publisher alive.
 *  - The HMI only sends cmd + value (it has no notion of uid/addr). Each HMI
 *    command (topic="hmi_inverter", type="command", key=<cmd name>) is routed
 *    to its own on_xxx_cmd handler, which owns the register address for that
 *    command, converts value to uint16_t, and delegates the shared
 *    validate/enqueue or validate/read flow to the internal on_write() /
 *    on_read() helpers (see inverter_cmos_bridge.c).
 *
 * CMOS message protocol (publisher side)
 * ───────────────────────────────────────
 *  Topic: "hmi_inverter", type="command", key=<cmd name> (e.g. "output_voltage_set")
 *    cmos_publish(NULL, "command", "output_voltage_set", "<uint16 value>");
 *
 *  Periodic status push (topic "inverter", see cmos_pub_poll_thread):
 *    key=ac_output_voltage / ... value=<uint16 decimal>
 *
 * Lifecycle
 * ─────────
 *  Call inverter_cmos_bridge_start() after start_inverter_modules().
 *  Call inverter_cmos_bridge_stop()  before stop_inverter_modules().
 *
 *  Both functions are called internally by inverter_module.c; main.c needs no
 *  changes.
 */

#ifndef INVERTER_CMOS_BRIDGE_H
#define INVERTER_CMOS_BRIDGE_H

/**
 * @brief Start the CMOS bridge (subscriber thread + publisher poll thread).
 *
 * Initialises the CMOS publisher for read responses, then spawns both threads.
 *
 * @return 0 on success, -1 on failure.
 */
int inverter_cmos_bridge_start(void);

/**
 * @brief Stop the CMOS bridge and release all resources.
 *
 * Cancels the subscriber thread (pthread_cancel – safe because epoll_wait is
 * a cancellation point), joins both threads, and closes the publisher.
 */
void inverter_cmos_bridge_stop(void);

#endif /* INVERTER_CMOS_BRIDGE_H */
