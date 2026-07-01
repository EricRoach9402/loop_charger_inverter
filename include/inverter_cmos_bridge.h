/**
 * @file inverter_cmos_bridge.h
 * @brief CMOS ↔ Inverter Modbus RTU bridge interface.
 *
 * Starts/stops the two threads that bridge CMOS pub/sub to the
 * Modbus RTU write queue and the internal register pool.
 */

#ifndef INVERTER_CMOS_BRIDGE_H
#define INVERTER_CMOS_BRIDGE_H

/**
 * @brief Start the CMOS bridge (publisher + subscriber threads).
 *
 * Must be called after the Inverter polling threads are started so that
 * the command queue is ready to accept writes.
 *
 * @return 0 on success, -1 on failure.
 */
int inverter_cmos_bridge_start(void);

/**
 * @brief Stop the CMOS bridge and release resources.
 *
 * Cancels the subscriber thread and waits for both threads to exit.
 * Safe to call even if start failed.
 */
void inverter_cmos_bridge_stop(void);

#endif /* INVERTER_CMOS_BRIDGE_H */
