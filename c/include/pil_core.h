/* pil_core.h -- the platform-independent flight-software step function.
 *
 * This is deliberately the ONLY piece of logic shared between the
 * host simulation (c/pil/host_sim_main.c, runs on the PC/Ubuntu) and
 * the STM32 firmware (c/stm32/, Phase 4b) -- everything platform-
 * specific (UART driver calls, DWT cycle counting, HAL init) lives
 * outside this file. Validating pil_core.c on the host is therefore
 * validating the exact logic that later runs on the target; only the
 * I/O plumbing around it differs.
 */
#ifndef PIL_CORE_H
#define PIL_CORE_H

#include "pil_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initializes the module-static ESKF and controller to the 160 m/s /
 * 8000 m trim condition used throughout this project's demos
 * (python/scripts/demo_closed_loop.py).-- an embedded target is not expected to solve trim online;
 * it receives pre-computed trim/reference parameters, consistent with
 * how a real flight-software load would be built and configured. */
void pil_core_init(void);

/* Runs one navigation-predict [+ optional GNSS/baro update] +
 * control step. Fills `out` (including checksum). Returns 0 on
 * success, -1 if `in`'s checksum did not verify (in which case `out`
 * is not touched -- the caller should re-request or skip the step). */
int pil_core_step(const PilInputPacket *in, PilOutputPacket *out);

#ifdef __cplusplus
}
#endif

#endif /* PIL_CORE_H */
