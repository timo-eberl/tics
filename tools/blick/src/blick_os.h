#ifndef BLICK_OS_H
#define BLICK_OS_H

#include "blick_protocol.h"

#include <stdint.h>

// --- Host API (Used by blick.c) ---
blick_shm_header* blick_os_host_init_shm(void);
void blick_os_host_spawn_viewer(void);
void blick_os_host_shutdown(blick_shm_header* shm);

// --- Viewer API (Used by viewer_main.c) ---
blick_shm_header* blick_os_viewer_open_shm(void);
void blick_os_viewer_close_shm(blick_shm_header* shm);

// --- Utility ---
void blick_os_sleep_ms(uint32_t ms);

#endif // BLICK_OS_H
