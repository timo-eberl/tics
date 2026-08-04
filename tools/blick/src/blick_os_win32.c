#include "blick_os.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

// Slashes are invalid in Windows kernel object names.
#define BLICK_SHM_NAME "blick_shm"

static HANDLE shm_handle = NULL;
static HANDLE viewer_process = NULL;

blick_shm_header* blick_os_host_init_shm(void) {
	shm_handle = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
									sizeof(blick_shm_header), BLICK_SHM_NAME);
	if (!shm_handle) {
		fprintf(stderr, "[BLICK] Error: CreateFileMappingA failed (%lu)\n", GetLastError());
		return NULL;
	}

	blick_shm_header* shm = (blick_shm_header*)MapViewOfFile(shm_handle, FILE_MAP_ALL_ACCESS,
															 0, 0, sizeof(blick_shm_header));
	if (!shm) {
		fprintf(stderr, "[BLICK] Error: MapViewOfFile failed (%lu)\n", GetLastError());
		CloseHandle(shm_handle);
		shm_handle = NULL;
		return NULL;
	}

	return shm;
}

void blick_os_host_spawn_viewer(void) {
	STARTUPINFOA si = {0};
	PROCESS_INFORMATION pi = {0};
	si.cb = sizeof(si);

	if (!CreateProcessA("blick_viewer.exe", NULL, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi)) {
		fprintf(stderr, "[BLICK] Error: Failed to spawn viewer (%lu)\n", GetLastError());
		return;
	}

	viewer_process = pi.hProcess;
	CloseHandle(pi.hThread); // We don't need the primary thread handle
}

void blick_os_host_shutdown(blick_shm_header* shm) {
	if (viewer_process) {
		TerminateProcess(viewer_process, 0);
		CloseHandle(viewer_process);
		viewer_process = NULL;
	}
	if (shm) {
		UnmapViewOfFile(shm);
	}
	if (shm_handle) {
		CloseHandle(shm_handle);
		shm_handle = NULL;
	}
}

blick_shm_header* blick_os_viewer_open_shm(void) {
	if (!shm_handle) {
		shm_handle = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, BLICK_SHM_NAME);
		if (!shm_handle) return NULL;
	}

	// Mapping with FILE_MAP_ALL_ACCESS because the viewer updates the 'reading_idx' atomic
	blick_shm_header* shm = (blick_shm_header*)MapViewOfFile(shm_handle, FILE_MAP_ALL_ACCESS,
															 0, 0, sizeof(blick_shm_header));
	if (!shm) return NULL;

	return shm;
}

void blick_os_viewer_close_shm(blick_shm_header* shm) {
	if (shm) {
		UnmapViewOfFile(shm);
	}
	if (shm_handle) {
		CloseHandle(shm_handle);
		shm_handle = NULL;
	}
}

void blick_os_sleep_ms(uint32_t ms) {
	Sleep(ms);
}
