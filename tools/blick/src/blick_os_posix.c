#include "blick_os.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static int shm_fd = -1;
static pid_t viewer_pid = -1;

blick_shm_header* blick_os_host_init_shm(void) {
	shm_fd = shm_open(BLICK_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("[BLICK] Error: shm_open failed");
		return NULL;
	}

	if (ftruncate(shm_fd, sizeof(blick_shm_header)) == -1) {
		perror("[BLICK] Error: ftruncate failed");
		return NULL;
	}

	blick_shm_header* shm = mmap(0, sizeof(blick_shm_header), PROT_READ | PROT_WRITE,
								 MAP_SHARED, shm_fd, 0);
	
	if (shm == MAP_FAILED) {
		perror("[BLICK] Error: mmap failed");
		return NULL;
	}

	return shm;
}

void blick_os_host_spawn_viewer(const char* viewer_path) {
	pid_t pid = fork();
	if (pid == 0) {
		if (getppid() == 1) exit(1);
		execl(viewer_path, viewer_path, NULL);
		perror("[BLICK] Error: Failed to spawn viewer");
		exit(1);
	}
	viewer_pid = pid;
}

void blick_os_host_shutdown(blick_shm_header* shm) {
	if (viewer_pid > 0) {
		kill(viewer_pid, SIGTERM);
		viewer_pid = -1;
	}
	if (shm) munmap(shm, sizeof(blick_shm_header));
	if (shm_fd != -1) {
		close(shm_fd);
		shm_fd = -1;
	}
	shm_unlink(BLICK_SHM_NAME);
}

blick_shm_header* blick_os_viewer_open_shm(void) {
	if (shm_fd == -1) {
		shm_fd = shm_open(BLICK_SHM_NAME, O_RDWR, 0666);
		if (shm_fd == -1) return NULL;
	}

	// Mapping with PROT_WRITE because the viewer updates the 'reading_idx' atomic
	blick_shm_header* shm = mmap(0, sizeof(blick_shm_header), PROT_READ | PROT_WRITE,
								 MAP_SHARED, shm_fd, 0);
	
	if (shm == MAP_FAILED) return NULL;

	return shm;
}

void blick_os_viewer_close_shm(blick_shm_header* shm) {
	if (shm) munmap(shm, sizeof(blick_shm_header));
	if (shm_fd != -1) {
		close(shm_fd);
		shm_fd = -1;
	}
}

void blick_os_sleep_ms(uint32_t ms) {
	usleep(ms * 1000);
}
