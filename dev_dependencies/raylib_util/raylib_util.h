#ifndef RAYLIB_UTIL_H
#define RAYLIB_UTIL_H

#include <raylib.h>

#include <stdint.h>

/* -------------------------------------------------------------------------------------------------
Helper functions for raylib
------------------------------------------------------------------------------------------------- */

// look around: hold right mouse button + move mouse
// fly around: WASD + QE
void update_fly_camera(Camera3D* camera);

Model create_raylib_model(float* vertices, int vertexCount, uint32_t* indices, int indexCount);

static void set_window_top_left(int margin_x, int margin_y) {
	int monitor = GetCurrentMonitor();
	Vector2 mon_pos = GetMonitorPosition(monitor);
	SetWindowPosition((int)mon_pos.x + margin_x, (int)mon_pos.y + margin_y);
}

static void set_window_top_right(int margin_x, int margin_y) {
	int monitor = GetCurrentMonitor();
	int mon_width = GetMonitorWidth(monitor);
	Vector2 mon_pos = GetMonitorPosition(monitor);
	SetWindowPosition((int)mon_pos.x + mon_width - 1280 - margin_x, (int)mon_pos.y + margin_y);
}

static void set_window_bottom_left(int margin_x, int margin_y) {
	int monitor = GetCurrentMonitor();
	Vector2 mon_pos = GetMonitorPosition(monitor);
	int mon_height = GetMonitorHeight(monitor);

	// PROBLEM 1: This returns the content area size, not the full window size with titlebar!
	int win_height = GetScreenHeight();

	SetWindowPosition((int)mon_pos.x + margin_x,
					  (int)mon_pos.y + mon_height - win_height - margin_y);
}

static void set_window_bottom_right(int margin_x, int margin_y) {
	int monitor = GetCurrentMonitor();
	Vector2 mon_pos = GetMonitorPosition(monitor);
	int mon_width = GetMonitorWidth(monitor);
	int mon_height = GetMonitorHeight(monitor);

	int win_width = GetScreenWidth(); // Fixes the hardcoded 1280
	int win_height = GetScreenHeight();

	SetWindowPosition((int)mon_pos.x + mon_width - win_width - margin_x,
					  (int)mon_pos.y + mon_height - win_height - margin_y);
}

#endif // RAYLIB_UTIL_H
