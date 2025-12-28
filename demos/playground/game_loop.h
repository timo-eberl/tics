#pragma once

#include <chrono>
#include <functional>

class GameLoop {
public:
	GameLoop(
		std::function<void(float, float)> variable_update, std::function<void(float)> fixed_update,
		const float fixed_update_delta,
		const float variable_update_delta_min_seconds = 0.0f,
		// limit the max delta time, otherwise slow fixed updates can increase the delta time indefinitely
		// if the max delta time is not met, the game time will slow down
		const float variable_update_delta_max_seconds = 0.2f
	);
	~GameLoop() = default;

	const std::chrono::duration<float> fixed_delta;
	std::chrono::duration<float> variable_delta_min;
	std::chrono::duration<float> variable_delta_max;

	// tradeoff: busy sleep is more reliable,
	// thread sleep results in less cpu utilization and is OS dependent
	enum IdleMethod { BUSY_SLEEP, THREAD_SLEEP };

	IdleMethod idle_method = BUSY_SLEEP;

	void update() const;
private:
	const std::function<void(float, float)> m_variable_update;
	const std::function<void(float)> m_fixed_update;
	std::chrono::high_resolution_clock::time_point mutable m_last_update_start;
	std::chrono::duration<float> mutable m_fixed_delay;
};
