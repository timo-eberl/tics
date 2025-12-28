#include "game_loop.h"

#include <chrono>
#include <thread>

GameLoop::GameLoop(
		std::function<void(float, float)> variable_update, std::function<void(float)> fixed_update,
		const float fixed_update_delta,
		const float variable_update_delta_min, const float variable_update_delta_max
	)
		: m_variable_update(variable_update), m_fixed_update(fixed_update),
		fixed_delta(fixed_update_delta),
		variable_delta_min(variable_update_delta_min),
		variable_delta_max(variable_update_delta_max),
		m_last_update_start( std::chrono::high_resolution_clock::now() ),
		m_fixed_delay(0.0f)
	{ }

void GameLoop::update() const {
	auto start_time = std::chrono::high_resolution_clock::now();

	// delta of last update
	std::chrono::duration<float> delta = start_time - m_last_update_start;

	if (delta > variable_delta_max) {
		delta = variable_delta_max;
	}

	// if we are too early, we wait until variable_delta_min is reached
	if (delta < variable_delta_min) {
		std::chrono::duration<float> wait_time = variable_delta_min - delta;
		auto destination_time = start_time
			+ std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time);

		switch (idle_method) {
			case BUSY_SLEEP:
				while (std::chrono::high_resolution_clock::now() < destination_time) { }
				break;
			case THREAD_SLEEP:
				std::this_thread::sleep_until(destination_time);
				break;
		}

		auto now = std::chrono::high_resolution_clock::now();
		delta = now - m_last_update_start;
		m_last_update_start = now;
	}
	else {
		m_last_update_start = std::chrono::high_resolution_clock::now();
	}
	m_fixed_delay += delta;

	while (m_fixed_delay >= fixed_delta) {
		m_fixed_update(fixed_delta.count());
		m_fixed_delay -= fixed_delta;
	}

	m_variable_update(delta.count(), m_fixed_delay.count());
}
