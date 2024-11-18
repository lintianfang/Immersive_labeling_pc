#pragma once
#include <vector>
#include <array>
#include <cgv/render/context.h>
#include <cgv_gl/renderer.h>

namespace pct {

	class gpu_stopwatch {
		std::array<GLuint, 2> timer_queries;
		bool waiting_for_completion;
	public:
		gpu_stopwatch();

		void start_gpu_timer();

		void stop_gpu_timer();

		bool poll_gpu_timer(GLuint64& result);

		void init();

		void clear();
	};
}