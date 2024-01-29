#include "gpu_stopwatch.h"

pct::gpu_stopwatch::gpu_stopwatch()
{
	timer_queries[0] = 0;
	timer_queries[1] = 0;
	waiting_for_completion = false;
}

void pct::gpu_stopwatch::start_gpu_timer()
{
	if (!waiting_for_completion) {
		glQueryCounter(timer_queries[0], GL_TIMESTAMP);
	}
}

void pct::gpu_stopwatch::stop_gpu_timer()
{
	if (!waiting_for_completion) {
		glQueryCounter(timer_queries[1], GL_TIMESTAMP);
		waiting_for_completion = true;
	}
}

bool pct::gpu_stopwatch::poll_gpu_timer(GLuint64& result)
{
	if (waiting_for_completion) {
		GLint stop_timer_available = 0;
		glGetQueryObjectiv(timer_queries[1],
			GL_QUERY_RESULT_AVAILABLE, &stop_timer_available);
		if (stop_timer_available) {
			GLuint64 data[2] = { 0 ,0 };
			glGetQueryObjectui64v(timer_queries[0], GL_QUERY_RESULT, &data[0]);
			glGetQueryObjectui64v(timer_queries[1], GL_QUERY_RESULT, &data[1]);

			GLuint64 diff_labeling_gpu = data[1] - data[0];
			waiting_for_completion = false;
			result = diff_labeling_gpu;
			return true;
		}
	}
	return false;
}

void pct::gpu_stopwatch::init()
{
	glGenQueries(2, timer_queries.data());
}

void pct::gpu_stopwatch::clear()
{
	glDeleteQueries(2, timer_queries.data());
}
