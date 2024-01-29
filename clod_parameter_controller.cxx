#include "clod_parameter_controller.h"
#include <algorithm>


pct::clod_parameter_controller::clod_parameter_controller() : 
	targeted_visible_points(0), 
	min_clod(0.01)
{
}

void pct::clod_parameter_controller::control(cgv::render::clod_point_render_style& style, int last_reduce_point_count, float dt)
{
	float error = last_reduce_point_count - targeted_visible_points;
	float c = dt*(error / last_reduce_point_count);

	style.CLOD = std::max(min_clod, style.CLOD*(1.f+c));
}
