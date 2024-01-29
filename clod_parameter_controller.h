#pragma once
#include <cgv_gl/clod_point_renderer.h>

namespace pct {

	// keeps changing clod parameters until the desired number of points is visible
	struct clod_parameter_controller {
		// desired number of visible points
		int targeted_visible_points;
		// minimum clod setting
		float min_clod;

		clod_parameter_controller();

		/** @param style : clod render style to modify
			@param last_reduced_point_count : result of the last full reduction pass
			@param dt: passed time since last call in seconds */
		void control(cgv::render::clod_point_render_style& style, int last_reduce_point_count, float dt);
	};

}