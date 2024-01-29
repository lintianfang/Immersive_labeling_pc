#pragma once
#include<cgv/render/render_types.h>

namespace pct {

	struct waypoint_navigation_gui_creator;

	struct waypoint_navigation : 
		public cgv::render::render_types
	{
		bool loop_path = true;
		float speed = 0.7;
		std::vector<vec3> path;
		int next_way_point = 0;

		inline void set_speed(float s) {
			this->speed = s;
		}
		/**
		*	@param position: last position
		*	@param dt: passed time since last position update in seconds
		*	@return new position simulating a move to the next waypoint based on given position*/
		vec3 next_position(vec3 position, float dt);

		void set_path(vec3* path, size_t size);
	};

}