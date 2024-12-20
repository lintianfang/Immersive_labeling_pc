#pragma once
#include <cgv/math/vec.h>
#include <cgv/math/fvec.h>
#include <cgv/math/mat.h>
#include <cgv/math/fmat.h>
#include <cgv/math/quaternion.h>

//types in cgv namespace
using ivec2 = cgv::ivec2;
using ivec3 = cgv::ivec3;
using dvec2 = cgv::dvec2;
using dvec4 = cgv::dvec4;
using vec2 = cgv::vec2;
using vec3 = cgv::vec3;
using vec4 = cgv::vec4;
using mat3 = cgv::mat3;
using mat4 = cgv::mat4;
using mat34 = cgv::mat34;
using dmat4 = cgv::dmat4;
using dquat = cgv::dquat;


namespace pct {

	struct waypoint_navigation_gui_creator;

	struct waypoint_navigation
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