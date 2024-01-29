#pragma once
#include <cgv/math/pose.h>
#include <point_cloud/point_cloud.h>
#include <cgv_gl/clod_point_renderer.h>


//contains the positioning information like it is stored in a point_cloud object
struct pointcloud_transformation_state {
	
	cgv::math::fvec<float,3> position;
	cgv::math::fvec<float,3> rotation;
	float scale;

	pointcloud_transformation_state() = default;

	pointcloud_transformation_state(point_cloud& source_pc) {
		store_state(source_pc);
	}

	inline void store_state(point_cloud& source_pc) {
		position = source_pc.ref_point_cloud_position();
		rotation = source_pc.ref_point_cloud_rotation();
		scale = source_pc.ref_point_cloud_scale();
	}

	inline void restore_state(point_cloud& source_pc) {
		source_pc.ref_point_cloud_position() = position;
		source_pc.ref_point_cloud_rotation() = rotation;
		source_pc.ref_point_cloud_scale() = scale;
	}
};