#pragma once
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/box_wire_renderer.h>
#include <cgv_gl/clod_point_renderer.h>
#include <cgv_gl/arrow_renderer.h>
#include "label_shader_manager.h"
#include "point_labels.h"



enum class box_selection_phase {
	NO_POINT_CONFIRMED,
	FIRST_POINT_CONFIRMED,
	SECOND_POINT_CONFIRMED,
};

//this class deals with the visuals of the selection tool
class selection_tool {
	static uint32_t constexpr selection_flag = (uint32_t)point_label_group::SELECTED_BIT;
};

/*
a special selection tool that is inspired by pixel selection technics from image editing programs
users can span an box that can assign a label to all points inside
the boxes corners can be used to further adapt the selected area
a 
*/
class box_selection_tool : public selection_tool {

	using box3 = cgv::box3;
	using vec3 = cgv::vec3;
	using quat = cgv::quat;

	box3 selection_box;
	quat box_orientation;
	vec3 box_position;
	rgb corner_color;

	cgv::render::box_render_style box_style;
	cgv::render::sphere_render_style sphere_style;
	cgv::render::arrow_render_style arrow_style;
	cgv::render::box_wire_render_style box_wire_style;
	
	std::array<vec3,8> corner_node_positions;
	bool draw_box_p = false;
	bool is_selecting_orientation = false;

	//selection points given in global coordinates
	std::array<vec3, 2> points;
	
	box_selection_phase phase_p;

	std::vector<vec3> pre_crd;
	std::vector<rgb> pre_clr;
public:
	using LODPoint = cgv::render::clod_point_renderer::Point;

	box_selection_tool();

	vec3& point(int index);
	void clear_points();

	box_selection_phase& phase();

	quat& orientation();
	vec3& translation();
	box3& box();

	float& corner_node_radius();

	//makes the class react on changes in points and orientation
	void update_selection();

	void update_orientation(const bool is_changable);

	int num_points_defined();

	bool init(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);
	//Sets the position of a corner and adjusts also other corners to keep the box shape. This automatically updates the selection.
	void set_corner(int i, const vec3& p);

	const std::array<cgv::math::fvec<float,3>, 8>& get_corners();
protected:
	void update_corner_nodes();
};