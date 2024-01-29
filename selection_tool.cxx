#include "selection_tool.h"

#include <cgv/render/shader_program.h>

box_selection_tool::box_selection_tool()
{
	clear_points();
}

cgv::render::render_types::vec3& box_selection_tool::point(int index)
{
	return points[index];
}

void box_selection_tool::clear_points()
{
	points[0] = vec3(std::numeric_limits<float>::infinity());
	points[1] = vec3(std::numeric_limits<float>::infinity());
	phase_p = box_selection_phase::NO_POINT_CONFIRMED;
	draw_box_p = false;
	is_selecting_orientation = false;
}

box_selection_phase& box_selection_tool::phase()
{
	return phase_p;
}

cgv::render::render_types::quat& box_selection_tool::orientation()
{
	return box_orientation;
}

cgv::math::fvec<float,3>& box_selection_tool::translation()
{
	return box_position;
}

cgv::render::render_types::box3 &box_selection_tool::box()
{
	return selection_box;
}

float& box_selection_tool::corner_node_radius()
{
	return sphere_style.radius;
}

void box_selection_tool::update_selection()
{	
	vec3 centroid = 0.5f * (points[0] + points[1]);
	box_position = centroid;
	vec3 point_a = point(0)-centroid;
	vec3 point_b = point(1)-centroid;

	orientation().inverse_rotate(point_a);
	orientation().inverse_rotate(point_b);
	
	selection_box = box3();
	selection_box.add_point(point_a);
	selection_box.add_point(point_b);

	draw_box_p = num_points_defined() == 2;
	if (draw_box_p)
		update_corner_nodes();
}

int box_selection_tool::num_points_defined()
{
	int ret = 0;
	for (auto& p : points) {
		if (p.x() != std::numeric_limits<float>::infinity())
			++ret;
	}
	return ret;
}

void box_selection_tool::update_orientation(const bool is_changable)
{
	is_selecting_orientation = is_changable;
}

bool box_selection_tool::init(cgv::render::context& ctx)
{
	sphere_style.radius = 0.05f;
	sphere_style.material.set_transparency(0.2f);
	sphere_style.surface_color = rgb(0.38539, 0.212664, 0.725346);
	box_style.material.set_transparency(0.4f);
	box_wire_style.reference_line_width = 0.1;
	arrow_style.length_scale = 0.8f;
	arrow_style.radius_relative_to_length = 0.01f;
	arrow_style.radius_lower_bound = 0.01f;
	arrow_style.head_radius_scale = 2.0f;
	pre_clr.push_back(rgb(1.0, 0.0, 0.0));
	pre_clr.push_back(rgb(0.0, 1.0, 0.0));
	pre_clr.push_back(rgb(0.0, 0.0, 1.0));
	ref_sphere_renderer(ctx,1);
	ref_box_renderer(ctx,1);
	ref_box_wire_renderer(ctx,1);
	cgv::render::ref_arrow_renderer(ctx, 1);
	return true;
}

void box_selection_tool::clear(cgv::render::context& ctx)
{
	ref_sphere_renderer(ctx,-1);
	ref_box_renderer(ctx, -1);
	ref_box_wire_renderer(ctx, -1);
	cgv::render::ref_arrow_renderer(ctx, -1);
}

void box_selection_tool::draw(cgv::render::context& ctx)
{
	auto& sr = ref_sphere_renderer(ctx);
	auto& br = ref_box_renderer(ctx);
	auto& wbr = ref_box_wire_renderer(ctx);
	auto& ar = cgv::render::ref_arrow_renderer(ctx);

	if (phase() != box_selection_phase::NO_POINT_CONFIRMED) {
		glDisable(GL_CULL_FACE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (phase() == box_selection_phase::SECOND_POINT_CONFIRMED) {
			//wbr.set_render_style(box_wire_style);
			//glLineWidth(0.3f);
			//wbr.set_line_width(ctx, 0.5f);
			wbr.set_box_array(ctx, &selection_box, 1);
			wbr.set_translation_array(ctx, &box_position, 1);
			wbr.set_rotation_array(ctx, &box_orientation, 1);
			wbr.render(ctx, 0, 1);

			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, corner_node_positions.data(), 8);
			sr.render(ctx, 0, 8);
		}
		else if (phase() == box_selection_phase::FIRST_POINT_CONFIRMED) {
			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, &point(0), 1);
			sr.render(ctx, 0, 1);

			if (is_selecting_orientation) {
				for (int i = 0; i < 3; ++i) {
					pre_crd.push_back(vec3(box_orientation.get_homogeneous_matrix().col(i).x(), box_orientation.get_homogeneous_matrix().col(i).y(), box_orientation.get_homogeneous_matrix().col(i).z()));

					ar.set_render_style(arrow_style);
					ar.set_position_array(ctx, &point(0), 1);
					ar.set_color_array(ctx, &pre_clr.at(i), 1);
					ar.set_direction_array(ctx, &pre_crd.at(i), 1);
					ar.render(ctx, 0, 1);
				}
				pre_crd.clear();
			}

			if (draw_box_p) { //draw_box_p is set by update_selection when two points != infinity are defined
				br.set_render_style(box_style);
				br.set_box_array(ctx, &selection_box, 1);
				br.set_translation_array(ctx, &box_position, 1);
				br.set_rotation_array(ctx, &box_orientation, 1);
				br.render(ctx, 0, 1);
			}
		}

		glDisable(GL_BLEND);
		glEnable(GL_CULL_FACE);
	}
}

void box_selection_tool::set_corner(int i, const vec3& p)
{
	//transform into box coordinate system
	vec3 centroid = 0.5f * (points[0] + points[1]);
	vec3 point_a = point(0) - centroid;
	vec3 point_b = point(1) - centroid;
	orientation().inverse_rotate(point_a);
	orientation().inverse_rotate(point_b);

	//
	{
		//apply changes
		vec3 c = p - centroid;
		orientation().inverse_rotate(c);
		int bit = 1;
		for (unsigned int dim = 0; dim < 3; ++dim, bit *= 2)
			if (i & bit)
				point_b(dim) = c(dim);
			else
				point_a(dim) = c(dim);
		//back transform into global coordinate system
		orientation().rotate(point_a);
		orientation().rotate(point_b);
		point(0) = point_a + centroid;
		point(1) = point_b + centroid;
	}
	//recompute everything else
	update_selection();
}

const std::array<cgv::math::fvec<float, 3>, 8>& box_selection_tool::get_corners()
{
	return corner_node_positions;
}

void box_selection_tool::update_corner_nodes()
{
	
	vec3 centroid = 0.5f * (points[0] + points[1]);
	vec3 point_a = point(0) - centroid;
	vec3 point_b = point(1) - centroid;
	orientation().inverse_rotate(point_a);
	orientation().inverse_rotate(point_b);

	for (int i = 0; i < 8; ++i) {
		//generating corners here to have them in a fixed order
		vec3 c = point_a;
		int bit = 1;
		for (unsigned int dim = 0; dim < 3; ++dim, bit *= 2)
			if (i & bit)
				c(dim) = point_b(dim);

		orientation().rotate(c);
		corner_node_positions[i] = c+centroid;
	}
}
