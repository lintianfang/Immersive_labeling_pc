#include "world.h"

#include <random>

#include "intersection.h"

namespace pc_cleaning_tool {


	void world::construct_table(float tw, float td, float th, float tW)
	{
		// construct table
		rgb table_clr(0.3f, 0.2f, 0.0f);
		boxes.push_back(box3(
			vec3(-0.5f * tw - 2 * tW, th - tW, -0.5f * td - 2 * tW),
			vec3(0.5f * tw + 2 * tW, th, 0.5f * td + 2 * tW)));
		box_colors.push_back(table_clr);

		boxes.push_back(box3(vec3(-0.5f * tw, 0, -0.5f * td), vec3(-0.5f * tw - tW, th - tW, -0.5f * td - tW)));
		boxes.push_back(box3(vec3(-0.5f * tw, 0, 0.5f * td), vec3(-0.5f * tw - tW, th - tW, 0.5f * td + tW)));
		boxes.push_back(box3(vec3(0.5f * tw, 0, -0.5f * td), vec3(0.5f * tw + tW, th - tW, -0.5f * td - tW)));
		boxes.push_back(box3(vec3(0.5f * tw, 0, 0.5f * td), vec3(0.5f * tw + tW, th - tW, 0.5f * td + tW)));
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
	}

	void world::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
	{
		// construct floor
		ref_floor_box() = (box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
		ref_floor_box_clr() = (rgb(0.2f, 0.2f, 0.2f));

		if (walls) {
			// construct walls
			boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
			boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

			boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
		}
		if (ceiling) {
			// construct ceiling
			boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
		}
	}

	void world::construct_environment(float s, float ew, float ed, float w, float d, float h)
	{
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0, 1);
		unsigned n = unsigned(ew / s);
		unsigned m = unsigned(ed / s);
		float ox = 0.5f * float(n) * s;
		float oz = 0.5f * float(m) * s;
		for (unsigned i = 0; i < n; ++i) {
			float x = i * s - ox;
			for (unsigned j = 0; j < m; ++j) {
				float z = j * s - oz;
				if (fabsf(x) < 0.5f * w && fabsf(x + s) < 0.5f * w && fabsf(z) < 0.5f * d && fabsf(z + s) < 0.5f * d)
					continue;
				float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
				boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
				constexpr float hue_radius = 0.3f;
				constexpr float hue_center = 0.4f;
				rgb color = cgv::media::color<float, cgv::media::HLS>(fmod(hue_center + hue_radius * distribution(generator), 1.f), 0.1f * distribution(generator) + 0.15f, 0.6f);
				box_colors.push_back(color);
			}
		}
	}

	void world::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
	{
		renderer_data_outdated = true;
		if (show_floor)
			construct_room(w, d, h, W, false, false);
		if (show_table)
			construct_table(tw, td, th, tW);
		if (show_environment)
			construct_environment(0.30f, 3 * w, 3 * d, w, d, h);
	}

	void world::clear_scene()
	{
		boxes.resize(1);
		box_colors.resize(1);
	}

	world::world()
	{
		//place empty floor box on position 0 
		clear_scene();
		show_environment = false;
		show_table = true;
		show_floor = true;
	}

	bool world::init(cgv::render::context& ctx)
	{
		aam.init(ctx);
		world_renderer.init(ctx);
		world_renderer.set_attribute_array_manager(ctx,&aam);
		return true;
	}

	void world::clear(cgv::render::context& ctx)
	{
		world_renderer.clear(ctx);
	}

	void world::draw(cgv::render::context& ctx)
	{
		if (renderer_data_outdated)
			prepare_renderer(ctx);
		world_renderer.render(ctx, show_floor ? 0 : 1, boxes.size());
	}

	void world::prepare_renderer(cgv::render::context& ctx)
	{
		
		world_renderer.set_render_style(style);
		world_renderer.set_box_array(ctx, boxes);
		world_renderer.set_color_array(ctx, box_colors);
		renderer_data_outdated = false;
	}

	box3& world::ref_floor_box()
	{
		return boxes[0];
	}

	rgb& world::ref_floor_box_clr()
	{
		return box_colors[0];
	}

	ray_box_intersection_information world::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
	{
		ray_box_intersection_information intersection;
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin, direction,
			ref_floor_box(),
			t_result, p_result, n_result, 0.000001f)) {
			// store intersection information
			intersection.points.push_back(p_result);
			//intersection_box_indices.push_back((int)i);
			intersection.controller_indices.push_back(ci);
			intersection.colors.push_back(color);
		}
		return intersection;
	}


	void ray_box_intersection_information::clear()
	{
		points.clear();
		controller_indices.clear();
		colors.clear();
	}

}