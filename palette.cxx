#include "palette.h"
#include "label_shader_manager.h"
#include "util.h"
#include <cgv/math/ftransform.h>
#include "octree.h"

namespace vrui {
	size_t palette::num_objects() const
	{
		return palette_object_visibility.size();
	}

	cgv::render::shader_program& palette::instanced_rendering_prog()
	{
		static cgv::render::shader_program prog;
		return prog;
	}


	palette::palette()
	{
		point_cloud_style.screen_aligned = true;
		point_cloud_style.point_size = 0.1;
		point_cloud_style.measure_point_size_in_pixel = false;
		palette_changed = true;
		last_triggered_object = -1;

		ctx_ptr = nullptr;
	}

	palette::~palette()
	{
	}

	int palette::add_object(const PaletteObject shape, const cgv::vec3& position, const cgv::rgba& color, const PaletteObjectGroup pog)
	{
		int id = palette_object_shapes.size();
		palette_object_shapes.emplace_back(shape);
		palette_object_positions.emplace_back(position);
		palette_object_colors.emplace_back(color);
		palette_object_data.emplace_back(nullptr);
		palette_object_visibility.emplace_back(true);
		palette_object_groups.push_back(pog);
		palette_text_label_ids.push_back(-1);
		switch (pog) {
		case POG_TOP_TOOLBAR:
			positions_in_group.push_back(top_toolbar_object_group.size());
			top_toolbar_object_group.push_back(id);
			break;
		case POG_LEFT_TOOLBAR:
			positions_in_group.push_back(left_toolbar_object_group.size());
			left_toolbar_object_group.push_back(id);
			break;
		default:
			positions_in_group.push_back(num_objects() - 1 - top_toolbar_object_group.size() - left_toolbar_object_group.size());
		}
		return id;
	}

	int palette::add_pointcloud(const cgv::vec3* point_positions, const cgv::rgb8* point_colors, const size_t size, const cgv::vec3& position, const cgv::quat& rotation)
	{
		int pc_id = point_clouds_point_positions.size();
		point_cloud_positions.push_back(position);
		point_cloud_rotations.push_back(rotation);
		point_cloud_scales.push_back(1.f);
		point_clouds_point_positions.emplace_back(size);
		point_clouds_point_colors.emplace_back(size);
		point_cloud_array_managers.emplace_back();

		memcpy(point_clouds_point_positions[pc_id].data(), point_positions, size * sizeof(cgv::vec3));
		memcpy(point_clouds_point_colors[pc_id].data(), point_colors, size * sizeof(cgv::rgba8));
		return pc_id;
	}
	
	void palette::replace_pointcloud(const int pc_id, const cgv::vec3* point_positions, const cgv::rgb8* point_colors, const size_t size)
	{
		replace_pointcloud(pc_id, point_positions, point_colors, size, point_cloud_positions[pc_id], point_cloud_rotations[pc_id]);
		palette_changed = true;
	}


	void palette::replace_pointcloud(const int pc_id, const cgv::vec3* point_positions, const cgv::rgb8* point_colors, const size_t size, const cgv::vec3& position, const cgv::quat& rotation)
	{
		point_cloud_positions[pc_id] = position;
		point_cloud_rotations[pc_id] = rotation;
		point_cloud_scales[pc_id] = 1.f;
		point_clouds_point_positions[pc_id].resize(size);
		point_clouds_point_colors[pc_id].resize(size);
		if (size > 0) {
			memcpy(point_clouds_point_positions[pc_id].data(), point_positions, size * sizeof(vec3));
			memcpy(point_clouds_point_colors[pc_id].data(), point_colors, size * sizeof(cgv::rgb8));
		}
		palette_changed = true;
	}

	void palette::set_function(object_funct& funct)
	{
		palette_picking_function = funct;
	}

	const std::vector<int>& palette::left_toolbar()
	{
		return left_toolbar_object_group;
	}


	int palette::pick_object(const cgv::vec3& pnt, const cgv::mat34& palette_pose, float& dist)
	{
		vec3 picking_position_rhand = pnt;
		dist = std::numeric_limits<float>::max();
		for (int i = 0; i < palette_object_positions.size(); i++) {
			vec3 object_position = palette_pose * palette_object_positions[i].lift();
			float cur_dist = (object_position - picking_position_rhand).length();
			if (cur_dist < p_sphere_style.radius) {
				dist = cur_dist;
				return i;
			}
		}
		return -1;
	}

	int palette::trigger_object(const cgv::vec3& pnt, const cgv::mat34& palette_pose, float& dist, bool only_once_between_objects)
	{
		// check for picked object
		int nearest_palette_idx = pick_object(pnt, palette_pose, dist);
		//call the palette objects function (does nothing if an invalid id is passed)
		if (only_once_between_objects) {
			if (last_triggered_object != nearest_palette_idx) {
				trigger_function(nearest_palette_idx);
			}
		}
		else {
			trigger_function(nearest_palette_idx);
		}
		
		last_triggered_object = nearest_palette_idx;
		return nearest_palette_idx;
	}

	void palette::trigger_function(int id)
	{
		if (object_id_is_valid(id)) {
			picked_object po(*this, id);
			palette_picking_function(po);
		}
	}

	void palette::set_top_toolbar_visibility(const bool visibility)
	{
		if (top_toolbar_visible != visibility) {
			for (int i : top_toolbar_object_group) {
				palette_object_visibility[i] = visibility;
			}
			top_toolbar_visible = visibility;
			palette_changed = true;
		}
	}

	void palette::set_palette_changed()
	{
		palette_changed = true;
	}

	void palette::init_frame(cgv::render::context& ctx)
	{
		palette_text_labels.init_frame(ctx);
	}

	bool palette::object_id_is_valid(const int id) const
	{
		return id >= 0 && id < palette_object_colors.size();
	}

	bool palette::pointcloud_id_is_valid(const int id) const
	{
		return id >= 0 && id < point_cloud_array_managers.size();
	}

	void palette::init(cgv::render::context& ctx)
	{
		ctx_ptr = &ctx;
		
		palette_box_renderer.init(ctx);
		palette_sphere_renderer.init(ctx);
		palette_plane_renderer.init(ctx);
		palette_box_wire_renderer.init(ctx);
		palette_box_plane_renderer.init(ctx);

		palette_spheres.init(ctx);
		palette_boxes.init(ctx);
		palette_planes.init(ctx);
		palette_box_frames.init(ctx);
		palette_box_planes.init(ctx);

		palette_plane_renderer.enable_attribute_array_manager(ctx, palette_planes);
		palette_sphere_renderer.enable_attribute_array_manager(ctx, palette_spheres);
		palette_box_renderer.enable_attribute_array_manager(ctx, palette_boxes);
		palette_box_wire_renderer.enable_attribute_array_manager(ctx, palette_box_frames);
		palette_box_plane_renderer.enable_attribute_array_manager(ctx, palette_box_planes);

		if (!instanced_rendering_prog().is_created()) {
			instanced_rendering_prog().build_program(ctx, "pallete_mesh.glpr", true);
		}
		
		glCreateBuffers(1, &sphere_frame_point_buffer);
		glCreateBuffers(1, &sphere_frame_offset_buffer);
		glCreateBuffers(1, &sphere_frame_color_buffer);
		glGenVertexArrays(1, &sphere_frame_vao);
		{
			auto& prog = instanced_rendering_prog();

			GLuint position_ix = prog.get_position_index();
			GLuint color_ix = prog.get_color_index();
			GLuint offset_ix = prog.get_attribute_location(ctx, "offset");
			
			glBindVertexArray(sphere_frame_vao);
			glEnableVertexAttribArray(position_ix);
			glBindBuffer(GL_ARRAY_BUFFER, sphere_frame_point_buffer);
			glVertexAttribPointer(position_ix, 3, GL_FLOAT, false, 0, nullptr);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glEnableVertexAttribArray(color_ix);
			glBindBuffer(GL_ARRAY_BUFFER, sphere_frame_color_buffer);
			glVertexAttribPointer(color_ix, 4, GL_FLOAT, false, 0, nullptr);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(color_ix, 1);

			glEnableVertexAttribArray(offset_ix);
			glBindBuffer(GL_ARRAY_BUFFER, sphere_frame_offset_buffer);
			glVertexAttribPointer(offset_ix, 4, GL_FLOAT, false, 0, nullptr);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(offset_ix, 1);
			glBindVertexArray(0);
		}

		cgv::render::ref_point_renderer(ctx, 1);
	}

	cgv::rgba& palette::object_color(const int id)
	{
		return palette_object_colors[id];
	}

	cgv::render::point_render_style& palette::point_style()
	{
		return point_cloud_style;
	}

	cgv::render::sphere_render_style& palette::sphere_style()
	{
		return p_sphere_style;
	}

	cgv::render::box_render_style& palette::box_style()
	{
		return p_box_style;
	}

	cgv::render::box_render_style& palette::plane_style()
	{
		return p_plane_style;
	}

	cgv::render::box_wire_render_style& palette::wire_box_style()
	{
		return p_box_wire_style;
	}

	cgv::render::box_render_style& palette::box_plane_style()
	{
		return p_box_plane_style;
	}

	void palette::clear(cgv::render::context& ctx)
	{
		palette_box_renderer.clear(ctx);
		palette_sphere_renderer.clear(ctx);
		palette_plane_renderer.clear(ctx);
		palette_box_wire_renderer.clear(ctx);
		palette_box_plane_renderer.clear(ctx);

		glDeleteBuffers(1, &sphere_frame_point_buffer);
		glDeleteBuffers(1, &sphere_frame_offset_buffer);
		glDeleteBuffers(1, &sphere_frame_color_buffer);
		glDeleteVertexArrays(1, &sphere_frame_vao);
		
		cgv::render::ref_point_renderer(ctx, -1);

		for (auto& m : point_cloud_array_managers) {
			m.destruct(ctx);
		}
	}


	void palette::init_text_labels(cgv::render::context& ctx, vr_view_interactor* vr_view_ptr)
	{
		palette_text_labels.set_vr_view(vr_view_ptr);
		palette_text_labels.init(ctx);
	}


	void palette::set_colors(const std::vector<cgv::rgba>& palette_colors)
	{
		palette_changed = true;

		//update colors
		for (int i = 0; i < palette_colors.size(); i++)
			palette_object_colors[i] = palette_colors[i];
	}

	//text labels

	void palette::set_label_text(unsigned int obj_ix, std::string txt)
	{
		//original = 0.05
		static double constexpr step_width = 0.10;
		
		auto palette_label_offset = [&](const vec3& palette_object_position, const float& step_width) -> vec3 {
			return palette_object_position + vec3(0.0, 0.7071068 * 0.5 * step_width, 0.7071068 * 0.5 * step_width);
		};

		quat delete_label_ori = quat(0.7071068, -0.7071068, 0, 0);
		float delete_label_scale = 0.2f;

		auto& li = palette_text_label_ids[obj_ix];
		if (li == -1) {
			li = palette_text_labels.add_label(txt, cgv::rgba(0.5, 0.5, 0.5, 0.75));
		}
		else {
			palette_text_labels.set_label(li, txt, cgv::rgba(0.5, 0.5, 0.5, 0.75));
			//palette_text_labels.update_label_text(li, name);
		}

		//palette_text_labels.place_label(li, palette_label_offset(palette_lefthand_object_positions[obj_ix], step_width), delete_label_ori, CS_LEFT_CONTROLLER, LA_LEFT, delete_label_scale);
		palette_text_labels.place_label(li, palette_label_offset(palette_object_positions[obj_ix], step_width), delete_label_ori, CS_LEFT_CONTROLLER, LA_CENTER, delete_label_scale);
		palette_text_labels.show_label(li);
	}

	const std::string& palette::get_label_text(unsigned obj_ix)
	{
		return this->palette_text_labels.get_label_text(palette_text_label_ids[obj_ix]);
	}

	void palette::set_object_data(const int id, void* data_ptr)
	{
		palette_object_data[id] = data_ptr;
	}

	void palette::render_palette(cgv::render::context& ctx, const cgv::mat34& pose) {
		static const quat object_tilt = quat(vec3(0, 0, 1), cgv::math::deg2rad(-37.5f));

		

		auto shape_to_index = [](const selection_shape& shape) {return (unsigned)shape - 1; };

		palette_text_labels.draw(ctx);

		if (palette_changed) {
			palette_indices.clear();
			palette_indices.resize(NUM_PALETTE_OBJECTS);
			for (int i = 0; i < palette_object_shapes.size(); ++i) {
				if (palette_object_visibility[i])
					palette_indices[palette_object_shapes[i]].push_back(i);
			}
		}

		auto& sphere_indices = palette_indices[PO_SPHERE];
		auto& box_indices = palette_indices[PO_CUBOID];
		auto& plane_indices = palette_indices[PO_PLANE];
		auto& box_wire_indices = palette_indices[PO_BOX_FRAME];
		auto& sphere_frame_indices = palette_indices[PO_SPHERE_FRAME];
		auto& box_plane_indices = palette_indices[PO_BOX_PLANE];

		if (palette_changed) {
			//palette_changed = false;
			//std::cout << "sphere_indices: " << sphere_indices.size() << std::endl;
			if (sphere_indices.size() > 0) {
				palette_sphere_renderer.set_render_style(p_sphere_style);
				palette_sphere_renderer.set_position_array(ctx, palette_object_positions);
				palette_sphere_renderer.set_color_array(ctx, palette_object_colors);
				palette_sphere_renderer.set_indices(ctx, sphere_indices);
			}
			//std::cout << "box_indices: " << box_indices.size() << std::endl;
			quat object_rotation = object_tilt;
			std::vector<quat> rotations = std::vector<quat>(palette_object_shapes.size(), object_rotation);
			if (box_indices.size() > 0) {
				palette_box_renderer.set_render_style(p_box_style);
				palette_box_renderer.set_position_array(ctx, palette_object_positions);
				palette_box_renderer.set_rotation_array(ctx, rotations);
				palette_box_renderer.set_color_array(ctx, palette_object_colors);
				palette_box_renderer.set_indices(ctx, box_indices);
			}

			//std::cout << "plane_indices: " << plane_indices.size() << std::endl;
			p_plane_style.default_extent = vec3(0.04, 0.005, 0.04);
			if (plane_indices.size() > 0) {
				palette_plane_renderer.set_render_style(p_plane_style);
				palette_plane_renderer.set_position_array(ctx, palette_object_positions);
				palette_plane_renderer.set_rotation_array(ctx, rotations);
				palette_plane_renderer.set_color_array(ctx, palette_object_colors);
				palette_plane_renderer.set_indices(ctx, plane_indices);
			}
			//std::cout << "box_wire_indices: " << box_wire_indices.size() << std::endl;
			if (box_wire_indices.size() > 0) {
				palette_box_wire_renderer.set_render_style(p_box_wire_style);
				palette_box_wire_renderer.set_position_array(ctx, palette_object_positions);
				palette_box_wire_renderer.set_rotation_array(ctx, rotations);
				palette_box_wire_renderer.set_color_array(ctx, palette_object_colors);
				palette_box_wire_renderer.set_indices(ctx, box_wire_indices);
			}

			if (sphere_frame_indices.size() > 0) {
				
				//auto& points = icosahedron_points();

				tesselate_geodesic_sphere(1, sphere_points, sphere_triangles);

				GLuint point_buffer = sphere_frame_point_buffer;
				GLuint offset_buffer = sphere_frame_offset_buffer;
				GLuint color_buffer = sphere_frame_color_buffer;
				
				std::vector<cgv::vec4> offsets;
				std::vector<cgv::rgba> colors;

				for (auto& sfi : sphere_frame_indices) {
					offsets.emplace_back(palette_object_positions[sfi], 1.f);
					colors.emplace_back(palette_object_colors[sfi]);
				}

				auto& prog = instanced_rendering_prog();

				glNamedBufferData(point_buffer, sphere_points.size() * sizeof(vec3), sphere_points.data(), GL_STATIC_DRAW);
				glNamedBufferData(offset_buffer, offsets.size() * sizeof(cgv::vec4), offsets.data(), GL_STATIC_DRAW);
				glNamedBufferData(color_buffer, colors.size() * sizeof(cgv::rgba), colors.data(), GL_STATIC_DRAW);
			}

			// render rounding corner box renderer for palette
			p_box_plane_style.rounding = true;
			p_box_plane_style.default_extent = vec3(0.04, 0.01, 0.04);
			p_box_plane_style.default_radius = 0.0075f;
			//std::cout << "box_plane_indices: " << box_plane_indices.size() << std::endl;
			if (box_plane_indices.size() > 0) {
				palette_box_plane_renderer.set_render_style(p_box_plane_style);
				palette_box_plane_renderer.set_position_array(ctx, palette_object_positions);
				palette_box_plane_renderer.set_color_array(ctx, palette_object_colors);
				palette_box_plane_renderer.set_indices(ctx, box_plane_indices);
			}
		}
		ctx.push_modelview_matrix();
		auto transform = cgv::math::identity4<double>();
		for (int i = 0; i < 3; ++i)
			transform.set_row(i, pose.row(i));
		ctx.set_modelview_matrix(ctx.get_modelview_matrix() * transform);

		if (box_indices.size())
			palette_box_renderer.render(ctx, 0, box_indices.size());

		if (box_wire_indices.size())
			palette_box_wire_renderer.render(ctx, 0, box_wire_indices.size());

		if (sphere_indices.size())
			palette_sphere_renderer.render(ctx, 0, sphere_indices.size());
		if (plane_indices.size())
			palette_plane_renderer.render(ctx, 0, plane_indices.size());

		if (box_plane_indices.size())
			palette_box_plane_renderer.render(ctx, 0, box_plane_indices.size());

		if (sphere_frame_indices.size() > 0) {
			auto& prog = instanced_rendering_prog();
			prog.set_uniform(ctx, "model_matrix", cgv::math::scale4(icosahedron_scale, icosahedron_scale, icosahedron_scale));

			//auto& indices = icosahedron_triangles();
			
			prog.enable(ctx);
			glBindVertexArray(sphere_frame_vao);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawElementsInstanced(GL_TRIANGLES, sphere_triangles.size(), GL_UNSIGNED_INT, sphere_triangles.data(), (GLsizei)sphere_frame_indices.size());
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBindVertexArray(0);
			prog.disable(ctx);

			//ctx.draw_edges_of_faces(&points[0][0], nullptr, nullptr, indices.data(), nullptr, nullptr, 20, 3);
			//ctx.ref_default_shader_program().disable(ctx);

		}

		auto model_view = ctx.get_modelview_matrix();
		ctx.push_modelview_matrix();
		for (int i = 0; i < point_clouds_point_positions.size();++i) {
			cgv::dmat4 model = cgv::math::translate4<float>(point_cloud_positions[i]) * cgv::math::scale4(vec3(point_cloud_scales[i])); //TODO add rotation
			ctx.set_modelview_matrix(model_view * model);
			auto& pr = cgv::render::ref_point_renderer(ctx);
			if (point_clouds_point_positions[i].size() == 0)
				continue;
			if (!point_cloud_array_managers[i].is_created())
				point_cloud_array_managers[i].init(*ctx_ptr);
			pr.set_render_style(point_cloud_style);
			if (palette_changed) {
				pr.enable_attribute_array_manager(ctx, point_cloud_array_managers[i]);
				pr.set_position_array(ctx, point_clouds_point_positions[i]);
				pr.set_color_array(ctx, point_clouds_point_colors[i]);
				pr.render(ctx, 0, point_clouds_point_positions[i].size());
			}
			else {
				pr.enable_attribute_array_manager(ctx, point_cloud_array_managers[i]);
				pr.render(ctx, 0, point_clouds_point_positions[i].size());
			}
			pr.disable_attribute_array_manager(ctx, point_cloud_array_managers[i]);
		}
		ctx.pop_modelview_matrix();

		ctx.pop_modelview_matrix();
		palette_changed = false;
	}

	void palette::generate_preview(point_cloud& points, std::vector<vec3>& point_positions, std::vector<cgv::rgb8>& point_colors, int detail_level)
	{
		static cgv::pointcloud::octree_lod_generator<cgv::pointcloud::SimpleLODPoint> lod_generator;

		std::vector<cgv::pointcloud::SimpleLODPoint> scaled_points(points.get_nr_points());
		points.create_colors(); // create colors if not already existing

		//create a scaled copy
		cgv::dvec3 centroid(0);
		box3 bounds;
		int nr_points = points.get_nr_points();
		for (int i = 0; i < nr_points; ++i) {
			vec3& p = points.pnt(i);
			centroid += p;
			bounds.add_point(p);
		}
		centroid /= (double)nr_points;
		int max_ix = bounds.get_max_extent_coord_index();
		const float phi = (1.0 + sqrt(5.0)) / 2.0;
		float scale = (phi * palette::icosahedron_scale) / (0.5f * (bounds.get_max_pnt()[max_ix] - bounds.get_min_pnt()[max_ix]));
		for (int i = 0; i < nr_points; ++i) {
			vec3& p = points.pnt(i);
			scaled_points[i].position() = scale * (p - centroid);
			scaled_points[i].color() = points.clr(i);
		}
		//generate lod points
		std::vector<cgv::pointcloud::SimpleLODPoint> preview_lod_points;
		preview_lod_points = lod_generator.generate_lods(scaled_points);

		//get rid of higher level points for preview in palette
		for (int i = 0; i < preview_lod_points.size(); ++i) {
			cgv::pointcloud::SimpleLODPoint& p = preview_lod_points[i];
			if (p.level() <= detail_level) {
				point_colors.push_back(p.color());
				point_positions.push_back(p.position());
			}
		}
	}



	picked_object::picked_object(palette& p, int id)
	{
		palette_ptr = &p;
		id_p = id;
	}

	void* picked_object::data()
	{
		return palette_ptr->palette_object_data[id()];
	}
	int picked_object::position_in_group()
	{
		return palette_ptr->positions_in_group[id()];
	}
	int picked_object::id()
	{
		return id_p;
	}
	PaletteObjectGroup picked_object::object_group()
	{
		return palette_ptr->palette_object_groups[id()];
	}
	PaletteObject picked_object::object_type()
	{
		return palette_ptr->palette_object_shapes[id()];
	}
	palette* picked_object::get_palette_ptr()
	{
		return this->palette_ptr;
	}
}



#include <cgv/gui/provider.h>

namespace cgv {
	namespace gui {

		struct palette_gui_creator : public gui_creator {
			/// attempt to create a gui and return whether this was successful
			bool create(provider* p, const std::string& label,
				void* value_ptr, const std::string& value_type,
				const std::string& gui_type, const std::string& options, bool*) {
				if (value_type != cgv::type::info::type_name<vrui::palette>::get_name())
					return false;
				vrui::palette* s_ptr = reinterpret_cast<vrui::palette*>(value_ptr);
				cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);
				if (p->begin_tree_node("point cloud style", s_ptr->point_style(), false)) {
					p->align("\a");
					p->add_gui("point cloud style", s_ptr->point_style());
					p->align("\b");
				}
				if (p->begin_tree_node("sphere style", s_ptr->sphere_style(), false)) {
					p->align("\a");
					p->add_gui("sphere style", s_ptr->sphere_style());
					p->align("\b");
				}
				if (p->begin_tree_node("box style", s_ptr->box_style(), false)) {
					p->align("\a");
					p->add_gui("box style", s_ptr->box_style());
					p->align("\b");
				}
				if (p->begin_tree_node("plane style", s_ptr->plane_style(), false)) {
					p->align("\a");
					p->add_gui("plane style", s_ptr->plane_style());
					p->align("\b");
				}
				if (p->begin_tree_node("wire box style", s_ptr->wire_box_style(), false)) {
					p->align("\a");
					p->add_gui("wire box style", s_ptr->wire_box_style());
					p->align("\b");
				}
				//p->add_member_control(b, "CLOD factor", s_ptr->CLOD, "value_slider", "min=0.1;max=10;ticks=true");
				return true;
			}
		};

		cgv::gui::gui_creator_registration<palette_gui_creator> pgc("palette_gui_creator");
	}
}