#include <cgv/base/base.h>
#include "pointcloud_labeling_tool.h"
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/gui/trigger.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/constants.h>
#include <cgv/math/pose.h>
#include <cgv/utils/dir.h>

#include <cgv/base/import.h>
#include <cgv/defines/quote.h>
#include <cgv/base/find_action.h>
#include <cgv/signal/signal.h>
#include <cgv/utils/file.h>

#include <algorithm>
#include <random>
#include <chrono>
#include <numeric>
#include <cstdio>
#include <concurrency.h>
#include "util.h"

using namespace std;
using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::type;
using namespace cgv::gui;
using namespace cgv::data;
using namespace cgv::utils;
using namespace cgv::render;
using namespace cgv::pointcloud;
using namespace vrui;

using namespace pct;

namespace {	

	//functions and data

	static cgv::pointcloud::utility::WorkerPool pool(std::thread::hardware_concurrency() - 1);
	
	//glCheckError from https://learnopengl.com/In-Practice/Debugging
	GLenum glCheckError_(const char* file, int line)
	{
		GLenum errorCode;
		while ((errorCode = glGetError()) != GL_NO_ERROR)
		{
			std::string error;
			switch (errorCode)
			{
			case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
			case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
			case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
			case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
			case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
			case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
			case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
			}
			std::cout << error << " | " << file << " (" << line << ")" << std::endl;
		}
		return errorCode;
	}
	#define glCheckError() glCheckError_(__FILE__, __LINE__)

	//prints errors in debug builds if shader code is wrong
	void shaderCheckError(cgv::render::shader_program& prog, const char name[]) {
#ifndef NDEBUG
		if (prog.last_error.size() > 0) {
			std::cerr << "error in " << name << "\n" << prog.last_error << '\n';
			prog.last_error = "";
		}
#endif // #ifdef NDEBUG
	}

	std::array<std::string, 4> registration_tool_names = { "Pull","ICP","Fuse/Paste" };

	std::string rgbd_input_help_text(const int mode, const point_cloud_registration_tool& tool) {
		std::ostringstream text;
		std::string ICP_text = mode != (int)RGBDInputModeTools::Pull && tool.registration_thread_is_busy() ? "(Processing...)" : " (Trigger)";
		text << "[RGBD INPUT]\n"
			<< "Mode: " << registration_tool_names[mode] << ICP_text << "\n"
			<< "Selected point cloud: " << tool.get_name() << "\n"
			//<< "Pointcloud " << tool.get_index() << "/" << tool.max_index() << " (Trackpad left/right)\n"
			<< "-- Controls --\n"
			<< "Grip Buttons: moving, rotating and scaling\n"
			<< "(Controller 0)\n"
			<< "Trackpad left/right: switch tool\n"
			<< "(Controller 1)\n"
			<< "Trackpad left/right: switch to rgbd_input/fill new slot\n"
			<< "Menu Button: swap point cloud\n";
		return text.str();
	}
	std::array<bool, NUM_OF_INTERACTIONS> interaction_mode_help_text_default_visibility = {false,true,true};

	void tesselate_ray(const int edges, const float length, const float radius, std::vector<cgv::math::fvec<float, 3>>& container) {

		for (int e = 0; e <= edges; ++e) {
			float angle = (2.f * PI * e) / edges;
			float y = radius * std::sin(angle);
			float z = radius * std::cos(angle);
			container.emplace_back(0.f, y, z);
			container.emplace_back(length, y, z);
		}
	}
}
// methods

///
pointcloud_labeling_tool::pointcloud_labeling_tool() : palette_clipboard_record_id(-1){
	set_name("pointcloud_cleaning_tool");
	
	point_server_ptr = nullptr;

	file_contains_label_groups = false;

	//initialize custom control providers
	labeled_point_group_control = pct::bitfield_control_provider<GLint>(~(~0u << 32) & (~0u << 16), &visible_point_groups);
	default_point_group_control = pct::bitfield_control_provider<GLint>((GLint)point_label_group::VISIBLE, &visible_point_groups);
	selected_point_group_control = pct::bitfield_control_provider<GLint>((GLint)point_label_group::SELECTED_BIT, &visible_point_groups);

	clipboard_ptr = std::make_shared<point_cloud_clipboard>();
	cgv::pointcloud::ref_octree_lod_generator<pct::indexed_point>(1); // initialize and/or increment ref count of lod generator singelton
	
	//add a record for the label palette
	std::unique_ptr<point_cloud_record> rec = std::make_unique<point_cloud_record>();

	point_cloud_registration = point_cloud_registration_tool(clipboard_ptr);
	clipboard_ptr->register_event_listener(&point_cloud_registration);

	writing_thread = nullptr;
	skip_frustum_culling = false;
	chunks_disabled = false;

	point_surface_style = surface_render_style();

	source_point_cloud.ref_render_style().draw_circles = true;
	
	max_points = 1000000;
	extended_frustum_size = 1.67;

	draw_teleport_destination = false;
	show_teleport_ray = true;
	teleport_spere_radius_factor = 0.04;
	teleport_ray_radius = 0.03;

	srs.radius = 0.12f;
	ars.length_scale = 0.05f;

	rcrs.radius = 0.003f;
	rcrs.rounded_caps = true;

	visible_point_groups = (GLint)point_label_group::VISIBLE;

	cube_length = 0.05f;
	cube_rhand = box3(vec3(-cube_length, cube_length, cube_length), vec3(cube_length, cube_length, cube_length));
	//plane_length = 0.05f;
	plane_box_rhand = box3(vec3(-0.001f, -0.1f, -0.1f), vec3(0.001f, 0.1f, 0.1f));

	interaction_mode = InteractionMode::TELEPORT;

	world.build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);

	//reflected members
	radius_for_test_labeling = 1.4;

	color_based_on_lod = false;
	
	pointcloud_fit_table = true;

	clear_selection_labels_after_copy = true;

	// model positioning 
	concat_mat.identity();
	is_scaling = false;
	is_rotating_moving = false;
	is_tp_floor = false;

	//load mesh
	show_surface = false;
	cull_mode = CM_BACKFACE;
	color_mapping = cgv::render::CM_COLOR;
	surface_color = rgb(0.7f, 0.2f, 1.0f);
	illumination_mode = IM_ONE_SIDED;

	show_wireframe = false;

	show_vertices = false;

	for (int i =0;i<2;++i){
		controller_poses[i] = mat34(0);
	}
	// set up point selection group filters, enable labeling of point in any group except protected 
	default_point_selection_group_mask = point_selection_group_mask = (int)point_label_group::GROUP_MASK & ~(int)point_label_group::PROTECTED_BIT;
	default_point_selection_exclude_group_mask = point_selection_exclude_group_mask = (int)point_label_group::PROTECTED_BIT; //exclude

	//create label history with a fixed size of (2^24)-1 points
	history_ptr = new history(24);

	default_point_selection_color = point_selection_color = rgba(0.09, 0.7, 1.0, 1.0);
	// offset in minus Z axis -0.1f
	initial_offset_rhand = vec3(0, 0, -0.1f);
	curr_offset_rhand = initial_offset_rhand;
	
	build_palette();
	build_clipboard_palette();
	
	picked_sphere_index = 0;
	picked_label = (int32_t)point_label_group::DELETED;
	picked_label_operation = point_label_operation::REPLACE;
	point_selection_shape = selection_shape::SS_SPHERE;
	point_pushing_shape = selection_shape::SS_CUBOID;
	point_editing_tool = pallete_tool::PT_BRUSH;

	sphere_style_rhand.radius = 0.02f;
	sphere_style_lhand.radius = 0.02f;
	radius_adjust_step = 0.001f;
	//create a blacklist for shapes not avaiable as push tool
	pushing_shape_blacklist.insert(selection_shape::SS_PLANE);
	pushing_shape_blacklist.insert(selection_shape::SS_ALL);

	selection_shape_blacklist.insert(selection_shape::SS_ALL);

	//help text labels
	li_help[0] = li_help[1] = -1;
	
	share_reduction_pass = false;

	use_vr_kit_as_pivot = false;
	
	plane_invert_selection = false;		

	sliding_window_size = 100;
	draw_time = stats::scalar_sliding_window<GLuint64>(sliding_window_size);

	connect(get_animation_trigger().shoot, this, &pointcloud_labeling_tool::timer_event);
	
	reduce_time_cpu = stats::scalar_sliding_window<double>(sliding_window_size);
	reduce_time_gpu = stats::scalar_sliding_window<double>(sliding_window_size);
	
	labeling_time_cpu = stats::scalar_sliding_window<double>(sliding_window_size);
	labeling_time_gpu = stats::scalar_sliding_window<double>(sliding_window_size);
	//poll_reduce_timer and poll_query_data are initialized as false
	poll_reduce_timer_data = poll_query_data = false;

	paste_pointcloud_follow_controller = false;

	static const std::unordered_map<unsigned, std::string> name_map;
	static const std::unordered_map<unsigned, rgba> color_map;
	set_palette_label_color_map(color_map);
	set_palette_label_text_map(name_map);

	//initialize tool label arrays
	for (auto& a : tool_help_label_id)
		for (auto& e : a)
			e = -1;
	for (auto& a : tool_help_label_allways_drawn)
		for (auto& e : a)
			e = false;

	box_shaped_selection_is_constraint = false;

	use_autopilot = false;
	navigation_is_controller_path = false;
	navigation_selected_controller = 1;
	start_l = false;

	log_fps = false;
	log_labeling_gpu_time = false;
	log_labeling_cpu_time = false;
	log_reduce_time = false;
	log_reduce_cpu_time = false;
	log_draw_time = false;

	tra_hmd = false;
	is_show_tra = false;
	adapt_clod = false;
}

pointcloud_labeling_tool::~pointcloud_labeling_tool()
{
	cgv::pointcloud::ref_octree_lod_generator<pct::indexed_point>(-1);
}

///
namespace {

	struct point_label_brush {
		uint32_t label;
		point_label_operation operation;
		selection_shape shape;
		
		uint32_t point_groups;
		uint32_t excluded_point_groups;
		bool group_override;

		constexpr point_label_brush() : label(0), operation(point_label_operation::NONE), shape(selection_shape::SS_NONE),
			point_groups((uint32_t)point_label_group::GROUP_MASK), excluded_point_groups(0), group_override(false){}

		constexpr point_label_brush(const uint32_t l, const point_label_operation op, const selection_shape sh) : label(l), operation(op), shape(sh),
			point_groups((uint32_t)point_label_group::GROUP_MASK), excluded_point_groups(0), group_override(false){}
	
		void set_group_override(uint32_t groups, uint32_t excluded_groups) {
			point_groups = groups;
			excluded_point_groups = excluded_groups;
			group_override = true;
		}
	};
}
///
bool pointcloud_labeling_tool::self_reflect(cgv::reflect::reflection_handler & rh)
{
	return
		rh.reflect_member("pointcloud_fit_table", pointcloud_fit_table) &&
		rh.reflect_member("max_points", max_points) &&
		rh.reflect_member("point_color_based_on_lod", color_based_on_lod) &&
		rh.reflect_member("model_scale", source_point_cloud.ref_point_cloud_scale()) &&
		rh.reflect_member("model_position_x", source_point_cloud.ref_point_cloud_position().x()) &&
		rh.reflect_member("model_position_y", source_point_cloud.ref_point_cloud_position().y()) &&
		rh.reflect_member("model_position_z", source_point_cloud.ref_point_cloud_position().z()) &&
		rh.reflect_member("model_rotation_x", source_point_cloud.ref_point_cloud_rotation().x()) &&
		rh.reflect_member("model_rotation_y", source_point_cloud.ref_point_cloud_rotation().y()) &&
		rh.reflect_member("model_rotation_z", source_point_cloud.ref_point_cloud_rotation().z()) &&
		rh.reflect_member("show_environment", world.show_environment) &&
		rh.reflect_member("show_table", world.show_table) &&
		rh.reflect_member("show_controller_button_labels", show_controller_labels) &&
		rh.reflect_member("show_left_bar", show_left_bar) &&
		rh.reflect_member("show_help_always_visible", always_show_help) &&
		rh.reflect_member("clod_point_style", source_point_cloud.ref_render_style()) &&
		rh.reflect_member("chunk_size", preparation_settings.chunk_cube_size) &&
		rh.reflect_member("label_palette_file", label_palette_file) &&
		rh.reflect_member("interaction_mode", interaction_mode) &&
		rh.reflect_member("skip_frustum_culling", skip_frustum_culling) &&
		rh.reflect_member("use_chunks_for_labeling", point_cloud_interaction_settings.use_chunks) &&
		rh.reflect_member("enable_performance_stats", enable_performance_stats) &&
		rh.reflect_member("sliding_window_size", sliding_window_size) &&
		rh.reflect_member("disable_main_view", disable_main_view) &&
		rh.reflect_member("allow_point_deduplication", preparation_settings.allow_point_deduplication) &&
		rh.reflect_member("extended_frustum_size", extended_frustum_size) &&
		rh.reflect_member("point_editing_tool", point_editing_tool) &&
		rh.reflect_member("show_teleport_ray", show_teleport_ray) &&
		rh.reflect_member("visible_groups", visible_point_groups) &&
		rh.reflect_member("clear_selection_labels_after_copy", clear_selection_labels_after_copy);
}
///
void pointcloud_labeling_tool::on_set(void * member_ptr)
{
	if (member_ptr == &world.show_environment || member_ptr == &world.show_table || member_ptr == &world.show_floor) {
		world.clear_scene();
		world.build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);
	}
	else if (member_ptr == &label_palette_file) {
		if (!label_palette_file.empty()) {
			auto fn = cgv::base::find_data_file(label_palette_file, "MD", QUOTE_SYMBOL_VALUE(INPUT_DIR)); //makro suffers from namespace pollution from math/constants-h if path starts with e.g. E:
			if (fn.size() > 0)
				load_labels_from(fn);
			else
				load_labels_from(label_palette_file);
		}
	}
	else if ((intptr_t)member_ptr >= (intptr_t)(&source_point_cloud.ref_point_cloud_position().x()) && (intptr_t)member_ptr <= (intptr_t)(&source_point_cloud.ref_point_cloud_position().z()))
	{
		point_server_ptr->ref_point_cloud_position() = source_point_cloud.ref_point_cloud_position();
	}
	else if ((intptr_t)member_ptr >= (intptr_t)(&source_point_cloud.ref_point_cloud_rotation().x()) && (intptr_t)member_ptr <= (intptr_t)(&source_point_cloud.ref_point_cloud_rotation().z()))
	{
		point_server_ptr->ref_point_cloud_rotation() = source_point_cloud.ref_point_cloud_rotation();
	}
	else if (member_ptr == &source_point_cloud.ref_point_cloud_scale())
	{
		point_server_ptr->ref_point_cloud_scale() = source_point_cloud.ref_point_cloud_scale();
	}
	else if (member_ptr == &sliding_window_size) {
		draw_time.resize(sliding_window_size);
		reduce_time_cpu.resize(sliding_window_size);
		reduce_time_gpu.resize(sliding_window_size);
	}
	else if (member_ptr == &interaction_mode) {
		update_interaction_mode((InteractionMode)interaction_mode);
	}
	else if (member_ptr == &max_points) {
		if (max_points <= 0)
			max_points = 1;
		for (auto& e : reduction_state)
			e.resize(max_points);
	}
	else if (member_ptr == &disable_main_view) {
		//free some memory if disabled
		if (disable_main_view) {
			for (auto& s : reduction_state)
				s.clear(*this->get_context());
			reduction_state.clear();
		}
	}
	else if (member_ptr == &enable_performance_stats) {
		point_server_ptr->configure_timers(enable_performance_stats, enable_performance_stats);
	}
	update_member(member_ptr);
	post_redraw();
}
///
void pointcloud_labeling_tool::on_register()
{
	if (point_cloud_registration.get_provider() == nullptr) {
		cgv::base::node* node_ptr = const_cast<cgv::base::node*>(dynamic_cast<const cgv::base::node*>(this));
		point_cloud_providers.clear();
		cgv::base::find_interface<cgv::pointcloud::point_cloud_provider>(cgv::base::base_ptr(node_ptr), point_cloud_providers);
		if (point_cloud_providers.size() > 0)
			point_cloud_registration.set_provider(point_cloud_providers[0]);
	}
}
///
void pointcloud_labeling_tool::unregister()
{
	if (point_cloud_registration.get_provider()) {
		cgv::base::node* node_ptr = const_cast<cgv::base::node*>(dynamic_cast<const cgv::base::node*>(this));
		point_cloud_providers.clear();
		cgv::base::find_interface<cgv::pointcloud::point_cloud_provider>(cgv::base::base_ptr(node_ptr), point_cloud_providers);
		if (point_cloud_providers.size() == 0)
			point_cloud_registration.set_provider(nullptr);
	}
}
///
bool pointcloud_labeling_tool::init(cgv::render::context& ctx)
{
	//glDisable(GL_DEBUG_OUTPUT);
	cgv::gui::connect_vr_server(true);

	glGenQueries(2, gl_render_query);
	glGenQueries(2, reduce_timer_query);
	poll_query_data = false;

	cgv::render::view* view_ptr = find_view_as_node();

	if (view_ptr) {
		view_ptr->set_view_up_dir(vec3(0, 1, 0));
		view_ptr->set_focus(vec3(0, 0, 0));
		//view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	cgv::render::ref_cone_renderer(ctx, 1);
	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_clod_point_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_arrow_renderer(ctx, 1);
	cgv::render::ref_box_wire_renderer(ctx, 1);
	cgv::render::ref_point_renderer(ctx, 1);

	// share history with point server
	point_server_ptr = &ref_point_cloud_server(ctx, 1);
	std::unique_ptr<history> history_uptr(history_ptr, std::default_delete<history>());
	point_server_ptr->swap_history(history_uptr);

	palette.init(ctx);
	rgbd_input_palette.init(ctx);

	//ctx.set_bg_color(0.7, 0.7, 0.8, 1.0);
	ctx.set_bg_color(1.0, 1.0, 1.0, 1.0);

	//add label color lookup table buffer, begin copy after the first element which stands for the color of the delete label
	glCreateBuffers(1, &color_map_buffer);
	glNamedBufferData(color_map_buffer, (PALETTE_COLOR_MAPPING.size() - 1) * sizeof(vec4), PALETTE_COLOR_MAPPING.data() + 1, GL_STATIC_READ);
	color_map_buffer_size = (GLuint)(PALETTE_COLOR_MAPPING.size() - 1);

	// create shader prog for selectiv labeling based on existing labels
	if (!selection_relabel_prog.is_created()) {
		selection_relabel_prog.create(ctx);
		selection_relabel_prog.attach_file(ctx, "selection_labeler.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "selection_labeler.glcs");
		selection_relabel_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "point_label.glsl");
		selection_relabel_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "point_label_history.glsl");
		selection_relabel_prog.link(ctx, true);
	}
	// create shader prog for rollback
	if (!label_rollback_prog.is_created()) {
		label_rollback_prog.create(ctx);
		label_rollback_prog.attach_file(ctx, "point_label_rollback.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(label_rollback_prog, "point_label_rollback.glcs");
		label_rollback_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		label_rollback_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		label_rollback_prog.link(ctx, true);
		history_ptr->set_rollback_shader_prog(&label_rollback_prog);
	}

	//initialize label shaders
	ref_label_shader_manager(ctx, 1);

	//initialize label buffer
	glCreateBuffers(1, &lod_to_color_lut_buffer);


	//initialize paste buffer
	glCreateBuffers(1, &clipboard_paste_point_buffer);
	glCreateBuffers(1, &clipboard_paste_reduce_ids);
	//initialize rollback feature
	history_ptr->init(ctx);
	//initialize text field
	text_labels.set_vr_view(vr_view_ptr);
	text_labels.init(ctx);

	//initialize controller text fields
	controller_labels[0].set_vr_view(vr_view_ptr);
	controller_labels[1].set_vr_view(vr_view_ptr);
	controller_labels[0].init(ctx);
	controller_labels[1].init(ctx);

	//initialize palettes
	palette.init_text_labels(ctx, vr_view_ptr);
	rgbd_input_palette.init_text_labels(ctx, vr_view_ptr);

	//add labels for controllers

	tool_help_label_id.fill(std::array<uint32_t, 2>{ (uint32_t)-1, (uint32_t)-1});
	const rgba tool_label_color = rgba(0.8f, 0.4f, 0.4f, 1.0f);
	vec3 tool_help_label_offset = vec3(0.06, 0.0, 0.0125);
	right_tool_label_offset = tool_help_label_offset;
	left_tool_label_offset = vec3(-0.06, 0.0, 0.0125);
	tool_label_scale = 0.2f;
	tool_label_ori = quat(vec3(1, 0, 0), -1.5f);
	CoordinateSystem tool_help_label_coordinate_system = CS_RIGHT_CONTROLLER;
	CoordinateSystem status_label_coordinate_system = CS_LEFT_CONTROLLER;
	controller_labels[0].set_coordinate_system(CoordinateSystem::CS_LEFT_CONTROLLER);
	controller_labels[1].set_coordinate_system(CoordinateSystem::CS_RIGHT_CONTROLLER);
	controller_label_color = rgba(0.1, 0.2, 0.93, 1.0);

	
	{	//defaults for trackpad up/down
		int label_right_track_pad_up = controller_labels[1].add_variant(CLP_TRACKPAD_UP, "forward", controller_label_color);
		int label_right_track_pad_down = controller_labels[1].add_variant(CLP_TRACKPAD_DOWN, "backward", controller_label_color);

		int label_left_track_pad_up = controller_labels[0].add_variant(CLP_TRACKPAD_UP, "next interaction", controller_label_color);
		int label_left_track_pad_down = controller_labels[0].add_variant(CLP_TRACKPAD_DOWN, "prev. interaction", controller_label_color);

		for (int i = 0; i < InteractionMode::NUM_OF_INTERACTIONS; ++i) {
			controller_label_variants[1][i][CLP_TRACKPAD_UP].push_back(label_right_track_pad_up);
			controller_label_variants[1][i][CLP_TRACKPAD_DOWN].push_back(label_right_track_pad_down);
			controller_label_variants[0][i][CLP_TRACKPAD_UP].push_back(label_left_track_pad_up);
			controller_label_variants[0][i][CLP_TRACKPAD_DOWN].push_back(label_left_track_pad_down);
		}
	}

	{
		config_help_label_id = text_labels.add_label(
				"[Config]\n"
				"Adjust the size of points and CLOD factor\n"
				"Controller Left\n"
				"   Trackpad: touch left-right to adjust the size of points\n"
				"Controller Right\n"
				"   Trackpad: touch left-right to adjust the CLOD factor\n"
				"Grip Buttons\n"
				" toggle point spacing probe\n",
				tool_label_color);
		tool_help_label_id[InteractionMode::CONFIG][1] = config_help_label_id;
		text_labels.place_label(config_help_label_id,
			tool_help_label_offset, tool_label_ori, tool_help_label_coordinate_system, LA_LEFT, tool_label_scale);
		text_labels.hide_label(config_help_label_id);
	}
	controller_label_variants[1][InteractionMode::CONFIG][CLP_GRIP].push_back(
		controller_labels[1].add_variant(CLP_GRIP, "toggle\npoint spacing\nprobe", controller_label_color));
	
	{
		clod_parameters_template_str =
			"Trackpad:\n"
			" up down   : change interaction mode\n"
			"====================================\n"
			"CLOD Factor: %f\n"
			"Scale: %f\n"
			"Point Spacing (L0): %f m\n"
			"Point Size:    %f\n"
			"min. Millimeters: %f mm\n";
		std::array<char, 512> buff;
		auto& style = source_point_cloud.ref_render_style();
		std::snprintf(buff.data(), buff.size(), clod_parameters_template_str.data(), style.CLOD, style.scale, style.spacing, style.pointSize, style.min_millimeters);
		clod_parameters_label_id = text_labels.add_label(
			std::string(buff.data()),
			tool_label_color);

		tool_help_label_id[InteractionMode::CONFIG][0] = clod_parameters_label_id;
		
		text_labels.place_label(clod_parameters_label_id,
			left_tool_label_offset, tool_label_ori, status_label_coordinate_system, LA_RIGHT, tool_label_scale);
		text_labels.hide_label(clod_parameters_label_id);
	}

	{
		tool_help_label_id[InteractionMode::LABELING][1] =
			text_labels.add_label(
				"[Labeling]\n"
				"use the trigger to label points\n"
				"Trackpad of Left Controller:\n"
				" up down   : change interaction mode\n"
				" left right: change size\n"
				"Trackpad of Right Controller:\n"
				" up down   : move\n"
				" left right: change shape\n"
				" right: paste"
				"Grip Button of Right Controller:\n"
				" with plane selected: invert\n"
				" with box frame: set label\n"
				"Menu buttons:\n"
				" right: copy selected points\n"
				" left: rollback last operation\n",
				tool_label_color);

		auto li = tool_help_label_id[InteractionMode::LABELING][1];
		text_labels.place_label(li,
			tool_help_label_offset, tool_label_ori, tool_help_label_coordinate_system, LA_LEFT, tool_label_scale);
		text_labels.hide_label(li);

		controller_label_variants[1][InteractionMode::LABELING][CLP_TRACKPAD_RIGHT].push_back(
			controller_labels[1].add_variant(CLP_TRACKPAD_RIGHT, "shape->", controller_label_color));

		box_is_constraint_label_variant = controller_labels[1].add_variant(CLP_TRACKPAD_RIGHT, "constraint", controller_label_color);

		paste_is_shown = controller_labels[1].add_variant(CLP_TRACKPAD_RIGHT, "paste", controller_label_color);

		controller_label_variants[1][InteractionMode::LABELING][CLP_TRACKPAD_LEFT].push_back(
			controller_labels[1].add_variant(CLP_TRACKPAD_LEFT, "<-shape", controller_label_color));
		clear_box_label_variant = controller_labels[1].add_variant(CLP_TRACKPAD_LEFT, "clear box", controller_label_color);

		// reserve some space for each pickable shape + paste mode
		controller_label_variants[1][InteractionMode::LABELING][CLP_GRIP].resize(selection_shape::NUM_OF_SHAPES + 1, -1);

		// set grip button texts
		int label_move_right_controller = controller_labels[1].add_variant(CLP_GRIP, "move", controller_label_color);
		for (int v = 0; v < controller_label_variants[1][InteractionMode::LABELING][CLP_GRIP].size(); ++v)
			controller_label_variants[1][InteractionMode::LABELING][CLP_GRIP][v] = label_move_right_controller;

		controller_label_variants[1][InteractionMode::LABELING][CLP_GRIP][selection_shape::SS_PLANE] =
			controller_labels[1].add_variant(CLP_GRIP, "invert", controller_label_color);
		//paste_mode_controller_label_variant_index = selection_shape::NUM_OF_SHAPES;
		//controller_label_variants[1][InteractionMode::LABELING][CLP_GRIP][paste_mode_controller_label_variant_index] =
			//controller_labels[1].add_variant(CLP_GRIP, "paste", controller_label_color);
		controller_label_variants[0][InteractionMode::LABELING][CLP_GRIP].push_back(
			controller_labels[0].add_variant(CLP_GRIP, "scale", controller_label_color));

		controller_label_variants[1][InteractionMode::LABELING][CLP_MENU_BUTTON].push_back(
			controller_labels[1].add_variant(CLP_MENU_BUTTON, "copy", controller_label_color));
		controller_label_variants[0][InteractionMode::LABELING][CLP_MENU_BUTTON].push_back(
			controller_labels[0].add_variant(CLP_MENU_BUTTON, "rollback", controller_label_color));
	}

	/* {
		tool_help_label_id[InteractionMode::CONFIG][1] =
			text_labels.add_label(
				"[Config]\n"
				"Adjust the size of points and CLOD factor\n"
				"Controller Left\n"
				"   Trackpad: touch left-right to adjust the size of points\n"
				"Controller Right\n"
				"   Trackpad: touch left-right to adjust the CLOD factor\n"
				"Grip buttons\n"
				"	enable/disable point spacing probe\n",
				tool_label_color);
		auto li = tool_help_label_id[InteractionMode::CONFIG][1];
		text_labels.place_label(li,
			tool_help_label_offset, tool_label_ori, tool_help_label_coordinate_system, LA_LEFT, tool_label_scale);
		text_labels.hide_label(li);
	}*/
	controller_label_variants[0][InteractionMode::CONFIG][CLP_TRACKPAD_CENTER].push_back(
		controller_labels[0].add_variant(CLP_TRACKPAD_CENTER, "- <   point size   > +", controller_label_color));
	controller_label_variants[1][InteractionMode::CONFIG][CLP_TRACKPAD_CENTER].push_back(
		controller_labels[1].add_variant(CLP_TRACKPAD_CENTER, "- <   CLOD factor  > +", controller_label_color));
	apply_spacing_label_variant = controller_labels[1].add_variant(CLP_MENU_BUTTON, "apply spacing", controller_label_color);

	{
		tool_help_label_id[InteractionMode::TELEPORT][1] =
			text_labels.add_label(
				"[Teleport]\n"
				"Trigger Controller Right: teleport\n"
				"Controller 1 trigger: teleport with preview\n"
				"Controller 1 menu: put pointcloud on table\n",
				tool_label_color);
		auto li = tool_help_label_id[InteractionMode::TELEPORT][1];
		text_labels.place_label(li,
			tool_help_label_offset, tool_label_ori, tool_help_label_coordinate_system, LA_LEFT, tool_label_scale);
		text_labels.hide_label(li);
	}

	{ // point spacing tool
		spacing_tool_template_str =
			"Hold the sphere halfways into a surface and\n"
			"press the trigger to measure the point spacing.\n"
			"probe position: %f, %f, %f\n"
			"point density: %f pnts/m²\n"
			"point spacing: %f m\n"
			"l0 spacing:    %f m (required by the clod renderer)";
		std::array<char, 512> buff;
		std::snprintf(buff.data(), buff.size(), spacing_tool_template_str.data(), 
			last_measured_position.x(), last_measured_position.y(), last_measured_position.z(), 
			last_measured_point_density, last_measured_point_spacing, last_measured_l0_spacing);
		spacing_tool_label_id = text_labels.add_label(
			std::string(buff.data()),
			tool_label_color);
		text_labels.place_label(spacing_tool_label_id,
			tool_help_label_offset, tool_label_ori, tool_help_label_coordinate_system, LA_LEFT, tool_label_scale);
		text_labels.hide_label(spacing_tool_label_id);
	}

	world.init(ctx);

	box_shaped_selection.init(ctx);
	
	point_cloud_registration.init(ctx);

	return true;
}

///
void pointcloud_labeling_tool::clear(cgv::render::context& ctx)
{
	glDeleteQueries(2, gl_render_query);
	glDeleteQueries(2, reduce_timer_query);

	selection_relabel_prog.destruct(ctx);
	//glDeleteBuffers(1, &point_label_buffer);
	glDeleteBuffers(1, &color_map_buffer);
	glDeleteBuffers(1, &lod_to_color_lut_buffer);
	glDeleteBuffers(1, &clipboard_paste_point_buffer);	
	glDeleteBuffers(1, &clipboard_paste_reduce_ids);
	history_ptr->clear(ctx);
	text_labels.clear(ctx);

	ref_label_shader_manager(ctx, -1);
	cgv::render::ref_cone_renderer(ctx, -1);
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_clod_point_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_arrow_renderer(ctx, -1);
	cgv::render::ref_box_wire_renderer(ctx, -1);
	cgv::render::ref_point_renderer(ctx, -1);

	ref_point_cloud_server(ctx, -1);
	point_server_ptr = nullptr;

	palette.clear(ctx);
	rgbd_input_palette.clear(ctx);

	world.clear(ctx);
	box_shaped_selection.clear(ctx);
	point_cloud_registration.clear(ctx);

	for (auto& e : reduction_state)
		e.clear(ctx);
	controller_labels[0].clear(ctx);
	controller_labels[1].clear(ctx);
}



///
void pointcloud_labeling_tool::init_frame(cgv::render::context& ctx)
{
	//auto start_reduce = std::chrono::steady_clock::now();
	text_labels.init_frame(ctx);
	if (show_controller_labels) {
		controller_labels[0].init_frame(ctx);
		controller_labels[1].init_frame(ctx);
	}

	if (interaction_mode == LABELING)
		palette.init_frame(ctx);

	// update visibility of visibility changing labels
	
	if (vr_view_ptr && vr_view_ptr->get_current_vr_state()) {
		vec3 view_dir = -reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->hmd.pose[6]);
		
		vec3 view_pos = reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->hmd.pose[9]);
		for (int ci = 0; ci < 2; ++ci) {
			if (li_help[ci] == -1)
				continue;
			// always_show_help can also be used for debugging
			if (always_show_help || allways_show_controller_label[ci] || interaction_mode_help_text_default_visibility[interaction_mode]) {
				text_labels.show_label(li_help[ci]);
				continue;
			}
			auto controller_pose = reinterpret_cast<const mat34*>(vr_view_ptr->get_current_vr_state()->controller[ci].pose);
			vec3 controller_dir = cgv::math::pose_orientation(*controller_pose)*vec3(0,0,-1);
			vec3 controller_pos = reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->controller[ci].pose[9]);
			float controller_depth = dot(view_dir, controller_pos - view_pos);
			float controller_dist = (view_pos + controller_depth * view_dir - controller_pos).length();
			// show help based on view direction
			if (std::abs(cgv::math::dot(view_dir, controller_dir)) < 0.2 /* && view_dir.y() < 0.05f */ ) {
				text_labels.show_label(li_help[ci]);
			}
			else {
				text_labels.hide_label(li_help[ci]);
			}
		}
	}

	
	/*auto stop_reduce = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff_d = stop_reduce - start_reduce;
	std::cout << "diff_init: " << diff_d.count() << std::endl;*/
}
///
void pointcloud_labeling_tool::draw(cgv::render::context & ctx)
{
	if (fps_watch_file_ptr && log_fps && vr_view_ptr->get_rendered_eye() == 2) {
		frame_time = fps_watch.get_elapsed_time();
		fps_watch.restart();
		*fps_watch_file_ptr << 1.0/frame_time << "\n";
		
	}
	if (adapt_clod && vr_view_ptr->get_rendered_eye() == 2)
	{
		frame_time = fps_watch.get_elapsed_time();
		fps_watch.restart();
		// adapt clod rendering parameters
		frame_rate = 1.0/ frame_time;
		int f = int(frame_rate);
		adapt_parameters(f);
		update_views(&frame_rate);
	}

	//auto start_reduce = std::chrono::steady_clock::now();
	if (disable_main_view && vr_view_ptr->get_rendered_eye() == 2) {
		return; //skip rendering
	}
	
	// some variables required later
	vec4 selection_center_in_view_space;
	vec4 selection_center_in_world_space;
	mat4 view_transform = ctx.get_modelview_matrix(); 	// save the view transfrom before its changed by set_modelview_matrix
	
	// build a view transform for hmd
	if (share_reduction_pass && vr_view_ptr->get_rendered_eye() == 0) {
		for (int i = 0; i < 4; ++i) {
			/*
			for (int j = 0; j < 3; ++j) {
				hmd_reduction_view_transform[i * 4 + j] = vr_view_ptr->get_current_vr_state()->hmd.pose[i * 3 + j];
			}
			*/
			memcpy(&hmd_reduction_view_transform[i * 4], &vr_view_ptr->get_current_vr_state()->hmd.pose[i * 3], 3*sizeof(float));

			hmd_reduction_view_transform[i * 4 + 3] = 0.f;
		}
		hmd_reduction_view_transform[15] = 1.f;
		
		hmd_reduction_view_transform = cgv::math::inv(hmd_reduction_view_transform);
	}
	// points to view matrix used in reduction 
	mat4* reduction_view_transform_ptr = share_reduction_pass  ? &hmd_reduction_view_transform : &view_transform;
		
	//the renderer is accessible over a reference to a singelton but new instances are also possible
	cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(ctx);
	
	// draw table and floor
	world.draw(ctx);
	
	cp_renderer.set_render_style(source_point_cloud.ref_render_style());
	// set frustum extend
	float frustum_extend = (share_reduction_pass || max_reduction_epoch) ? extended_frustum_size : 1.f;
	cp_renderer.set_frustum_extend(frustum_extend);
	
	//fix pivot point to vr headset position

	vec4 pivot(0, 0, 0, 1);
	if (use_vr_kit_as_pivot) { // if true use the vr kit regardless of rendered eye
		vec4 eye = vr_view_ptr->get_eye_of_kit(0).lift();
		pivot = view_transform * eye;
	}
	cp_renderer.set_pivot_point(pivot);

	// draw if source point cloud is not empty
	if (source_point_cloud.get_nr_points() > 0) {
		//setup shader parameters
		cp_renderer.set_max_drawn_points(ctx, max_points);
		
		mat4 concat_mat_f = point_server_ptr->get_point_cloud_model_transform();
		concat_mat = concat_mat_f;
		ctx.push_modelview_matrix();
		ctx.set_modelview_matrix(ctx.get_modelview_matrix()* concat_mat);

		//choose a shader program based on selection mode
		shader_program* active_label_prog = nullptr;
		if (use_label_prog) {
			static constexpr int selection_color_location = 20;
			auto& shader_manager = ref_label_shader_manager(ctx);
			light_model lighting = enable_lights ? light_model::LM_LOCAL : light_model::LM_NONE;

			if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
				// pick shader program, compute selection center and set uniforms based on used shape

				auto start_select = std::chrono::steady_clock::now();
				switch (point_selection_shape) {
				case SS_SPHERE: {
					selection_center_in_world_space = (cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(sphere_style_rhand.radius)).lift();
					selection_center_in_view_space = view_transform * selection_center_in_world_space;

					active_label_prog = &shader_manager.get_clod_render_shader(ctx, SS_SPHERE, lighting);
					vec4 selection_center_and_radius = selection_center_in_world_space;
					selection_center_and_radius.w() = sphere_style_rhand.radius;
					active_label_prog->set_uniform(ctx, "selection_enclosing_sphere", selection_center_and_radius, true);
					active_label_prog->set_uniform(ctx, selection_color_location, point_selection_color);
					break;
				}
				case SS_PLANE:
				{
					selection_center_in_world_space = (cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand).lift();
					selection_center_in_view_space = view_transform * selection_center_in_world_space;

					active_label_prog = &shader_manager.get_clod_render_shader(ctx, SS_PLANE, lighting);
					vec3 normal(1.f, 0.f, 0.f);
					plane_orientation_rhand.rotate(normal);
					vec4 plane = ((plane_invert_selection) ? -1.f : 1.f) * normal.lift(); plane.w() = 0;
					active_label_prog->set_uniform(ctx, "selection_plane_normal", plane, true);
					active_label_prog->set_uniform(ctx, "selection_plane_origin", selection_center_in_world_space, true);
					active_label_prog->set_uniform(ctx, selection_color_location, point_selection_color);
					break;
				}
				case SS_CUBOID: {
					selection_center_in_world_space = (cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(cube_rhand.get_extent().z()*0.5f)).lift();
					selection_center_in_view_space = view_transform * selection_center_in_world_space;

					active_label_prog = &shader_manager.get_clod_render_shader(ctx, SS_CUBOID, lighting);
					active_label_prog->set_uniform(ctx, "selection_box_rotation", cube_orientation_rhand);
					active_label_prog->set_uniform(ctx, "aabb_min_p", cube_rhand.get_min_pnt());
					active_label_prog->set_uniform(ctx, "aabb_max_p", cube_rhand.get_max_pnt());
					active_label_prog->set_uniform(ctx, "selection_box_translation", selection_center_in_world_space, true);
					active_label_prog->set_uniform(ctx, selection_color_location, point_selection_color);
					break;
				}
				default:
					selection_center_in_world_space = (cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand).lift();
					selection_center_in_view_space = view_transform * selection_center_in_world_space;

					active_label_prog = &shader_manager.get_clod_render_shader(ctx, SS_NONE, lighting);
				}
				auto stop_select = std::chrono::steady_clock::now();
				diff_select = stop_select - start_select;
			}
			else { //no labeling
				active_label_prog = &shader_manager.get_clod_render_shader(ctx, SS_NONE, lighting);
			}

			if (enable_lights)
				ctx.set_current_lights(*active_label_prog);
			
			//set model matrix
			static constexpr int model_mat_loc = 1; //active_label_prog->set_uniform(ctx, "model_mat", concat_mat_f);
			static constexpr int color_based_on_lods_loc = 2;
			static constexpr int visible_groups_loc = 21;
			active_label_prog->set_uniform(ctx, model_mat_loc, concat_mat_f);
			active_label_prog->set_uniform(ctx, color_based_on_lods_loc, color_based_on_lod);
			active_label_prog->set_uniform(ctx, visible_groups_loc, visible_point_groups);
		}

		{	// draw point cloud
			auto& chunked_points = point_server_ptr->ref_chunks();
			
			// debug: print chunks rendered in last pass (only without distributed reduction mode) 
			//std::cout << "rendered chunks: (" << visible_chunks.size() << "//"<< /*chunked_points.get_filled_chunks_ids().size() <<*/ ")\n";
			
			if (active_label_prog) {
				cp_renderer.set_prog(*active_label_prog);
			}

			int rendered_eye = vr_view_ptr->get_rendered_eye();

			// on demand initialization of reduction buffers for incremental mode
			if (max_reduction_epoch != 0 || share_reduction_pass) {
				if (rendered_eye >= reduction_state.size()) {
					int old_size = reduction_state.size();
					int new_size = rendered_eye+1;
					reduction_state.resize(new_size);
					for (int i = old_size; i < new_size; ++i) {
						reduction_state[i].init(ctx);
						reduction_state[i].resize(max_points);
					}
				}
			}
				
			if (cp_renderer.enable(ctx, *reduction_view_transform_ptr * concat_mat_f)) {

				std::chrono::steady_clock::time_point start_reduce, stop_reduce;

				//detect vr devices based on render pass
				bool run_reduction = (!share_reduction_pass) || (rendered_eye == 0);

				// reduction phase of clod rendering, can run in incremental mode or all at once
				if (run_reduction) {
					// measure cpu and gpu side
					if (enable_performance_stats && rendered_eye == monitored_eye) {
						start_reduce = std::chrono::steady_clock::now();
						if (!poll_reduce_timer_data){
							glQueryCounter(reduce_timer_query[0], GL_TIMESTAMP);
						}
					}
					
					// if incremental reduction is used
					if (max_reduction_epoch) {
						int selected_eye = rendered_eye;
						if (share_reduction_pass) { //implicit && share_reduction_pass && vr_view_ptr->get_rendered_eye() == 0, see run_reduction
							selected_eye = 0;
						}

						auto& current_state = reduction_state[selected_eye];
						int current_epoch = current_state.reduction_epoch;
						int visible_ix = current_state.visible_buffer_ix;
						int index = visible_ix;
						int reduce_index = ((~visible_ix) & 0x01);

						// reset reduction state and initialize
						if (current_epoch == 0) {
							if (!skip_frustum_culling) {
								dmat4 projection = ctx.get_projection_matrix();
								//extend frustum for reduce
								projection(0, 0) *= 1.f / frustum_extend;
								projection(1, 1) *= 1.f / frustum_extend;

								dmat4 transform = projection * ctx.get_modelview_matrix();
									
								// extract frustum planes
								std::array<vec4, 6> frustum_planes;
								vec4 p4 = transform.row(3);
								for (int i = 0; i < 3; ++i) {
									frustum_planes[(i << 1)] = p4 - transform.row(i);
									frustum_planes[(i << 1) + 1] = p4 + transform.row(i);
								}
								// find visible chunks by frustum culling
								current_state.visible_chunks.clear();
								chunked_points.intersect_frustum(frustum_planes.data(), current_state.visible_chunks);
							}
							else {
								current_state.visible_chunks.clear();
								chunked_points.copy_chunk_ptrs(current_state.visible_chunks);
							}


							current_state.reduction_demand_processed = 0;
							current_state.chunk_iterator = current_state.visible_chunks.cbegin();
							unsigned num_frustum_points = 0;
							for (auto& ch : current_state.visible_chunks) {
								num_frustum_points += ch->size();
							}
							current_state.reduction_demand = num_frustum_points;

							cp_renderer.enable_buffer_manager(current_state.reduced_points[reduce_index]);
							cp_renderer.reduce_buffer_init(ctx, true);
						}
						else {
							// initialize reduction but keep draw parameters
							cp_renderer.enable_buffer_manager(current_state.reduced_points[reduce_index]);
							cp_renderer.reduce_buffer_init(ctx, false);
						}

						//compute a limit where to stop process further chunks
						int limit = std::max(1,(++current_epoch * (int)current_state.reduction_demand) / max_reduction_epoch);
								
						int processed = current_state.reduction_demand_processed;
						auto visible_chunks_end = current_state.visible_chunks.cend();
								
						// reduce chunks
						for (auto it = current_state.chunk_iterator; it != visible_chunks_end; ++it) {
							const chunk<LODPoint>& ch = **it;
							cp_renderer.reduce_buffer(ctx, ch.point_buffer(), ch.id_buffer(), 0, ch.size());
							processed += (int)ch.size();

							if (processed > limit) {
								current_state.chunk_iterator = ++it;
								break;
							}
						}
						current_state.reduction_demand_processed = processed;

						if (current_epoch >= max_reduction_epoch) {
							//swap buffers, reset
							current_state.visible_buffer_ix ^= 1;
							current_state.reduction_epoch = 0;
							//keep current buffers enabled for drawing
						}
						else {
							//reenable to draw buffers
							cp_renderer.enable_buffer_manager(current_state.reduced_points[index]);
							current_state.reduction_epoch = current_epoch;
						}
						cp_renderer.reduce_buffer_finish(ctx);
					}
					else {
						if (skip_frustum_culling) { // if frustum culling is disabled on chunk level
							if (share_reduction_pass) // store reduction in a extra buffer for reuse
								cp_renderer.enable_buffer_manager(reduction_state[0].reduced_points[0]);
							cp_renderer.reduce_buffer_init(ctx);
							//iterate all chunks
							for (auto& ic_pair : chunked_points) {
								auto& ch = *ic_pair.second;
								cp_renderer.reduce_buffer(ctx, ch.point_buffer(), ch.id_buffer(), 0, ch.size());
							}
							cp_renderer.reduce_buffer_finish(ctx);
						}
						else {
							if (share_reduction_pass) // store reduction in a extra buffer for reuse
								cp_renderer.enable_buffer_manager(reduction_state[0].reduced_points[0]);

							dmat4 projection = ctx.get_projection_matrix();
							dmat4 transform = projection * ctx.get_modelview_matrix();

							//extract frustum planes
							std::array<vec4, 6> frustum_planes;
							vec4 p4 = transform.row(3);
							for (int i = 0; i < 3; ++i) {
								frustum_planes[(i << 1)] = p4 - transform.row(i);
								frustum_planes[(i << 1) + 1] = p4 + transform.row(i);
							}


							// initialize reduce
							cp_renderer.reduce_buffer_init(ctx);

							if (draw_chunk_bounding_boxes)
								visible_chunks.clear();

							//find visible chunks and reduce
							for (auto& ic_pair : chunked_points) {
								auto& ch = *ic_pair.second;
								if (chunks<LODPoint>::intersects_frustum(frustum_planes.data(), ch)) {
									cp_renderer.reduce_buffer(ctx, ch.point_buffer(), ch.id_buffer(), 0, ch.size());
									//store which chunks are considered visible for visual debugging
									if (draw_chunk_bounding_boxes)
										visible_chunks.emplace_back(ic_pair.first, ic_pair.second.get());
								}
							}
							cp_renderer.reduce_buffer_finish(ctx);
						}
					}
				

					if (enable_performance_stats && rendered_eye == monitored_eye) {
						stop_reduce = std::chrono::steady_clock::now();
						diff_reduce_cpu = stop_reduce - start_reduce;
						if (!poll_reduce_timer_data) {
							glQueryCounter(reduce_timer_query[1], GL_TIMESTAMP);
							poll_reduce_timer_data = true;
						}
					}
							
				} // run reduction end
				

				// bind labels and color map buffers for drawing colored labels or color points based on lod
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, pct::point_cloud_server::point_labels_layout_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, color_map_layout_pos, color_map_buffer);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, lod_color_map_pos, lod_to_color_lut_buffer);
				
				if (enable_lights)
					glBindBufferBase(GL_SHADER_STORAGE_BUFFER, pct::point_cloud_server::point_normals_layout_pos, chunked_points.get_attribute(normal_attribute_id).buffer_name());
				glCheckError();
					
				if (share_reduction_pass) // enable stored buffer
				{
					if (max_reduction_epoch) {
						cp_renderer.enable_buffer_manager(reduction_state[0].reduced_points[reduction_state[0].visible_buffer_ix]);
					}
					else {
						cp_renderer.enable_buffer_manager(reduction_state[0].reduced_points[0]);
					}
				}
					


				if (enable_performance_stats && (rendered_eye == monitored_eye) && !poll_query_data) {
					// insert a query into the pipeline for the monitored eye. the results are read later in the init_frame method
					glQueryCounter(gl_render_query[0], GL_TIMESTAMP);
					cp_renderer.draw_points(ctx);
					glQueryCounter(gl_render_query[1], GL_TIMESTAMP);
					poll_query_data = true;
				} else {
					cp_renderer.draw_points(ctx);
				}	
					
				if (rendered_eye == 0 && clod_controller.targeted_visible_points) {
					clod_controller.control(source_point_cloud.ref_render_style(), cp_renderer.num_reduced_points(), 1.f / 90.f);
					update_member(&source_point_cloud.ref_render_style().CLOD);
				}

				cp_renderer.disable(ctx);
				cp_renderer.disable_buffer_manager();
			}

			if (draw_chunk_bounding_boxes) {
				std::vector<box3> chunk_bbs;
				std::vector<rgb> chunk_bbs_colors;
				for (auto& chunk : visible_chunks) {
					chunk_bbs.push_back(chunk.second->bounding_box());
					chunk_bbs_colors.emplace_back(0.1, 0.1, 0.87);
				}

				if (visible_chunks.size() > 0) {
					box_wire_renderer& br = ref_box_wire_renderer(ctx);
					br.set_render_style(box_style);
					br.set_box_array(ctx, chunk_bbs);
					br.set_color_array(ctx, chunk_bbs_colors);
					br.render(ctx, 0, chunk_bbs.size());
				}
			}
			//debug: show highlighted chunks
				
			if (highlighted_chunks.size() > 0){
				std::vector<box3> chunk_bbs;
				std::vector<rgb> chunk_bbs_colors;
				for (auto& chunk : highlighted_chunks) {
					chunk_bbs.push_back(chunk.second->bounding_box());
					chunk_bbs_colors.emplace_back(0.87, 0.1, 0.1);
				}

				box_wire_renderer& br = ref_box_wire_renderer(ctx);
				br.set_render_style(box_style);
				br.set_box_array(ctx, chunk_bbs);
				br.set_color_array(ctx, chunk_bbs_colors);
				br.render(ctx, 0, chunk_bbs.size());
			}
			if (highlighted_points.size() > 0) {
				auto& sr = ref_sphere_renderer(ctx);
				std::vector<rgb8> colors(highlighted_points.size(), rgb8(255, 10, 255));
				std::vector<float> radii(highlighted_points.size(), teleport_ray_radius);
				sr.set_position_array(ctx, highlighted_points);
				sr.set_color_array(ctx, colors);
				sr.set_radius_array(ctx, radii);
				sr.render(ctx, 0, highlighted_points.size());
			}
		}
		ctx.pop_modelview_matrix();


		//collect data
		if (enable_performance_stats){
			if (poll_query_data) {
				GLuint64 data[2] = { 0,0 };
				GLint stop_timer_available = 0;
				glGetQueryObjectiv(gl_render_query[1], GL_QUERY_RESULT_AVAILABLE, &stop_timer_available);

				if (stop_timer_available) {
					GLuint64 data[2] = { 0 ,0 };
					glGetQueryObjectui64v(gl_render_query[0], GL_QUERY_RESULT, &data[0]);
					glGetQueryObjectui64v(gl_render_query[1], GL_QUERY_RESULT, &data[1]);

					diff_draw = data[1] - data[0];
					poll_query_data = false;

					// record data
					draw_time.update(diff_draw);

					reduce_time_cpu.update(diff_reduce_cpu.count());
					avg_reduce_time_cpu_view = reduce_time_cpu.average<double>() * 1e3;  //in milli seconds
					avg_draw_time_view = draw_time.average<double>() / 1e6; //in milli seconds
					update_views(&avg_reduce_time_cpu_view);
					update_views(&avg_draw_time_view);
					if (draw_watch_file_ptr && log_draw_time && vr_view_ptr->get_rendered_eye() == 0) {
						*draw_watch_file_ptr << static_cast<double>(draw_time.average<double>()) / 1e6 << "\n";
					}
					if (reduce_cpu_watch_file_ptr && log_reduce_cpu_time && vr_view_ptr->get_rendered_eye() == 0) {
						*reduce_cpu_watch_file_ptr << static_cast<double>(reduce_time_cpu.average<double>()) * 1e3 << "\n";
					}
				}
			}
			//measure the reduce time on GPU
			if (poll_reduce_timer_data) {
				GLuint64 data[2] = { 0,0 };
				GLint stop_timer_available = 0;
				glGetQueryObjectiv(reduce_timer_query[1], GL_QUERY_RESULT_AVAILABLE, &stop_timer_available);
				if (stop_timer_available) {
					glGetQueryObjectui64v(reduce_timer_query[0], GL_QUERY_RESULT, &data[0]);
					glGetQueryObjectui64v(reduce_timer_query[1], GL_QUERY_RESULT, &data[1]);
					diff_reduce_gpu = data[1]-data[0];
					poll_reduce_timer_data = false;
					reduce_time_gpu.update(diff_reduce_gpu);
					avg_reduce_time_gpu_view = reduce_time_gpu.average<double>() / 1e6;
					update_views(&avg_reduce_time_gpu_view);
					if (reduce_watch_file_ptr && log_reduce_time && vr_view_ptr->get_rendered_eye() == 1) {
						*reduce_watch_file_ptr << static_cast<double>(reduce_time_gpu.average<double>()) / 1e6 << "\n";
					}
				}
			}
			//measure labeling time on GPU
			GLuint64 diff_labeling_gpu;
			if (point_server_ptr->poll_labeling_time(diff_labeling_gpu)) {
				labeling_time_gpu.update(diff_labeling_gpu);
				avg_labeling_time_gpu = labeling_time_gpu.average<double>() / 1e6;
				last_labeling_time_gpu = static_cast<double>(diff_labeling_gpu) / 1e6;
				update_views(&avg_labeling_time_gpu);
				update_views(&last_labeling_time_gpu);
				if (labeling_watch_file_ptr && log_labeling_gpu_time && vr_view_ptr->get_rendered_eye() == 1) {
					*labeling_watch_file_ptr << static_cast<double>(diff_labeling_gpu) / 1e6 << "\n";
				}
			}
		}
	}

	// draw intersection points
	if (!intersections.points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersections.points);
		sr.set_color_array(ctx, intersections.colors);
		sr.set_render_style(srs);
		sr.render(ctx, 0, intersections.points.size());
	}

	/*if (pc_1.get_nr_points() > 0)
	{
		auto& pr = ref_point_renderer(ctx);
		pr.set_render_style(point_style);

		size_t nr_points = (size_t)pc_1.get_nr_points();
		if (nr_points > 0) {
			pr.set_position_array(ctx, &pc_1.pnt(0), nr_points);
			pr.set_color_array(ctx, &pc_1.clr(0), nr_points);
			pr.render(ctx, 0, nr_points);
		}
	}

	if (pc_2.get_nr_points() > 0)
	{
		auto& pr = ref_point_renderer(ctx);
		pr.set_render_style(point_style);

		size_t nr_points = (size_t)pc_2.get_nr_points();
		if (nr_points > 0) {
			pr.set_position_array(ctx, &pc_2.pnt(0), nr_points);
			pr.set_color_array(ctx, &pc_2.clr(0), nr_points);
			pr.render(ctx, 0, nr_points);
		}
	}*/

	// draw vr stuff
	if (is_selecting_mode)
	{
		//render_candidate_modes(ctx,);
	}
	render_a_handhold_arrow(ctx, rgb(0.4), 0.1f);

	switch ((InteractionMode)interaction_mode) {
	case InteractionMode::TELEPORT: {
		if (vr_view_ptr) {
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			static constexpr int cid = 1;
			//test cylinder chunk intersection
			mat4 t_model = cgv::math::transpose(concat_mat);
			mat4 inv_model = cgv::math::inv(concat_mat);
			vec4 pos = inv_model * controller_poses[cid].col(3).lift();

			vec4 ray_direction = -controller_poses[cid].col(2).lift(); ray_direction.w() = 0;
			vec4 ray_orth_direction = controller_poses[cid].col(0).lift(); ray_direction.w() = 0;

			ray_direction = t_model * ray_direction;
			ray_orth_direction = t_model * ray_orth_direction;
		}
		if (vr_view_ptr && is_tp_floor) {
			std::array<vec3, 4> P;
			std::array<float, 4> R;
			std::array<rgb, 4> C;
			size_t array_index = 0;
			constexpr float ray_length = 1.2;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 1; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED && ci == 0) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P[array_index] = ray_origin;
					R[array_index] = 0.002f;
					P[array_index + 1] = ray_origin + ray_length * ray_direction;
					R[array_index + 1] = 0.003f;
					rgb c(float(1 - ci), 0.5f, float(ci));
					C[array_index] = c;
					C[array_index + 1] = c;
					array_index += 2;
				}
			}
			if (!P.empty() && !C.empty()) {
				auto& cr = cgv::render::ref_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				cr.set_position_array(ctx, P.data(), array_index);
				cr.set_color_array(ctx, C.data(), array_index);
				cr.set_radius_array(ctx, R.data(), array_index);
				cr.render(ctx, 0, P.size());
			}
		}
		{
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			vec3 ray_origin, ray_direction;
			state_ptr->controller[1].put_ray(&ray_origin(0), &ray_direction(0));
			auto& sr = ref_sphere_renderer(ctx);

			auto dir = cgv::math::pose_orientation(controller_poses[1]) * vec3(1.f, 0.f, 0.f);
			std::vector<vec3> ray;
			float ray_length = 1e3;
			rgb ray_color(1.f, 0.5f, 0.f);
			if (draw_teleport_destination) {
				float r = std::max(0.05, teleport_spere_radius_factor * teleport_destination_t);
				sr.set_position(ctx, teleport_destination_point);
				sr.set_radius_array(ctx, &r, 1);
				sr.set_color(ctx, rgb(0.3, 0.0, 0.5));
				sr.set_render_style(sphere_style_rhand);
				sr.render(ctx, 0, 1);
				ray_length = cgv::math::dot(ray_direction, teleport_destination_point - ray_origin);
				ray_color = rgb(0.f, 0.5f, 1.f);
			}

			//draw if trigger is pressed
			if (show_teleport_ray && vr_view_ptr->get_current_vr_state()->controller[1].axes[2] > throttle_threshold) {
				std::array<vec3, 2> P;
				std::array<float, 2> R;
				std::array<rgb, 2> C;

				P[0] = ray_origin + curr_offset_rhand; //offset so the ray begins at the arrows end
				P[1] = ray_origin + ray_length * ray_direction;
				R[0] = R[1] = teleport_ray_radius;
				C[0] = C[1] = ray_color;

				auto& cr = cgv::render::ref_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				cr.set_position_array(ctx, P.data(), 2);
				cr.set_color_array(ctx, C.data(), 2);
				cr.set_radius_array(ctx, R.data(), 2);
				cr.render(ctx, 0, P.size());
			}
			else {
				//render sphere on hand
				sr.set_position(ctx, controller_poses[1].col(3) + curr_offset_rhand);
				float r = 0.05;
				rgb c(0.3, 0.0, 0.5);
				sr.set_radius_array(ctx, &r, 1);
				sr.set_color_array(ctx, &c, 1);
				sr.set_render_style(sphere_style_rhand);
				sr.render(ctx, 0, 1);
			}

		}
		break;
	}
	case InteractionMode::LABELING: {
		if (palette_position == PalettePosition::HEADON)
		{
			palette.render_palette(ctx, hmd_trans_palette);
		}
		else if(palette_position == PalettePosition::LEFTHAND)
			palette.render_palette(ctx, controller_poses[palette_hand]);

		//set color
		rgba color;
		if (palette.object_id_is_valid(picked_sphere_index)) {
			color = palette.object_color(picked_sphere_index); //copy picked from picked sphere
		}
		else {
			color = rgba(0.4);// gray for invalid ids
		}


		//draw picked shape on right vr controller
		if (point_editing_tool == pallete_tool::PT_BRUSH) {
			switch (point_selection_shape) {
			case SS_SPHERE:
				render_palette_sphere_on_rhand(ctx, color);
				break;
			case SS_PLANE:
				render_palette_plane_on_rhand(ctx, color);
				break;
			case SS_CUBOID:
				render_palette_cube_on_rhand(ctx, color);
				break;
			}
		}
		else if (point_editing_tool == pallete_tool::PT_PASTE) {
			auto* record = clipboard_ptr->get_by_id(palette_clipboard_record_id);

			if (record != nullptr) {
				ctx.push_modelview_matrix();

				dmat4 model = record->model_matrix();
				ctx.set_modelview_matrix(ctx.get_modelview_matrix() * model);
				cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(*get_context());

				cp_renderer.enable(ctx);
				cp_renderer.reduce_buffer_init(ctx, true);
				cp_renderer.reduce_buffer(ctx, clipboard_paste_point_buffer, clipboard_paste_reduce_ids, 0, record->cached_lod_points.size());
				cp_renderer.reduce_buffer_finish(ctx);
				cp_renderer.draw_points(ctx);
				cp_renderer.disable(ctx);

				ctx.pop_modelview_matrix();
			}
		}
		else if (point_editing_tool == pallete_tool::PT_SELECTION) {
			//this is for copy selection
			glDisable(GL_CULL_FACE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			sphere_style_rhand.material.set_transparency(0.4);
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			float radius = box_shaped_selection.corner_node_radius();
			sr.set_render_style(sphere_style_rhand);
			sr.set_position_array(ctx, &(cgv::math::pose_position(controller_poses[1]) + curr_offset_rhand), 1);// picked_sphere_index changes in POSE event 
			sr.set_color_array(ctx, &color, 1);
			sr.set_radius_array(ctx, &radius, 1);
			sr.render(ctx, 0, 1);
			glDisable(GL_BLEND);
		}

		// draw selection / constraints box
		if (point_editing_tool == pallete_tool::PT_SELECTION || box_shaped_selection_is_constraint) {
			box_shaped_selection.draw(ctx);
		}

		break;
	}
	}

	// draw trajectory of hmd
	if (!trajectory_points.empty() && (tra_hmd || is_show_tra)) {
		auto& rcr = cgv::render::ref_cone_renderer(ctx);
		rcr.set_render_style(rcrs);
		std::vector<vec3> P;
		trajectory_color.clear();
		draw_hmd_tra(P, trajectory_color);
		rcr.set_position_array(ctx, P);
		rcr.set_color_array(ctx, trajectory_color);
		rcr.render(ctx, 0, P.size());
	}

	if (spacing_tool_enabled) {
		render_palette_sphere_on_rhand(ctx, rgba(0.4, 0.4, 0.8, 0.4));
	}
	
	if (show_text_labels) {
		text_labels.draw(ctx);
	}
	if (show_controller_labels) {
		controller_labels[0].update_coordinate_systems();
		controller_labels[0].draw(ctx);
		controller_labels[1].update_coordinate_systems();
		controller_labels[1].draw(ctx);
	}

	ctx.push_modelview_matrix();
	
	//ctx.mul_modelview_matrix(point_server_ptr->get_point_cloud_model_transform());
	//ctx.mul_modelview_matrix();
	//ctx.mul_modelview_matrix(cgv::math::scale4<float>(0.0005f, 0.0005f, 0.0005f));
	
	if (show_vertices) {
		/*sphere_renderer& sr = ref_sphere_renderer(ctx);
		sr.set_render_style(s_style);
		sr.set_position_array(ctx, M.get_positions());
		if (M.has_colors())
			sr.set_color_array(ctx, *reinterpret_cast<const std::vector<rgb>*>(M.get_color_data_vector_ptr()));
		sr.render(ctx, 0, M.get_nr_positions());*/
	}
	if (show_wireframe) {
		/*cone_renderer& cr = ref_cone_renderer(ctx);
		cr.set_render_style(c_style);
		if (cr.enable(ctx)) {
			mesh_info.draw_wireframe(ctx);
			cr.disable(ctx);
		}*/
	}
	if (show_surface) {
		draw_surface(ctx, false);
	}
	ctx.pop_modelview_matrix();
	/*auto stop_reduce = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff_d = stop_reduce - start_reduce;
	std::cout << "diff_d: " << diff_d.count() << std::endl;*/
}
///
void pointcloud_labeling_tool::finish_draw(cgv::render::context& ctx) {

}

void pointcloud_labeling_tool::on_device_change(void* kit_handle, bool attach)
{
	
}

void pointcloud_labeling_tool::render_candidate_modes(cgv::render::context& ctx)
{
	cgv::render::sphere_renderer& sr_mode = ref_sphere_renderer(ctx);
	sr_mode.set_render_style(sphere_style_lhand_modes);
	//sr_mode.set_position_array(ctx, );
	//sr_mode.set_color_array(ctx, );
	sr_mode.render(ctx, 0, 5);
}
///
void pointcloud_labeling_tool::render_a_handhold_arrow(cgv::render::context& ctx, rgb c, float r) {
	if (curr_offset_rhand.length() < 1e-6)
		return;
	
	float previous_scale = ars.length_scale;
	ars.length_scale = 1.0f;
	cgv::render::arrow_renderer& a_renderer = ref_arrow_renderer(ctx);
	a_renderer.set_render_style(ars);
	a_renderer.set_position_array(ctx, &cgv::math::pose_position(controller_poses[1]), 1);
	a_renderer.set_color_array(ctx, &rgb(0.4), 1);
	a_renderer.set_direction_array(ctx, &curr_offset_rhand, 1);
	a_renderer.render(ctx, 0, 1);
	ars.length_scale = previous_scale;
}

// step_width = 0.05
void add_left_sidebar(vrui::palette& palette, double toolbar_gap = -0.013, double step_width = 0.10) {
	int initial_offset_mult = -5;
	{// add select label to left toolbar
		auto id = palette.add_object(vrui::PaletteObject::PO_SPHERE, vec3(-3.0 * step_width + toolbar_gap, 0.1, initial_offset_mult++ * step_width), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
		palette.set_label_text(id, "add select");
		//function for select label
		static point_label_brush selection_brush(make_label(0, point_label_group::SELECTED_BIT), point_label_operation::OR, selection_shape::SS_NONE);
		palette.set_object_data(id, &selection_brush);
	}
	{// add protect label to left toolbar
		auto id = palette.add_object(vrui::PaletteObject::PO_SPHERE, vec3(-3.0 * step_width + toolbar_gap, 0.1, initial_offset_mult++ * step_width), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
		palette.set_label_text(id, "add protected");
		//function for select label
		static point_label_brush selection_brush(make_label(0, point_label_group::PROTECTED_BIT), point_label_operation::OR, selection_shape::SS_NONE);
		selection_brush.set_group_override((uint32_t)point_label_group::GROUP_MASK, 0);
		palette.set_object_data(id, &selection_brush);
	}
	uint32_t to_clear = (uint32_t)point_label_group::SELECTED_BIT | (uint32_t)point_label_group::PROTECTED_BIT;
	{// add clear select label to left toolbar
		auto id = palette.add_object(vrui::PaletteObject::PO_SPHERE, vec3(-3.0 * step_width + toolbar_gap, 0.1, initial_offset_mult++ * step_width), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
		palette.set_label_text(id, "clear select\nprotected");
		//function for select label
		static point_label_brush selection_brush(~make_label(0, to_clear), point_label_operation::AND, selection_shape::SS_NONE);
		selection_brush.set_group_override((uint32_t)point_label_group::GROUP_MASK, 0);
		palette.set_object_data(id, &selection_brush);
	}
	{// add clear all
		auto id = palette.add_object(vrui::PaletteObject::PO_SPHERE, vec3(-3.0 * step_width + toolbar_gap, 0.1, initial_offset_mult++ * step_width), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
		palette.set_label_text(id, "clear all\nselect/protected");
		//clear select and protected bits
		static point_label_brush selection_brush(~make_label(0, to_clear), point_label_operation::AND, selection_shape::SS_ALL);
		selection_brush.set_group_override((uint32_t)point_label_group::GROUP_MASK, 0);
		palette.set_object_data(id, &selection_brush);
	}
}

/// (left vr controller) render the palette(choosing a label),left bar(copy_paste), and top bar(selection primitive)
void pointcloud_labeling_tool::build_palette()
{
	// hard written color mapping, accessible form the shaders as shader storage buffer 
	PALETTE_COLOR_MAPPING.push_back(rgba(0.885186, 0.349231, 0.384895, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.501028, 0.51219, 0.540788, 1));
	//first label begins here
	PALETTE_COLOR_MAPPING.push_back(rgba(0.839713, 0.841112, 0.994662, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.622861, 0.963362, 0.480849, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.49487, 0.697245, 0.131001, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.548528, 0.51043, 0.207098, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.9294, 0.30196, 0.23137, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.38539, 0.212664, 0.725346, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.467858, 0.268185, 0.132797, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.679565, 0.246351, 0.739634, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.814578, 0.681682, 0.538812, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.779218, 0.928787, 0.738428, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.735197, 0.348423, 0.826778, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.689588, 0.102537, 0.711732, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.187419, 0.234203, 0.141554, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.946067, 0.555361, 0.838757, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.616379, 0.96377, 0.796525, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.626741, 0.889082, 0.406347, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.115997, 0.301431, 0.827358, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.329586, 0.839121, 0.77614, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.81568, 0.146095, 0.788965, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.54153, 0.9552, 0.787375, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.901813, 0.4714, 0.729169, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.224762, 0.242252, 0.592494, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.30714, 0.234365, 0.785558, 1));
	//reserve colors
	/*
	PALETTE_COLOR_MAPPING.push_back(rgba(0.289188, 0.443403, 0.213307, 1));
	*/

	
	//palette picking function
	std::function<void(picked_object)> palette_picking_func = [this](picked_object po) {
		int ix = po.id();
		paste_pointcloud_follow_controller = false;

		switch (po.object_group()) {
		case vrui::POG_NONE: { // it's one of the 25 spheres from the center
			picked_sphere_index = ix;
			point_selection_group_mask = default_point_selection_group_mask;
			point_selection_exclude_group_mask = default_point_selection_exclude_group_mask;

			switch (ix) {
			case 0: //special label for deleted points
				picked_label = make_label(0, point_label_group::DELETED);
				picked_label_operation = point_label_operation::REPLACE;
				point_selection_color = PALETTE_COLOR_MAPPING[0];
				break;
			case 1: //default label
				picked_label = make_label(0, point_label_group::VISIBLE);
				picked_label_operation = point_label_operation::REPLACE;
				point_selection_color = default_point_selection_color;
				break;
			default:
			{
				picked_label = make_label(ix - 1, point_label_group::VISIBLE);
				picked_label_operation = point_label_operation::REPLACE;
				//cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY> c = ix < PALETTE_COLOR_MAPPING.size() ? PALETTE_COLOR_MAPPING[ix] : default_point_selection_color;
				//c.H() = std::fmodf(c.H() + 0.5f, 1.f);
				point_selection_color = ix < PALETTE_COLOR_MAPPING.size() ? PALETTE_COLOR_MAPPING[ix] : default_point_selection_color;
			}
			}
			break;
		}
		case vrui::POG_TOP_TOOLBAR: {
			//this is the top bar of the Palette on left controller
			int i = po.position_in_group();
			if (i < 3) {
				point_selection_shape = (selection_shape)(i + 1);
				point_editing_tool = pallete_tool::PT_BRUSH;
			}
			else if (i == 3) {
				point_selection_shape = selection_shape::SS_NONE;
				point_editing_tool = pallete_tool::PT_SELECTION;
			}
			update_controller_labels(); //need to update grip label since editing tool may have changed
			break;
		}
		case vrui::POG_LEFT_TOOLBAR: {
			//this is the left bar of the Palette on left controller
			if (po.object_type() == PaletteObject::PO_SPHERE_FRAME) { // at the moment only the copy paste feature uses this shape 
				auto* record_ptr = clipboard_ptr->get_by_id(palette_clipboard_record_id);
				if (record_ptr) {
					// set paste tool
					point_selection_shape = selection_shape::SS_NONE;
					point_editing_tool = pallete_tool::PT_PASTE;
					paste_pointcloud_follow_controller = true;

					record_ptr->translation = -record_ptr->scale*record_ptr->centroid + controller_poses[1].col(3)+curr_offset_rhand;
					record_ptr->rotation = quat(1, 0, 0, 0);
					//record_ptr->scale = 1.f;
					auto& lod_points = record_ptr->lod_points();
					// load data into buffers for rendering
					glNamedBufferData(clipboard_paste_point_buffer, lod_points.size() * sizeof(LODPoint), lod_points.data(), GL_STATIC_DRAW);
					glNamedBufferData(clipboard_paste_reduce_ids, lod_points.size() * sizeof(GLint),nullptr, GL_STATIC_DRAW);
				}
			}
			else {
				auto* plb_ptr = static_cast<point_label_brush*>(po.data()); //per object defined operation, shape, label
				if (plb_ptr->operation != point_label_operation::NONE)
					picked_label_operation = plb_ptr->operation;
				if (plb_ptr->shape != selection_shape::SS_NONE)
					point_selection_shape = plb_ptr->shape;
				picked_label = plb_ptr->label;
				picked_sphere_index = ix;
				point_selection_color = default_point_selection_color;
				if (plb_ptr->group_override) {
					point_selection_group_mask = plb_ptr->point_groups;
					point_selection_exclude_group_mask = plb_ptr->excluded_point_groups;
				}
				else {
					point_selection_group_mask = default_point_selection_group_mask;
					point_selection_exclude_group_mask = default_point_selection_exclude_group_mask;
				}
			}
			update_controller_labels(); //need to update grip label, paste mode may be engaged or disengaged
			break;
		}
		}
	};
	
	//build palette
	palette.build(PALETTE_COLOR_MAPPING);
	palette.set_function(palette_picking_func);

	static constexpr double toolbar_gap = -0.013;
	static double constexpr step_width = 0.10;
	//add toolbars to pallete
	// 
	// add top toolbar brushes


	std::vector<vrui::PaletteObject> top_bar_shapes = { PO_SPHERE, PO_PLANE, PO_CUBOID, PO_BOX_FRAME };
	for (int i = 0; i < top_bar_shapes.size(); ++i) {
		auto id = palette.add_object(top_bar_shapes[i], vec3((i - 1.5) * step_width, 0.1, -6 * step_width + toolbar_gap), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_TOP_TOOLBAR);
	}
	//This is for adding left copy-paste menu bar
	if (show_left_bar) {
		add_left_sidebar(palette, toolbar_gap, step_width);
		{// clipboard
			vec3 position = vec3(-3.0 * step_width + toolbar_gap, 0.1, -1 * step_width);
			auto id = palette.add_object(vrui::PaletteObject::PO_SPHERE_FRAME, position, rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
			palette.set_label_text(id, "clipboard");

			palette_clipboard_point_cloud_id = palette.add_pointcloud(nullptr, nullptr, 0, position, quat(1.f, 0.f, 0.f, 0.f));
			//function for select label
			//static point_label_brush selection_brush(~make_label(0, point_label_group::SELECTED_BIT), point_label_operation::AND, selection_shape::SS_NONE);
			//palette.set_object_data(id, &selection_brush);
		}
	}
}

void pointcloud_labeling_tool::build_clipboard_palette()
{
	static const rgba remove_color = rgba(0.501028, 0.51219, 0.540788, 1);
	static const rgba remove_active_color = rgb8(212, 16, 16);

	static constexpr int clipboard_palette_width = 5, clipboard_palette_height = 5;

	//build rgbd_input palette
	rgbd_input_palette.build(clipboard_palette_width, clipboard_palette_height);

	//palette picking function
	std::function<void(picked_object)> palette_picking_func = [this](picked_object po) {
		int ix = po.id();
		paste_pointcloud_follow_controller = false;
		point_cloud_palette* palette_ptr = dynamic_cast<point_cloud_palette*>(po.get_palette_ptr());
		if (palette_ptr) {

			if (po.id() == rgbd_input_palette_delete_id) {
				rgbd_input_palette_delete_mode = !rgbd_input_palette_delete_mode;
				// change color of sphere
				rgbd_input_palette.object_color(rgbd_input_palette_delete_id) = rgbd_input_palette_delete_mode ? remove_active_color : remove_color;
				rgbd_input_palette.set_palette_changed();
				return;
			}

			switch (po.object_group()) {
			case vrui::POG_NONE: { // it's one of the 25 spheres from the center
				point_cloud_palette_slot* s = palette_ptr->find_slot_by_index(po.id());
				if (rgbd_input_palette_delete_mode) {
					// remove
					if (s->record_id != -1) {
						clipboard_ptr->erase(s->record_id);
						rgbd_input_palette_delete_mode = false;
						rgbd_input_palette.object_color(rgbd_input_palette_delete_id) = rgbd_input_palette_delete_mode ? remove_active_color : remove_color;
						rgbd_input_palette.set_palette_changed();
					}
				}
				else { // set as registration / paste point cloud
					point_cloud_registration.set_registration_pointcloud(s->record_id);
					point_cloud_registration.set_index(point_cloud_registration_tool::registration_slot);
					paste_pointcloud_follow_controller = true;
					rgbd_input_palette.highlight(po.id());
				}
				break;
			}
			default:
				return;
			}
		}
	};


	{// add delete to left toolbar
		this->rgbd_input_palette_delete_id = rgbd_input_palette.add_object(vrui::PaletteObject::PO_SPHERE, vec3(-3.0 * rgbd_input_palette.step_width + rgbd_input_palette.toolbar_gap, 0.1, -5 * rgbd_input_palette.step_width), rgba(0.501028, 0.51219, 0.540788, 1), vrui::POG_LEFT_TOOLBAR);
		rgbd_input_palette.set_label_text(rgbd_input_palette_delete_id, "remove\npoint cloud");
	}


	//add_left_sidebar(rgbd_input_palette, toolbar_gap, step_width);
	// use defined lambda as function called by picking
	rgbd_input_palette.set_function(palette_picking_func);
	// set up as event listener for the clipboard
	clipboard_ptr->register_event_listener(&rgbd_input_palette);
	// set a size limit for the clipboard to match the point cloud palette's size
	clipboard_ptr->limit_size(clipboard_palette_width*clipboard_palette_height);
}



/// (right vr controller) render the palette(only the sphere selection primitive)
void pointcloud_labeling_tool::render_palette_sphere_on_rhand(cgv::render::context& ctx, const rgba& color) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	sphere_style_rhand.material.set_transparency(0.4);
	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	sr.set_render_style(sphere_style_rhand);
	//sr.set_position_array(ctx, &(cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand), 1);// picked_sphere_index changes in POSE event 
	sr.set_position_array(ctx, &(cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(sphere_style_rhand.radius)), 1);// picked_sphere_index changes in POSE event 
	sr.set_color_array(ctx, &color, 1);
	sr.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
///
void pointcloud_labeling_tool::set_palette_toolbar_visibilty(bool visibility)
{
	palette.set_top_toolbar_visibility(visibility);
}
/// (right vr controller) render the palette(only the cutting plane selection primitive)
void pointcloud_labeling_tool::render_palette_plane_on_rhand(cgv::render::context& ctx, const rgba& color) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	plane_style_rhand.material.set_transparency(0.4);
	auto& cuber = cgv::render::ref_box_renderer(ctx);
	
	cuber.set_render_style(plane_style_rhand);
	cuber.set_box_array(ctx, &plane_box_rhand,1);
	cuber.set_rotation_array(ctx, &plane_orientation_rhand,1);
	cuber.set_color_array(ctx, &color, 1);
	cuber.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
/// (right vr controller) render the palette(only the cube selection primitive)
void pointcloud_labeling_tool::render_palette_cube_on_rhand(cgv::render::context& ctx, const rgba& color) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	cube_style_rhand.material.set_transparency(0.4);
	auto& cuber = cgv::render::ref_box_renderer(ctx);
	
	cuber.set_render_style(cube_style_rhand);
	cuber.set_box_array(ctx, &cube_rhand,1);
	auto& box_extent = cube_style_rhand.default_extent;
	vec3 cube_position = cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(cube_rhand.get_extent().z()*0.5f);
	cuber.set_translation_array(ctx, &cube_position,1);
	cuber.set_rotation_array(ctx, &cube_orientation_rhand,1);
			
	cuber.set_color_array(ctx, &color, 1); // gray means out of range 
	cuber.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
///
bool pointcloud_labeling_tool::handle(cgv::gui::event & e)
{
	//auto start_t = std::chrono::steady_clock::now();
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;
	
	auto& chunked_points = point_server_ptr->ref_chunks();

	switch (e.get_kind()) {
		case cgv::gui::EID_POSE:
		{
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();

			if (ci == -1) {
				hmd_last_pos = vrpe.get_last_position();
				hmd_pos = vrpe.get_position();
				hmd_last_ori = vrpe.get_last_orientation();
				hmd_ori = vrpe.get_orientation();
				hmd_ori3 = vrpe.get_orientation();
				hmd_pose34 = vrpe.get_pose_matrix();
				mat3 rot_palette;
				vec3 trans_palette = vec3(0.0, 0.0, 0.20);
				rot_palette.identity();
				rot_palette.set_col(0, vec3(1, 0, 0));
				rot_palette.set_col(1, vec3(0, 1, 0));
				rot_palette.set_col(2, vec3(0, 0, 1));
				hmd_trans_palette = cgv::math::pose_construct(rot_palette, cgv::math::pose_position(hmd_pose34) + trans_palette);
				//hmd_trans_palette.col(4) = hmd_trans_palette.col(4) + trans_palette;
				if (tra_hmd && vrpe.get_last_position()) {
					tra_las_points.push_back(hmd_last_pos);
					trajectory_points.push_back(hmd_pos);
					tra_las_ori.push_back(hmd_last_ori);
					trajectory_orientation.push_back(hmd_ori);
					post_redraw();
				}
			}

			// process pose events of the first two controllers
			if (ci < 0 || ci >= 2) {
				return true;
			}
			// store pose for later use
			controller_poses[ci] = vrpe.get_pose_matrix();
			if (ci != -1) {
				pos = vrpe.get_position();
				ori = vrpe.get_orientation();
			}

			// update offset on right hand
			if (ci == 1) {
				vec3 off = initial_offset_rhand;
				vrpe.get_quaternion().rotate(off);
				curr_offset_rhand = off;
			}
			// pick a mode
			if (is_selecting_mode)
			{
				vec3 picking_position_lhand = cgv::math::pose_position(controller_poses[mode_selection_hand]);
				float dist;
				//int nearest_mode_idx = mode_options.trigger_mode(picking_position_lhand, dist);
			}

			if (is_scaling) {
				static float movement_to_scale_factor = 1.00;

				// find some controller parameters
				int other_controller = 1 ^ ci; //(ci == 0) ? 1 : 0;
				float dist_controllers = (controller_poses[0].col(3) - controller_poses[1].col(3)).length();
				float old_dist_controllers = (vrpe.get_last_pose_matrix().col(3) - controller_poses[other_controller].col(3)).length();

				//scale point cloud

				float scale_delta = (dist_controllers - old_dist_controllers) * movement_to_scale_factor;

				
				{ //scale source_pc
					float new_scale = point_server_ptr->ref_point_cloud_scale() * std::exp(scale_delta);
					// build model_transform
					mat4 model_transform = cgv::math::translate4(point_server_ptr->ref_point_cloud_position())
						* cgv::math::rotate4<float>(point_server_ptr->ref_point_cloud_rotation())
						* cgv::math::scale4(point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale());

					rescale_point_cloud(new_scale);

					mat4 new_model_transform = cgv::math::translate4(point_server_ptr->ref_point_cloud_position())
						* cgv::math::rotate4<float>(point_server_ptr->ref_point_cloud_rotation())
						* cgv::math::scale4(point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale());

					vec4 center = (0.5f * (controller_poses[0].col(3) + controller_poses[1].col(3))).lift();
					vec4 center_in_model_cs = inv(model_transform) * center;
					vec4 new_center = new_model_transform * center_in_model_cs;
					point_server_ptr->ref_point_cloud_position() += (vec3)(center - new_center);
					// TODO: add adaptive spacing adjustment
					if (last_measured_l0_spacing > 0.0 && last_measured_l0_spacing < std::numeric_limits<float>::infinity()) {
						source_point_cloud.ref_render_style().scale = new_scale;
						update_member(&source_point_cloud.ref_render_style().scale);
					}
				}

			}
			// rotate the model by gripping the right controller
			if (is_rotating_moving && ci == 1) {
				vec3 movement = controller_poses[1].col(3) - vrpe.get_last_pose_matrix().col(3);
				//calculate rotation delta
				quat to = quat(mat3(3, 3, controller_poses[1].begin()));
				quat from = quat(mat3(3, 3, vrpe.get_last_pose_matrix().begin()));
				quat delta_rotation = to * from.conj(); //delta_rotation = to*inv(from), conjugate of rotation quaternion == inverse of rotation quaternion
				// move pointcloud from RGBD_INPUT tool
				{ // move source_pc
					mat4 concat_mat = point_server_ptr->get_point_cloud_model_transform();

					mat4 linear_transform = cgv::math::translate4(controller_poses[1].col(3) + movement) * delta_rotation.get_homogeneous_matrix() * cgv::math::translate4(-controller_poses[1].col(3)) * concat_mat;

					mat3 rotation;
					for (int j = 0; j < 3; ++j) {
						for (int i = 0; i < 3; ++i) {
							rotation(i, j) = linear_transform(i, j);
						}
						rotation(j, j) /= point_server_ptr->ref_point_cloud_scale();
					}
					quat q_rotation = quat(rotation);
					vec4 point_cloud_translation = linear_transform.col(3);
					point_server_ptr->ref_point_cloud_rotation() = cgv::math::rad2deg(to_euler_angels(q_rotation));
					point_server_ptr->ref_point_cloud_position() = vec3(point_cloud_translation.x(), point_cloud_translation.y(), point_cloud_translation.z());
					if (box_shaped_selection.phase() == box_selection_phase::FIRST_POINT_CONFIRMED || box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED) {
						//std::cout << "ori:" << box_shaped_selection.orientation() << std::endl;

						//compute the matrix between point cloud and box
						box_shaped_selection.point(0) = vec3(point_cloud_translation.x(), point_cloud_translation.y(), point_cloud_translation.z()) + box_shaped_selection.point(0);
						box_shaped_selection.point(1) = vec3(point_cloud_translation.x(), point_cloud_translation.y(), point_cloud_translation.z()) + box_shaped_selection.point(1);
						box_shaped_selection.orientation() = q_rotation * box_shaped_selection.orientation();
						//box_shaped_selection.translation() = vec3(point_cloud_translation.x(), point_cloud_translation.y(), point_cloud_translation.z()) + box_shaped_selection.translation();
						box_shaped_selection.update_selection();
					}
				}
			}

			// pick a label 
			if ((InteractionMode)interaction_mode == InteractionMode::LABELING)
			{
				// handle/check palette object picking
				vec3 picking_position_rhand = cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand;
				float dist;
				// check for picked object, calls function given by palette.set_function(f) if something was picked
				int nearest_palette_idx = palette.trigger_object(picking_position_rhand, controller_poses[palette_hand], dist);

				// continue based labeling tool mode
				if (point_editing_tool == pallete_tool::PT_SELECTION) {
					if (box_shaped_selection.phase() == box_selection_phase::FIRST_POINT_CONFIRMED) {
						box_shaped_selection.orientation() = quat(mat3(3, 3, vr_view_ptr->get_current_vr_state()->controller[0].pose));
					}
					//check if controller is near enough to a corner node
					if (box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED) {
						selection_touched_corner = -1;
						auto& corners = box_shaped_selection.get_corners();
						for (int i = 0; i < corners.size(); ++i) {
							const vec3& corner = corners[i];
							float current_dist = (corner - picking_position_rhand).length();
							if (current_dist < box_shaped_selection.corner_node_radius()) {
								selection_touched_corner = i;
								break;
							}
						}
					}
				}
				else if (point_editing_tool == pallete_tool::PT_PASTE) {
					if (paste_pointcloud_follow_controller && ci == 1) {
						// rotate and translate clipboard pointcloud
						vec3 movement = controller_poses[1].col(3) - vrpe.get_last_pose_matrix().col(3);
						quat to = quat(mat3(3, 3, controller_poses[1].begin()));
						quat from = quat(mat3(3, 3, vrpe.get_last_pose_matrix().begin()));
						quat delta_rotation = to * from.conj();

						auto* record_ptr = clipboard_ptr->get_by_id(palette_clipboard_record_id);
						if (record_ptr) {
							//float& scale = record_ptr->scale;
							mat4 rotation = mat4(3, 3, record_ptr->rotation.get_matrix().begin());
							//mat4 model = cgv::math::translate4(record_ptr->translation)* rotation * cgv::math::scale4(scale,scale,scale);
							mat4 model = cgv::math::translate4(record_ptr->translation) * rotation;

							mat4 linear_transform = cgv::math::translate4(controller_poses[1].col(3)) * delta_rotation.get_homogeneous_matrix() * cgv::math::translate4(-vrpe.get_last_pose_matrix().col(3)) * model;


							mat3 final_rotation;
							for (int j = 0; j < 3; ++j) {
								for (int i = 0; i < 3; ++i) {
									final_rotation(i, j) = linear_transform(i, j);
								}
								//final_rotation(j, j) /= scale;
							}
							quat point_cloud_rotation = quat(final_rotation);
							vec4 point_cloud_translation = linear_transform.col(3);

							record_ptr->rotation = point_cloud_rotation;
							record_ptr->translation = point_cloud_translation;
						}
					}
				}

			}
			//update shapes for rendering
			{
				//PLANE
				plane_box_rhand = box3(vec3(-0.001f, -1.f, -1.f), vec3(0.001f, 1.f, 1.f));
				plane_box_rhand.translate(cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand);
				plane_orientation_rhand = cgv::math::pose_orientation(controller_poses[point_selection_hand]);
				//CUBOID
				float cube_height = cube_rhand_flat ? cube_length * 0.125f : cube_length;
				cube_rhand = box3(vec3(-cube_length, -cube_height, -cube_length), vec3(cube_length, cube_height, cube_length));
				cube_orientation_rhand = cgv::math::pose_orientation(controller_poses[point_selection_hand]);
			}

			/// repeating actions while the trigger is held
			auto& state = vrpe.get_state();
			if (ci == 1) {
				if (state.controller[1].axes[2] > 0.25 || start_l) {
					// label points if trigger is pressed
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						if (point_editing_tool == pallete_tool::PT_BRUSH) {
							cgv::render::context& ctx = *get_context();

							std::chrono::steady_clock::time_point start_labeling, stop_labeling;
							start_labeling = std::chrono::steady_clock::now();

							point_server_ptr->ref_interaction_settings() = point_cloud_interaction_settings;

							switch (point_selection_shape) {
							case SS_SPHERE:
								point_server_ptr->label_points_in_sphere(picked_label, point_selection_group_mask, point_selection_exclude_group_mask, cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(sphere_style_rhand.radius), sphere_style_rhand.radius, picked_label_operation);
								break;
							case SS_PLANE:
								point_server_ptr->label_points_by_clipping(picked_label, point_selection_group_mask, point_selection_exclude_group_mask,
									cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand,
									cgv::math::pose_orientation(controller_poses[point_selection_hand]), plane_invert_selection, picked_label_operation);
								break;
							case SS_CUBOID:
								point_server_ptr->label_points_in_box(picked_label, point_selection_group_mask, point_selection_exclude_group_mask,
									cube_rhand, cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(cube_rhand.get_extent().z()*0.5f), cube_orientation_rhand, picked_label_operation);
								break;
							case SS_ALL:
								//disable rollback for this operation
								bool history_flag = point_cloud_interaction_settings.enable_history;
								point_cloud_interaction_settings.enable_history = false;
								point_server_ptr->label_all_points(picked_label, point_selection_group_mask, point_selection_exclude_group_mask, picked_label_operation);
								point_cloud_interaction_settings.enable_history = history_flag;
							}

							stop_labeling = std::chrono::steady_clock::now();
							diff_label_cpu = stop_labeling - start_labeling;
							if (enable_performance_stats) {
								labeling_time_cpu.update(diff_label_cpu.count());
								last_labeling_time_cpu = labeling_time_cpu.last() * 1e3;
								avg_labeling_time_cpu = labeling_time_cpu.average<double>() * 1e3;
								update_views(&avg_labeling_time_cpu);
								update_views(&last_labeling_time_cpu);
								if (labeling_cpu_watch_file_ptr && log_labeling_cpu_time) {
									*labeling_cpu_watch_file_ptr << diff_label_cpu.count() * 1e3 << "\n";
								}
							}
						}
						else if (point_editing_tool == pallete_tool::PT_SELECTION) {
							//if (box_shaped_selection.phase() == box_selection_phase::NO_POINT_CONFIRMED) {
								//first point is set in on_throttle_threshold method
							//}
							if (box_shaped_selection.phase() == box_selection_phase::FIRST_POINT_CONFIRMED) {
								//set second point as long trigger is held
								box_shaped_selection.point(1) = cgv::math::pose_position(controller_poses[ci]) + shape_offset(0.f);
								//box_shaped_selection.orientation() = vrpe.get_quaternion();
								box_shaped_selection.update_selection();
							}
							else if (box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED) {
								if (selection_touched_corner >= 0) {
									//change selection box if touched at corner
									vec3 picking_position = cgv::math::pose_position(controller_poses[point_selection_hand]) + curr_offset_rhand;
									box_shaped_selection.set_corner(selection_touched_corner, picking_position);
									if (box_shaped_selection_is_constraint)
										point_server_ptr->set_constraint_box((GLint)point_label_group::PROTECTED_BIT, box_shaped_selection.box(), box_shaped_selection.translation(), box_shaped_selection.orientation(), true);
								}
							}
						}
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT) {
						static constexpr int cid = 1;
						//draw teleport destination
						draw_teleport_destination = point_server_ptr->raycast_points(-controller_poses[cid].col(2), controller_poses[cid].col(3), teleport_ray_radius, teleport_destination_t, teleport_destination_point);
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) 
					{
						if (config_mode_tool == (int)ConfigModeTools::CMT_PointSpacingProbe) {
							static constexpr int cid = 1;
							cgv::render::context& ctx = *get_context();
							last_measured_position = controller_poses[cid].col(3) + shape_offset(sphere_style_rhand.radius);
							auto num_points = point_server_ptr->count_points_in_sphere(last_measured_position, sphere_style_rhand.radius);
							int total_points = 0, max_lod_level = 0;
							// num_points refers to the numbers of each level within the sphere range
							for (int i = 0; i < num_points.size(); ++i) {
								if (num_points[i] > 0)
									max_lod_level = i; //implicit max due to iteration order
								total_points += num_points[i];
							}

							auto compute_density = [&](int num_points) {
								//std::cout << num_points << " point in sphere\n";
								float point_density = static_cast<double>(num_points) / (PI * std::pow(sphere_style_rhand.radius, 2.0));
								//std::cout << point_density << " point density\n";
								return point_density;
							};

							//std::cout << "Total\n";
							double total_point_density = compute_density(total_points);
							float total_point_spacing = std::sqrt(1.0 / total_point_density);
							//std::cout << total_point_spacing << " point spacing\n";

							//double l0_density = total_point_density / std::pow(2, max_lod_level);
							double l0_spacing = std::sqrt(1.0 / total_point_density) * std::pow(2, max_lod_level);
							//std::cout << "Point spacing computed for Level 0: " << l0_spacing << "\n";

							last_measured_l0_spacing = (float)l0_spacing;
							last_measured_point_density = (float)total_point_density;
							last_measured_point_spacing = total_point_spacing;

							//update text label
							std::array<char, 512> buff;
							std::snprintf(buff.data(), buff.size(), spacing_tool_template_str.data(), last_measured_position.x(), last_measured_position.y(), last_measured_position.z(), last_measured_point_density, last_measured_point_spacing, last_measured_l0_spacing);
							text_labels.update_label_text(spacing_tool_label_id, std::string(buff.data()));
							text_labels.place_label(spacing_tool_label_id,
								right_tool_label_offset, tool_label_ori, CS_RIGHT_CONTROLLER, LA_LEFT, tool_label_scale);
						}
					}
				}
			}
		}
		case cgv::gui::EID_KEY:
		{
			int trackpad_direction = 0; //1 for right or up and -1 for left or down
			//read out some buttons that need to be holded down for somthing to happen.
			//The state may have changed without noticing the application, e.g. a controller got disconnected before releasing a button and the button relase event is never triggered.
			{
				const vr::vr_kit_state* state = vr_view_ptr->get_current_vr_state();
				if (state) {
					if ((interaction_mode == (int)InteractionMode::LABELING) ||
						(interaction_mode == (int)InteractionMode::TELEPORT)) {
						//first controller allows scaling
						is_scaling = state->controller[0].button_flags & vr::VRF_GRIP;
						//second does movement and rotating
						is_rotating_moving = state->controller[1].button_flags & vr::VRF_GRIP;
					}
					else {
						is_scaling = false;
						is_rotating_moving = false;
					}
				}
			}

			static const float angle = std::asin(1.f) / 3.f;
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);

			if (vrke.get_action() == cgv::gui::KA_RELEASE) {
				return true;
			}
			switch (vrke.get_key()) {
				//left-hand menu key event: undo operation
				//right-hand menu key event: selecting the second point of wireframe box, copy points; set scale corrected root level point spacing
			case vr::VR_MENU:
				if (vrke.get_controller_index() == 0)
				{
					rollback_last_operation(*get_context());
				}
				if (vrke.get_controller_index() == 1)
				{
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						if (chunked_points.num_chunks() > 0) {
							if (point_editing_tool == pallete_tool::PT_SELECTION) {
								//the menu button can label points inside the box with the picked label, this is for the wireframe box selector
								if (box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED)
								{
									point_server_ptr->ref_interaction_settings() = point_cloud_interaction_settings;
									point_server_ptr->label_points_in_box(picked_label, point_selection_group_mask, point_selection_exclude_group_mask, box_shaped_selection.box(), box_shaped_selection.translation(), box_shaped_selection.orientation(), point_label_operation::REPLACE);
									if (point_cloud_interaction_settings.enable_history)
										history_ptr->add_rollback_operation();
								}
							}
							else {
								// copy points
								auto& collected = collect_points(*this->get_context(), (int)point_label_group::SELECTED_BIT);
								auto& labels_ref = chunked_points.get_attribute(label_attribute_id);

								move_points_to_clipboard(collected.first, collected.second, labels_ref.data<GLint>());
								if (clear_selection_labels_after_copy) {
									//clear labels by using the logical AND-operation on all points labels
									point_server_ptr->label_all_points(~(GLint)point_label_group::SELECTED_BIT, (int32_t)point_label_group::SELECTED_BIT, 0, point_label_operation::AND);
								}
							}
						}
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						//set scale corrected root level point spacing
						if (last_measured_l0_spacing > 0.0 && last_measured_l0_spacing < std::numeric_limits<float>::infinity()) {
							source_point_cloud.ref_render_style().spacing = last_measured_l0_spacing / source_point_cloud.ref_render_style().scale;
							update_member(&source_point_cloud.ref_render_style().spacing);
						}
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT) {
						//stores the pointclouds transformation state, restores on teleport
						//stored_state.store_state(source_pc);
						stored_state.first = point_server_ptr->get_transformation_state();
						stored_state.second = source_point_cloud.ref_render_style();
						overview_mode = true;
						on_point_cloud_fit_table();
						vr_view_ptr->set_tracking_origin(vec3(0.f, 0.f, 0.f));
					}
				}
				break;
			case vr::VR_DPAD_LEFT:
				trackpad_direction = (trackpad_direction == 0) ? -1 : trackpad_direction;
			case vr::VR_DPAD_RIGHT:
				trackpad_direction = (trackpad_direction == 0) ? 1 : trackpad_direction;
				if (vrke.get_controller_index() == 1) {
					if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						if (source_point_cloud.get_nr_points() > 0)
						{
							source_point_cloud.ref_render_style().spacing += trackpad_direction*0.01;
							update_member(&source_point_cloud.ref_render_style().spacing);
							update_clod_parameters_label();
						}
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						if (point_editing_tool == pallete_tool::PT_BRUSH) {
							do {
								point_selection_shape = (selection_shape)((point_selection_shape + trackpad_direction) % selection_shape::NUM_OF_SHAPES);
								if (point_selection_shape < 0)
									point_selection_shape = (selection_shape)(point_selection_shape + selection_shape::NUM_OF_SHAPES);
							} while (selection_shape_blacklist.find(point_selection_shape) != selection_shape_blacklist.end());
							
							assert(point_selection_shape > -1);
						}
						else if (point_editing_tool == pallete_tool::PT_SELECTION) {
							//deselect points
							if (trackpad_direction == -1) { //press left for clear selection
								box_shaped_selection.clear_points();
								if (box_shaped_selection_is_constraint)
									point_server_ptr->clear_constraints();
							}
							else if (trackpad_direction == 1) {//press right to toggle constraint mode
								box_shaped_selection_is_constraint = !box_shaped_selection_is_constraint;
								update_controller_labels();
								if (box_shaped_selection_is_constraint) {
									point_server_ptr->set_constraint_box((GLint)point_label_group::PROTECTED_BIT, box_shaped_selection.box(), box_shaped_selection.translation(), box_shaped_selection.orientation(), true);
								}
								else {
									point_server_ptr->clear_constraints();
								}
							}
						}
						else if (point_editing_tool == pallete_tool::PT_PASTE) {
							if (vrke.get_controller_index() == 1 && chunked_points.num_chunks() > 0)
							{
								auto* record_ptr = clipboard_ptr->get_by_id(palette_clipboard_record_id);
								if (record_ptr->points)
								{
									// paste pointcloud and disengage paste mode
									auto model_mat = record_ptr->model_matrix();
									point_server_ptr->fuse_point_cloud(*record_ptr->points, model_mat, true);
									point_editing_tool = pallete_tool::PT_BRUSH;
									update_controller_labels();
								}
							}
						}
						//palette_element_shapes[picked_sphere_index] = point_selection_shape != selection_shape::SS_NONE ? point_selection_shape : selection_shape::SS_SPHERE;
						//update_palette();
					}
					
				}
				else if (vrke.get_controller_index() == 0) {
					//left hand trackpad: left-right to change the RGBD mode: PULL, FUSE, ICP
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						//copy selection to clipboard
						if (trackpad_direction < 0) {
							auto& collected = collect_points(*this->get_context(), (int)point_label_group::SELECTED_BIT);
							auto& labels_ref = chunked_points.get_attribute(label_attribute_id);
							copy_points_to_clipboard(collected.first, collected.second, labels_ref.data<GLint>());
						}
					}
				}
				break;
			case vr::VR_DPAD_UP:
				trackpad_direction = (trackpad_direction == 0) ? 1 : trackpad_direction;
			case vr::VR_DPAD_DOWN:
				trackpad_direction = (trackpad_direction == 0) ? -1 : trackpad_direction;
				//switch interaction mode
				if (vrke.get_controller_index() == 0) {
					interaction_mode = (interaction_mode + trackpad_direction) % (int)(NUM_OF_INTERACTIONS);
					if (interaction_mode < 0)
						interaction_mode = interaction_mode + (int)InteractionMode::NUM_OF_INTERACTIONS;
					std::cout << "changed to interaction mode " << interaction_mode << std::endl;
					update_interaction_mode((InteractionMode)interaction_mode);
				}
				//move
				if (vrke.get_controller_index() == 1) {
					c_pos = vr_view_ptr->get_tracking_origin();
					//std::cout << "pos: " << c_pos << std::endl;
					
					p.position = ori * vec3(0.0f, 0.0f, -trackpad_direction*1.0f) * 0.1f + c_pos;
					p.color = rgb(1.0, 0.0, 0.0);
					p.radius = 0.15f;
					vr_view_ptr->set_tracking_origin(p.position);
				}

				break;
			case vr::VR_GRIP: 
				{
					switch (interaction_mode) {
					case InteractionMode::LABELING:
					{
						if (point_editing_tool == pallete_tool::PT_BRUSH) {
							switch (point_selection_shape) {
							case selection_shape::SS_PLANE: {
								plane_invert_selection = !plane_invert_selection;
								break;
							}
							/*case selection_shape::SS_CUBOID: {
								cube_rhand_flat = !cube_rhand_flat;
								break;
							}*/
							}
						}
						else if (point_editing_tool == pallete_tool::PT_SELECTION) {
							if (box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED)
							{
								box_shaped_selection_is_constraint = !box_shaped_selection_is_constraint;
								if (box_shaped_selection_is_constraint) {
									point_server_ptr->set_constraint_box((GLint)point_label_group::PROTECTED_BIT, box_shaped_selection.box(), box_shaped_selection.translation(), box_shaped_selection.orientation(), true);
								}
								else {
									point_server_ptr->clear_constraints();
								}
							}
						}
						break;
					}
					case  InteractionMode::TELEPORT:
					{
						static constexpr int cid = 1;
						const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
						vec3 ray_origin, ray_direction;
						state_ptr->controller[cid].put_ray(&ray_origin(0), &ray_direction(0));
						teleport_ray(ray_direction, ray_origin, teleport_ray_radius);
						draw_teleport_destination = false;
						break;
					}
					case InteractionMode::CONFIG : {
						config_mode_tool = ((int)config_mode_tool + 1) % (int)ConfigModeTools::CMT_NUM_ENUMS;
						update_interaction_mode(InteractionMode::CONFIG);
						break;
					}
					}
					break;
				}
			}
			break;
		}
	}

	if (e.get_kind() == cgv::gui::EID_STICK) {
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		// left-hand controller: left-right touching event: adjusting size of selection primitive
		// right - hand controller : config mode : left - right touching : adjust point size
		if (vrse.get_controller_index() == 0)
		{
			if (vrse.get_action() == cgv::gui::SA_MOVE) {
				//clamp to -1.0, 1.0
				static constexpr float threshold = 0.40f;
				//zero if sub threashold otherwise sign(vrse.get_y())*1.f
				float factor = abs(vrse.get_x()) > threshold ? (cgv::math::sign(vrse.get_x()) >= 0 ? 1.f : -1.f) : 0;

				selection_shape shape = selection_shape::SS_NONE;
				if ((InteractionMode)interaction_mode == InteractionMode::LABELING){ 
					shape = point_selection_shape;
				}
				if (factor != 0) {
					switch (shape) {
					case SS_SPHERE:
						sphere_style_rhand.radius += factor * radius_adjust_step;
						sphere_style_rhand.radius = std::max(0.f, sphere_style_rhand.radius);
						break;
					case SS_CUBOID:
						cube_length += factor * radius_adjust_step;
						cube_length = std::max(0.f, cube_length);
						cube_rhand = box3(vec3(-cube_length, -cube_length, -cube_length), vec3(cube_length, cube_length, cube_length));
						break;
					case SS_PLANE:
						//nothing to do here
						break;
					}
				}

				if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
					if (source_point_cloud.get_nr_points() > 0)
					{
						if (vrse.get_x() > 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5)
						{
							source_point_cloud.ref_render_style().pointSize += 0.01;
							update_member(&source_point_cloud.ref_render_style().pointSize);
							// must be updated
							update_clod_parameters_label();
						}
						else if (vrse.get_x() < 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5)
						{
							source_point_cloud.ref_render_style().pointSize -= 0.01;
							update_member(&source_point_cloud.ref_render_style().pointSize);
							update_clod_parameters_label();
						}
					}
				}
			}
		}
		// config mode: adjust the CLOD factor
		if (vrse.get_controller_index() == 1)
		{
			if (vrse.get_action() == cgv::gui::SA_MOVE) {
				if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) 
				{
					if (config_mode_tool == (int)ConfigModeTools::CMT_PointSpacingProbe) 
					{
						// adjust primitive size in spacing probe mode
						static constexpr float threshold = 0.40f;
						//zero if sub threashold otherwise sign(vrse.get_y())*1.f
						float factor = abs(vrse.get_x()) > threshold ? (cgv::math::sign(vrse.get_x()) >= 0 ? 1.f : -1.f) : 0;
						sphere_style_rhand.radius += factor * radius_adjust_step;
						sphere_style_rhand.radius = std::max(0.f, sphere_style_rhand.radius);
					}
					else if (config_mode_tool == (int)ConfigModeTools::CMT_CLODParameters) 
					{
						if (source_point_cloud.get_nr_points() > 0)
						{
							if (vrse.get_x() > 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5)
							{
								source_point_cloud.ref_render_style().CLOD += 0.01;
								update_member(&source_point_cloud.ref_render_style().CLOD);
								update_clod_parameters_label();
							}
							else if (vrse.get_x() < 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5) {
								source_point_cloud.ref_render_style().CLOD -= 0.01;
								update_member(&source_point_cloud.ref_render_style().CLOD);
								update_clod_parameters_label();
							}
						}
					}


				}
			}
		}
	}
	
	if (e.get_kind() == cgv::gui::EID_THROTTLE) {
		auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
		int ci = te.get_controller_index();
		float v = te.get_value();
		//bool d = (v == 1); // event 
		/*if (ci == 1) { // right hand 
			if (v > 0) {

			}
		}*/
		
		if (last_throttle_state[ci] < throttle_threshold && v >= throttle_threshold) {
			on_throttle_threshold(ci, true);
		}
		else if (last_throttle_state[ci] >= throttle_threshold && v < throttle_threshold) {
			on_throttle_threshold(ci, false);
		}
		last_throttle_state[te.get_controller_index()] = v;
		return true;
	}
	return false;
}


void pointcloud_labeling_tool::on_throttle_threshold(const int ci, const bool low_high) {

	auto& chunked_points = point_server_ptr->ref_chunks();

	if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
		if (point_editing_tool == pallete_tool::PT_PASTE) {
			if (ci == 1)
				paste_pointcloud_follow_controller = low_high; // make active copy paste pointcloud follow the controller if there was an low high transition
		} else if (point_editing_tool == pallete_tool::PT_SELECTION) {
			if (ci == 1) {
				if (low_high) {
					if (box_shaped_selection.phase() == box_selection_phase::NO_POINT_CONFIRMED) {
						box_shaped_selection.phase() = box_selection_phase::FIRST_POINT_CONFIRMED;
						box_shaped_selection.update_orientation(true);
						box_shaped_selection.point(0) = cgv::math::pose_position(controller_poses[ci]) + shape_offset(0.f);

						box_shaped_selection.orientation() = quat(mat3(3, 3, vr_view_ptr->get_current_vr_state()->controller[0].pose));
						box_shaped_selection.update_selection();
					}
					else if (box_shaped_selection.phase() == box_selection_phase::SECOND_POINT_CONFIRMED && selection_touched_corner == -1) {
						box_shaped_selection.phase() = box_selection_phase::NO_POINT_CONFIRMED;
						if (box_shaped_selection_is_constraint)
							point_server_ptr->clear_constraints();
					}
				}
				else {
					if (box_shaped_selection.phase() == box_selection_phase::FIRST_POINT_CONFIRMED) {
						box_shaped_selection.phase() = box_selection_phase::SECOND_POINT_CONFIRMED;
						box_shaped_selection.update_orientation(false);
						if (box_shaped_selection_is_constraint)
							point_server_ptr->set_constraint_box((GLint)point_label_group::PROTECTED_BIT, box_shaped_selection.box(), box_shaped_selection.translation(), box_shaped_selection.orientation(), true);

					}
				}
			}
		}
	}
	else if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
		if (low_high) {
			// print pose information of right controller
			const float* position_ptr = vr_view_ptr->get_current_vr_state()->controller[ci].pose + 9;
			//std::cout << position_ptr[0] << "," << position_ptr[1] << "," << position_ptr[2] << "\n";
		}
	}

	// !low_high means release the trigger
	if (!low_high) {
		// add rollback operation on releasing the trigger
		if (point_cloud_interaction_settings.enable_history)
			history_ptr->add_rollback_operation();
		
		if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT) {
			static constexpr int cid = 1;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			vec3 ray_origin, ray_direction;
			state_ptr->controller[cid].put_ray(&ray_origin(0), &ray_direction(0));
			teleport_ray(ray_direction, ray_origin, teleport_ray_radius);
			draw_teleport_destination = false;
		}
	}
}

void pointcloud_labeling_tool::on_registration_tool_load_point_cloud() {
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;
	auto* pc_ptr = point_cloud_registration.get_point_cloud();
	if (pc_ptr) {
		point_cloud_registration.ref_scale() = 1.f;
		pc_ptr->read(fn);
		if (!pc_ptr->has_colors()) {
			pc_ptr->create_colors();
			for (int i = 0; i < pc_ptr->get_nr_points(); ++i)
				pc_ptr->clr(i) = rgb8(10, 240, 10);
		}
	}
}

void pointcloud_labeling_tool::on_auto_pilot_load_path()
{
	std::string fn = cgv::gui::file_open_dialog("txt file with path(*.txt;*)", "Path Files:*.*");
	if (fn.empty())
		return;
	std::string data;
	if (!cgv::utils::file::read(fn, data)) {
		std::cerr << "on_auto_pilot_load_path: failed reading data from file: " << fn << '\n';
		return;
	}

	std::istringstream f(data);
	std::string line;

	std::vector<vec3> way_points;

	int line_counter = 0;

	while (!f.eof()) {
		++line_counter;
		std::getline(f, line); 	//read a line
		std::istringstream l(line);
		std::string sym;

		vec3 position;

		bool valid = true;
		bool error = false;
		for (int i = 0; i < 3; ++i) {
			if (!l.eof()) {
				getline(l, sym, ',');
			}
			else {
				valid = false;
				error = true;
				break;
			}
			if (sym.size() > 0)
				position[i] = stod(sym);
			else {
				valid = false;
				if (i > 0)
					error = true;
				break;
			}
		}
		if (valid)
			way_points.push_back(position);
		else if (error)
			std::cerr << "error on line " << line_counter << "\n";
	}

	automated_navigation.path.swap(way_points);
}

void pointcloud_labeling_tool::on_set_reduce_cpu_time_watch_file()
{
	reduce_cpu_time_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "Time log files:*.log;*.txt");
	if (reduce_cpu_time_watch_file_name.empty())
		return;
	if (reduce_cpu_watch_file_ptr) {
		reduce_cpu_watch_file_ptr->close();
	}
	reduce_cpu_watch_file_ptr = std::make_unique<std::ofstream>(reduce_cpu_time_watch_file_name);
	update_views(&reduce_cpu_time_watch_file_name);
}

void pointcloud_labeling_tool::on_set_reduce_gpu_time_watch_file()
{
	reduce_time_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "Time log files:*.log;*.txt");
	if (reduce_time_watch_file_name.empty())
		return;
	if (reduce_watch_file_ptr) {
		reduce_watch_file_ptr->close();
	}
	reduce_watch_file_ptr = std::make_unique<std::ofstream>(reduce_time_watch_file_name);
	update_views(&reduce_time_watch_file_name);
}

void pointcloud_labeling_tool::on_set_labeling_cpu_time_watch_file()
{
	labeling_cpu_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "Time log files:*.log;*.txt");
	if (labeling_cpu_watch_file_name.empty())
		return;
	if (labeling_cpu_watch_file_ptr) {
		labeling_cpu_watch_file_ptr->close();
	}
	labeling_cpu_watch_file_ptr = std::make_unique<std::ofstream>(labeling_cpu_watch_file_name);
	update_views(&labeling_cpu_watch_file_name);
}

void pointcloud_labeling_tool::on_set_draw_time_watch_file()
{
	//reused for draw and reduce
	draw_time_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "Time log files:*.log;*.txt");
	if (draw_time_watch_file_name.empty())
		return;
	if (draw_watch_file_ptr) {
		draw_watch_file_ptr->close();
	}
	draw_watch_file_ptr = std::make_unique<std::ofstream>(draw_time_watch_file_name);
	update_views(&draw_time_watch_file_name);
}

void pointcloud_labeling_tool::on_set_labeling_time_watch_file()
{
	labeling_time_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "Time log files:*.log;*.txt");
	if (labeling_time_watch_file_name.empty())
		return;
	if (labeling_watch_file_ptr){
		labeling_watch_file_ptr->close();
	}
	labeling_watch_file_ptr = std::make_unique<std::ofstream>(labeling_time_watch_file_name);
	update_views(&labeling_time_watch_file_name);
}

void pointcloud_labeling_tool::on_set_fps_watch_file()
{
	fps_watch_file_name = cgv::gui::file_save_dialog("txt file(*.txt;*.log)", "FPS log files:*.log;*.txt");
	if (fps_watch_file_name.empty())
		return;
	if (fps_watch_file_ptr) {
		fps_watch_file_ptr->close();
	}
	fps_watch_file_ptr = std::make_unique<std::ofstream>(fps_watch_file_name);
	update_views(&fps_watch_file_name);
}

void pointcloud_labeling_tool::on_update_gui()
{
	post_recreate_gui();
}

void pointcloud_labeling_tool::timer_event(double t, double dt) {

	if (use_autopilot) {
		const float* pose = vr_view_ptr->get_current_vr_state()->hmd.pose;
		vec3 hmd_position = vec3(pose[9], pose[10], pose[11]);

		if (navigation_is_controller_path) {
			const float* controller_position_ptr = vr_view_ptr->get_current_vr_state()->controller[navigation_selected_controller].pose + 9;
			vec3 controller_tip_position = vec3(controller_position_ptr[0], controller_position_ptr[1], controller_position_ptr[2]) + curr_offset_rhand;
			vec3 new_controller_position = automated_navigation.next_position(controller_tip_position, dt);
			//vec3 hmd_to_controller = controller_position - hmd_position;
			//vec3 new_hmd_position = new_controller_position - hmd_to_controller;
			vec3 tracking_origin = vr_view_ptr->get_tracking_origin() + new_controller_position - controller_tip_position;
			vr_view_ptr->set_tracking_origin(tracking_origin);
		}
		else {
			vec3 new_hmd_position = automated_navigation.next_position(hmd_position, dt);

			vec3 tracking_origin = vr_view_ptr->get_tracking_origin() + new_hmd_position - hmd_position;
			vr_view_ptr->set_tracking_origin(tracking_origin);
		}

	}
}

/// a quick test that enables to label the point cloud without a VR device 
void pointcloud_labeling_tool::test_labeling_of_points() {
	point_server_ptr->ref_interaction_settings() = point_cloud_interaction_settings;
	point_server_ptr->label_points_in_sphere((int)point_label_group::SELECTED_BIT, (int)point_label_group::GROUP_MASK, 0, vec3(0), radius_for_test_labeling, point_label_operation::REPLACE);
}

void pointcloud_labeling_tool::test_moving_points()
{
	static bool build_shader = true;
	static shader_program prog;

	auto& chunked_points = point_server_ptr->ref_chunks();

	if (chunked_points.num_chunks() == 0) {
		std::cerr << "test_moving_points: there are no points to move!\n";
		return;
	}

	vec3 point_move = vec3(preparation_settings.chunk_cube_size*0.34);

	context& ctx = *this->get_context();

	if (build_shader) {
		prog.create(ctx);
		prog.attach_file(ctx, "point_cloud_chunk.glsl", cgv::render::ST_COMPUTE);
		shaderCheckError(prog, "point_cloud_chunk.glsl");
		prog.attach_file(ctx, "point_moving_test.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(prog, "point_moving_test.glcs");

		prog.link(ctx);
		build_shader = false;
	}

	auto& swapping_chunk = chunked_points.swapping_chunk();
	swapping_chunk.resize_reserved(chunked_points.num_points());
	swapping_chunk.upload_to_buffers();
	
	constexpr int chunk_meta_pos = 29;
	constexpr int chunk_b_meta_pos = 30, chunk_b_points_pos = 10, chunk_b_index_pos = 11;

	//set uniforms bind buffers
	mat4 float_model_matrix = concat_mat;
	mat4 inv_float_model_matrix = inv(concat_mat);
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, input_buffer);
	prog.set_uniform(ctx, "model_transform", float_model_matrix, true);
	prog.set_uniform(ctx, "inv_model_transform", inv_float_model_matrix, true);
	prog.set_uniform(ctx, "move", point_move);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, label_shader_manager::labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	//bind chunk b buffers
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_index_pos, swapping_chunk.id_buffer());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_points_pos, swapping_chunk.point_buffer());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_meta_pos, swapping_chunk.meta_buffer());
	
	swapping_chunk.device_buffers_changed() = true;
	
	prog.enable(ctx);

	for (auto& index_chunk_pair : chunked_points) {
		auto& ch = *index_chunk_pair.second;

		if (ch.size() > 0) {
			auto num_points_in_chunk = ch.size();
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, label_shader_manager::point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, label_shader_manager::points_pos, ch.point_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_meta_pos, ch.meta_buffer());
			prog.set_uniform(ctx, "batch_size", (GLint)num_points_in_chunk, true);
			prog.set_uniform(ctx, "batch_offset", (GLint)0, true);
			glDispatchCompute((num_points_in_chunk / 128) + 1, 1, 1);

			ch.device_buffers_changed() = true;
		}
	}
	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);
	prog.disable(ctx);

	chunked_points.distribute_global_chunk();
}

void pointcloud_labeling_tool::test_fill_clipboard()
{
	point_cloud tmp;
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	tmp.read(fn);
	tmp.create_colors();

	std::vector<indexed_point> i_lod_points(tmp.get_nr_points());
	int nr_points = i_lod_points.size();
	std::vector<GLint> labels(nr_points);

	for (int i = 0; i < nr_points; ++i) {
		indexed_point p;
		p.position() = tmp.pnt(i);
		p.color() = tmp.clr(i);
		p.index = i;
		i_lod_points[i] = p;
		if (tmp.has_labels())
			labels[i] = tmp.label(i);
		else
			labels[i] = make_label(0, point_label_group::VISIBLE);
	}

	auto& lod_generator = cgv::pointcloud::octree::ref_octree_lod_generator<pct::indexed_point>();
	i_lod_points = lod_generator.generate_lods(i_lod_points);

	std::vector<LODPoint> lod_points(i_lod_points.size());
	std::vector<GLuint> point_ids(i_lod_points.size());
	int nr_lod_points = lod_points.size();
	for (int i = 0; i < nr_points; ++i) {
		lod_points[i] = i_lod_points[i];
		point_ids[i] = i_lod_points[i].index;
	}
	for (int i = 0; i < clipboard_ptr->get_max_size(); ++i) {
		copy_points_to_clipboard(lod_points, point_ids, labels.data());
	}
}

void pointcloud_labeling_tool::push_points(cgv::render::context& ctx, const selection_shape& shape) {
	// define buffer space for chunk a
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4, chunk_meta_pos = 29;
	// define buffer space for chunk b
	constexpr int chunk_b_meta_pos = 30, chunk_b_points_pos = 10, chunk_b_index_pos = 11;
	
	if (label_attribute_id == -1)
		return;

	auto& label_shaders = ref_label_shader_manager(ctx);
	auto& push_prog = label_shaders.get_pushing_tool(shape);
	
	std::vector<std::pair<const ivec3, chunk<LODPoint>*>> chunks;
	
	auto& chunked_points = point_server_ptr->ref_chunks();

	switch (shape) {
	case selection_shape::SS_SPHERE: {
		float radius = sphere_style_rhand.radius;
		float scaled_radius = radius / (point_server_ptr->ref_point_cloud_scale());
		dmat4& concat_trans = concat_mat;
		vec3 position = cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(radius);

		// find affected chunks
		vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
		vec3 position_in_model_space3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
		chunked_points.intersect_sphere(position_in_model_space3, scaled_radius, chunks);
		// set specific uniforms
		vec4 selection_enclosing_sphere = vec4(position.x(), position.y(), position.z(), radius);
		push_prog.set_uniform(ctx, "selection_enclosing_sphere", selection_enclosing_sphere);
		break;
	}
	case selection_shape::SS_CUBOID: {
		dmat4& concat_trans = concat_mat;
		box3 box = cube_rhand;

		vec3 position = cgv::math::pose_position(controller_poses[point_selection_hand]) + shape_offset(box.get_extent().z()*0.5f);
		vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
		vec3 position_in_model_space3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
		
		float scaled_radius = length(box.get_extent()) / (point_server_ptr->ref_point_cloud_scale());

		push_prog.set_uniform(ctx, "aabb_min_p", box.get_min_pnt());
		push_prog.set_uniform(ctx, "aabb_max_p", box.get_max_pnt());
		push_prog.set_uniform(ctx, "selection_box_translation", position.lift(),true);
		push_prog.set_uniform(ctx, "selection_box_rotation", cube_orientation_rhand,true);
		chunked_points.intersect_sphere(position_in_model_space3, scaled_radius, chunks); //use sphere intersection for now
		break;
	}
	default:
		std::cerr << "called push_points(..) with an unsupported shape parameter\n";
		return;
	}
	//set uniforms bind buffers
	mat4 float_model_matrix = concat_mat;
	mat4 inv_float_model_matrix = inv(concat_mat);
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, input_buffer);
	push_prog.set_uniform(ctx,"model_transform", float_model_matrix, true);
	push_prog.set_uniform(ctx,"inv_model_transform", inv_float_model_matrix, true);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());
	
	//bind chunk b buffers
	auto& swapping_chunk = chunked_points.swapping_chunk();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_points_pos, swapping_chunk.point_buffer());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_meta_pos, swapping_chunk.meta_buffer());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_b_index_pos, swapping_chunk.id_buffer());
	
	//mark buffer as changed
	swapping_chunk.device_buffers_changed() = true;

	//run push shader
	push_prog.enable(ctx);

	//run computation on non empty chunks
	for (auto index_chunk_pair : chunks) {
		auto& ch = *index_chunk_pair.second;
		if (ch.size() > 0) {
			auto num_points_in_chunk = ch.size();
			//bind chunk buffers
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, chunk_meta_pos, ch.meta_buffer());
			push_prog.set_uniform(ctx, "batch_size", (GLint)num_points_in_chunk, true);
			push_prog.set_uniform(ctx, "batch_offset", (GLint) 0, true);
			glDispatchCompute((num_points_in_chunk / 128) + 1, 1, 1);

			//mark changed chunks
			ch.device_buffers_changed() = true;
		}
	}
	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);
	push_prog.disable(ctx);
	//relocated points that crossed chunk borders
	chunked_points.distribute_global_chunk();
}

///
void pointcloud_labeling_tool::on_rollback_cb() {
	rollback_last_operation(*get_context());
}
///
void pointcloud_labeling_tool::stream_help(std::ostream & os)
{
}
void pointcloud_labeling_tool::update_help_labels(const InteractionMode mode) {
	//hide both labels
	for (int i = 0; i < 2; ++i) {
		if (li_help[i] != -1)
			text_labels.hide_label(li_help[i]);
	}
	
	//set labels for the given interaction mode
	//li_help[help_label_ci] = tool_help_label_id[mode][1];
	li_help[0] = tool_help_label_id[mode][0];
	li_help[1] = tool_help_label_id[mode][1];
	allways_show_controller_label[0] = tool_help_label_allways_drawn[mode][0];
	allways_show_controller_label[1] = tool_help_label_allways_drawn[mode][1];
}
void pointcloud_labeling_tool::update_controller_labels()
{
	auto default_settings = [this](int ci, int p) {
		if (controller_label_variants[ci][interaction_mode][p].size() > 0) {
			this->controller_labels[ci].set_active((controller_label_placement)p, controller_label_variants[ci][interaction_mode][p][0]);
		}
		else {
			this->controller_labels[ci].set_active((controller_label_placement)p, -1);
		}
	};

	//right controller's button labels
	for (int ci = 0; ci < 2; ++ci) {
		for (int p = 0; p < CLP_NUM_LABEL_PLACEMENTS; ++p) {
			switch (interaction_mode) {
			case InteractionMode::LABELING: {
				if (p == CLP_GRIP && ci == 1) {
					/*if (point_editing_tool == pallete_tool::PT_PASTE) {
						controller_labels[ci].set_active((controller_label_placement)p, controller_label_variants[ci][interaction_mode][p][paste_mode_controller_label_variant_index]);
					}
					else {
						controller_labels[ci].set_active((controller_label_placement)p, controller_label_variants[ci][interaction_mode][p][point_selection_shape]);
					}
					break;*/
				}
				else if (p == CLP_TRACKPAD_RIGHT && ci == 1) {
					if (point_editing_tool == PT_SELECTION) {
						controller_labels[ci].set_active((controller_label_placement)p, box_is_constraint_label_variant);
						if (box_shaped_selection_is_constraint) {
							controller_labels[ci].set_label_color((controller_label_placement)p, box_is_constraint_label_variant, rgba(0.23, 0.67, 0.0, 0.0));
						}
						else {
							controller_labels[ci].set_label_color((controller_label_placement)p, box_is_constraint_label_variant, controller_label_color);
						}
						break;
					}
					if (point_editing_tool == pallete_tool::PT_PASTE)
					{
						controller_labels[ci].set_active((controller_label_placement)p, paste_is_shown);
					}
					else
					{
						default_settings(ci, p);
					}
				}
				else if (p == CLP_TRACKPAD_LEFT && ci == 1) {
					if (point_editing_tool == PT_SELECTION) {
						controller_labels[ci].set_active((controller_label_placement)p, clear_box_label_variant);
						break;
					}
				}
				else if (p == CLP_MENU_BUTTON && ci == 1) {
					if (point_editing_tool == PT_SELECTION) {
						controller_labels[ci].set_active((controller_label_placement)p, box_is_active_labeling_variant);
						break;
					}
				}
				//fall through
				default_settings(ci, p);
				break;
			}
			case InteractionMode::CONFIG: {
				if (p == CLP_MENU_BUTTON && ci == 1) {
					if (spacing_tool_enabled) {
						controller_labels[ci].set_active((controller_label_placement)p, apply_spacing_label_variant);
						break;
					}
				}
				default_settings(ci, p);
				break;
			}
			}
		}
	}
}

void pointcloud_labeling_tool::update_palette()
{
	//update color map
	palette.set_colors(PALETTE_COLOR_MAPPING);

	if (color_map_buffer) {
		glNamedBufferData(color_map_buffer, (PALETTE_COLOR_MAPPING.size() - 1) * sizeof(vec4), PALETTE_COLOR_MAPPING.data() + 1, GL_STATIC_READ);
		color_map_buffer_size = (GLuint)(PALETTE_COLOR_MAPPING.size() - 1);
	}
}

vec3 pointcloud_labeling_tool::shape_offset(float radius) const
{
	return curr_offset_rhand + cgv::math::normalize(curr_offset_rhand) * radius;
}

void pointcloud_labeling_tool::show_clipboard_in_palette(pct::point_cloud_clipboard& pcc)
{
	
}


void pointcloud_labeling_tool::set_palette_label_text_map(const std::unordered_map<unsigned int, std::string>& map)
{
	static constexpr int labels_begin = 1; //points to "clear label"
	for (auto& label_name_pair : map) {
		palette.set_label_text(label_name_pair.first + labels_begin, label_name_pair.second);
	}
}

void pointcloud_labeling_tool::colorize_with_height() {
	
	auto& chunked_points = point_server_ptr->ref_chunks();

	chunked_points.download_buffers();

	vec3 pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	for (auto& location_chunk_pair :  chunked_points) {
		auto& ch = *location_chunk_pair.second;
		for (unsigned int i = 0; i < ch.size(); ++i) {
			auto& pnt = ch.point_at(i).position();

			pmin.x() = std::min(pnt.x(), pmin.x());
			pmin.y() = std::min(pnt.y(), pmin.y());
			pmin.z() = std::min(pnt.z(), pmin.z());

			pmax.x() = std::max(pnt.x(), pmax.x());
			pmax.y() = std::max(pnt.y(), pmax.y());
			pmax.z() = std::max(pnt.z(), pmax.z());
		}
	}
	
	vec3 ext = (pmax - pmin);
	int minimal_ext_idx = 0;
	//if (ext.x() < ext.y() && ext.x() < ext.z())
	//	minimal_ext_idx = 0;
	if (ext.y() < ext.x() && ext.y() < ext.z())
		minimal_ext_idx = 1;
	if (ext.z() < ext.y() && ext.z() < ext.x())
		minimal_ext_idx = 2;

	for (auto& location_chunk_pair : chunked_points) {
		auto& ch = *location_chunk_pair.second;
		for (unsigned int i = 0; i < ch.size(); ++i) {
			auto& pnt = ch.point_at(i);
			float factor;
			switch (minimal_ext_idx) {
			case 0:
				factor = (pnt.position().x() - pmin.x()) / ext.x();
				break;
			case 1:
				factor = (pnt.position().y() - pmin.y()) / ext.y();
				break;
			case 2:
				factor = (pnt.position().z() - pmin.z()) / ext.z();
				break;
			}
			pnt.color() = rgb8((int)255 * factor, 0, (int)255 * (1 - factor));
		}
	}
	//push data to buffers
	chunked_points.upload_to_buffers();
}
///
void pointcloud_labeling_tool::print_point_cloud_info() {
	auto& chunked_points = point_server_ptr->ref_chunks();

	std::cout << "point loaded: " << source_point_cloud.get_nr_points() << std::endl;
	std::cout << "chunks used: " << chunked_points.num_chunks() << std::endl;
}
/// fills chunked_points with data generated from the source point cloud (source_pc)
void pointcloud_labeling_tool::prepare_point_cloud() noexcept
{ 
	point_server_ptr->use_point_cloud(source_point_cloud, preparation_settings);
	//make a lod to color lookup table
	auto col_lut = point_server_ptr->make_lod_to_color_lut();
	glNamedBufferData(lod_to_color_lut_buffer, sizeof(rgba) * col_lut.size(), col_lut.data(), GL_STATIC_READ);
	//get references to some point attributes
	auto& chunked_points = point_server_ptr->ref_chunks();
	label_attribute_id = chunked_points.find_attribute_id_by_name("label");
	normal_attribute_id = chunked_points.find_attribute_id_by_name("normal");
}

///
void pointcloud_labeling_tool::on_point_cloud_fit_table()
{
	auto& chunked_points = point_server_ptr->ref_chunks();

	chunked_points.download_buffers();
	
	rgb color(1.0, 0.0, 0.0);
	//find weighted center
	vec3 centroid(0.f), pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	dmat4 model_transform = point_server_ptr->get_point_cloud_model_transform();
	float scale = 1.f;

	//todo may use points stored in point_cloud_server for centroid computation, source_pc may be out of sync
	for (int i = 0; i < source_point_cloud.get_nr_points(); ++i) {
		vec3 pnt = model_transform * dvec4(source_point_cloud.pnt(i).lift());
		centroid += pnt;
		pmin.x() = std::min(pnt.x(), pmin.x());
		pmin.y() = std::min(pnt.y(), pmin.y());
		pmin.z() = std::min(pnt.z(), pmin.z());
		pmax.x() = std::max(pnt.x(), pmax.x());
		pmax.y() = std::max(pnt.y(), pmax.y());
		pmax.z() = std::max(pnt.z(), pmax.z());
	}
	centroid /= source_point_cloud.get_nr_points();
	vec3 ext = (pmax - pmin);

	scale = (1.0 / static_cast<double>(*std::max_element(ext.begin(), ext.end())));

	//source_pc.ref_point_cloud_scale() *= scale;
	rescale_point_cloud(point_server_ptr->ref_point_cloud_scale() * scale);

	mat4 new_model_transform = cgv::math::translate4(point_server_ptr->ref_point_cloud_position())
		* cgv::math::rotate4<float>(point_server_ptr->ref_point_cloud_rotation())
		* cgv::math::scale4(point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale());

	vec3 p_bottom(centroid.x(), pmin.y(), centroid.z());
	p_bottom = new_model_transform * mat4(cgv::math::inv(model_transform)) * p_bottom.lift();
	
	vec3 move(0);

	move.y() = table_height;
	move -= p_bottom;
	point_server_ptr->ref_point_cloud_position() += move;
}

void pointcloud_labeling_tool::on_add_point_cloud_to_clipboard()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	std::unique_ptr<point_cloud_record> pcr_ptr = std::make_unique<point_cloud_record>();
	pcr_ptr->points = std::make_unique<point_cloud>();
	//read new pointcloud	
	pcr_ptr->points->read(fn);
	if (file_contains_label_groups && pcr_ptr->points->has_labels()) {
		size_t num_points = pcr_ptr->points->get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			pcr_ptr->points->label(i) = (pcr_ptr->points->label(i) >> 16);
		}
	}

	pcr_ptr->compute_centroid();
	pcr_ptr->name = cgv::utils::file::get_file_name(fn);

	clipboard_ptr->move_from(pcr_ptr);
	post_redraw();
}

void pointcloud_labeling_tool::on_load_point_cloud_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//clear parameters 
	on_clear_point_cloud_cb();
	//read new pointcloud	
	source_point_cloud.read(fn);
	std::cout << "source_pc.read done." << std::endl;
	//this is for loading the old color format
	if (file_contains_label_groups && source_point_cloud.has_labels()) {
		size_t num_points = source_point_cloud.get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			source_point_cloud.label(i) = (source_point_cloud.label(i) >> 16);
		}
	}
	// the first mode is Teleport
	update_interaction_mode((InteractionMode)interaction_mode);

	//the octree_lod_generator expects the input points to be an array of structs, so we need to reshape the data
	//prepare_point_cloud() does this, the labels from source_pc will end up in a point_labels attribute and the points in chunked_points
	prepare_point_cloud();
	std::cout << "prepare_point_cloud done." << std::endl;

	if (pointcloud_fit_table) {
		on_point_cloud_fit_table();
		std::cout << "automatic_scale_to_fit_table done." << std::endl;	
	}
	source_name = cgv::utils::file::get_file_name(fn);
	post_redraw();

	std::cout << "loaded pointcloud " << fn << " with " << source_point_cloud.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_load_comparison_point_cloud_1_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	pc_1.read(fn);
	std::cout << "source_pc.read done." << std::endl;
	if (file_contains_label_groups && pc_1.has_labels()) {
		size_t num_points = pc_1.get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			pc_1.label(i) = (pc_1.label(i) >> 16);
		}
	}

	std::cout << "loaded pointcloud " << fn << " with " << pc_1.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_load_comparison_point_cloud_2_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	pc_2.read(fn);
	std::cout << "source_pc.read done." << std::endl;
	if (file_contains_label_groups && pc_2.has_labels()) {
		size_t num_points = pc_2.get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			pc_2.label(i) = (pc_2.label(i) >> 16);
		}
	}

	std::cout << "loaded pointcloud " << fn << " with " << pc_2.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_load_annotated_point_cloud_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	pc_annotated.read(fn);
	std::cout << "source_pc.read done." << std::endl;
	if (file_contains_label_groups && pc_annotated.has_labels()) {
		size_t num_points = pc_annotated.get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			pc_annotated.label(i) = (pc_annotated.label(i) >> 16);
		}
	}

	std::cout << "loaded pointcloud " << fn << " with " << pc_annotated.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_compare_annotated_data()
{
	for (int i = 0; i < source_point_cloud.get_nr_points(); ++i)
	{
		for (int j = 0; j < pc_annotated.get_nr_points(); ++j)
		{
			if (source_point_cloud.pnt(i) == pc_annotated.pnt(j))
			{

			}
		}
	}
}

void pointcloud_labeling_tool::on_load_comparison_point_cloud_cb()
{
	size_t num_points = pc_2.get_nr_points();
	if (pc_1.get_nr_points() != num_points || num_points == 0) {
		std::cout << "Error: Point clouds do not match in size or are empty.\n";
		return;
	}

	// Maps to store label counts
	std::unordered_map<int, int> intersection, union_pc1, union_pc2;

	for (size_t i = 0; i < num_points; ++i) {
		int label1 = pc_1.label(i);
		int label2 = pc_2.label(i);

		if (label1 == label2) {
			intersection[label1]++;
		}

		union_pc1[label1]++;
		union_pc2[label2]++;
	}

	// Calculate IoU for each label and overall IoU
	std::unordered_map<int, float> iou_per_label;
	int total_intersection = 0, total_union = 0;

	// Calculate IoU for each label
	for (const auto& p : union_pc1) {
		int label = p.first;
		int union_count = p.second + union_pc2[label] - intersection[label];
		int intersect_count = intersection[label];

		iou_per_label[label] = static_cast<float>(intersect_count) / union_count;

		// Accumulate totals for overall IoU
		total_intersection += intersect_count;
		total_union += union_count;
	}

	// Include labels that only appear in pc_2
	for (const auto& p : union_pc2) {
		if (union_pc1.find(p.first) == union_pc1.end()) {
			total_union += p.second;
		}
	}

	// Calculate overall IoU
	float overall_iou = static_cast<float>(total_intersection) / total_union;

	// Output IoU for each label
	for (const auto& p : iou_per_label) {
		std::cout << "Label " << p.first << " IoU: " << p.second * 100.0f << "%\n";
	}

	// Output overall IoU
	std::cout << "Overall IoU: " << overall_iou * 100.0f << "%\n";
}

void pointcloud_labeling_tool::on_clear_comparison_point_cloud_cb()
{
	pc_1.clear();
	pc_2.clear();
	history_ptr->remove_all();
	label_attribute_id = -1;
	normal_attribute_id = -1;
	prepare_point_cloud();
	overview_mode = false;
	post_redraw();
}

void pointcloud_labeling_tool::on_load_ori_pc_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	pc_gt.read(fn);
	std::cout << "source_pc.read done." << std::endl;
	if (file_contains_label_groups && pc_gt.has_labels()) {
		size_t num_points = pc_gt.get_nr_points();
		for (size_t i = 0; i < num_points; ++i) {
			pc_gt.label(i) = (pc_gt.label(i) >> 16);
		}
	}

	std::cout << "loaded pointcloud " << fn << " with " << pc_gt.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_load_anno_pcs_cb() 
{
	std::string directory_str = cgv::gui::directory_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	std::vector<string> filelists;
	getfilelist(directory_str, filelists);
	point_cloud temp;
	for (int i = 0; i < filelists.size(); ++i)
	{
		temp.clear();
		std::cout << filelists.at(i) << std::endl;
		std::string filename = directory_str + "/" + filelists.at(i) + ".txt";
		temp.read(filename);
		pc_list.push_back(temp);
		std::cout << "number: " << pc_list.at(i).get_nr_points() << std::endl;
	}
	/*std::unordered_map<std::string, point_cloud> annotatedClouds;

	for (const auto& filename : filenames) {
		annotatedClouds[filename] = loadPointCloud(filename);
	}

	return annotatedClouds;*/
}

void pointcloud_labeling_tool::getdirectorylist(const std::string& dirpath, std::vector<std::string>& directorylist)
{
	std::string path = dirpath + "/*.*";
	_finddata_t file;
	long handle;
	if ((handle = _findfirst(path.c_str(), &file)) == -1) {
		std::cout << "Not Found File!" << std::endl;
	}
	else {
		_findnext(handle, &file); //.

		while (_findnext(handle, &file) == 0) {
			std::string directoryname = file.name;
			directorylist.push_back(directoryname);
		}
	}
	_findclose(handle);
}

void pointcloud_labeling_tool::getfilelist(const std::string& dirpath, std::vector<std::string>& filelist)
{
	std::string path = dirpath + "/*.*";
	_finddata_t file;
	intptr_t handle;
	if ((handle = _findfirst(path.c_str(), &file)) == -1) {
		std::cout << "Not Found File!" << std::endl;
	}
	else {
		_findnext(handle, &file); //.

		while (_findnext(handle, &file) == 0) {
			std::string filename = file.name;
			if (filename.size() > 4 && filename.substr(filename.length() - 4, 4) == ".txt") {
				filelist.push_back(filename.substr(0, filename.length() - 4));
			}
		}
	}
	_findclose(handle);
}

void pointcloud_labeling_tool::on_load_CAD_cb() {
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	M.read(fn);

	std::cout << "loaded CAD " << fn << " with " << pc_2.get_nr_points() << " points!\n";
}

void pointcloud_labeling_tool::on_load_scannet_gt_cb() {
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;
}

void pointcloud_labeling_tool::draw_surface(cgv::render::context& ctx, bool opaque_part)
{
	if (have_new_mesh) {
		// auto-compute mesh normals if not available
		if (!M.has_normals())
			M.compute_vertex_normals();
		// [re-]compute mesh render info
		mesh_info.destruct(ctx);
		mesh_info.construct(ctx, M);
		// bind mesh attributes to standard surface shader program
		mesh_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
		mesh_info.bind_wireframe(ctx, ref_cone_renderer(ctx).ref_prog(), true);
		// ensure that materials are presented in gui
		post_recreate_gui();
		have_new_mesh = false;
	}
	// remember current culling setting
	GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
	GLint cull_face;
	glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

	// ensure that opengl culling is identical to shader program based culling
	if (cull_mode > 0) {
		glEnable(GL_CULL_FACE);
		glCullFace(cull_mode == CM_BACKFACE ? GL_BACK : GL_FRONT);
	}
	else
		glDisable(GL_CULL_FACE);

	// choose a shader program and configure it based on current settings
	shader_program& prog = ctx.ref_surface_shader_program(true);
	prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
	prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
	prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
	// set default surface color for color mapping which only affects 
	// rendering if mesh does not have per vertex colors and color_mapping is on
	//prog.set_attribute(ctx, prog.get_color_index(), surface_color);
	ctx.set_color(surface_color);
	// render the mesh from the vertex buffers with selected program
	mesh_info.draw_all(ctx, false, false);

	// recover opengl culling mode
	if (is_culling)
		glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);
	glCullFace(cull_face);
}

void pointcloud_labeling_tool::draw_mesh() {
	have_new_mesh = true;
	show_surface = true;
	show_vertices = true;
	show_wireframe = true;
}

void pointcloud_labeling_tool::adapt_parameters(const int current_framerate) {
	float temp_ps = source_point_cloud.ref_render_style().pointSize;
	float temp_clod = source_point_cloud.ref_render_style().CLOD;
	if (current_framerate < desired_fps) {
		// If the current framerate is less than the desired framerate,
		// decrease the sizeMultiplier to render points smaller, which should be less intensive
		temp_ps *= (1.0f - adaptation_rate);
		// Optionally, adjust CLOD to decrease level of detail dynamically
		temp_clod += adaptation_rate;
	}
	else if (current_framerate > stable_fps) {
		// If performance allows, we can increase quality by decreasing targetSpacing
		// and increasing sizeMultiplier, but make sure not to exceed desired framerate
		temp_ps *= (1.0f + adaptation_rate);
		temp_clod -= adaptation_rate;
	}
	// Clamp sizeMultiplier and targetSpacing to prevent over-scaling
	source_point_cloud.ref_render_style().pointSize = cgv::math::clamp(temp_ps, 0.01f, 1.0f);
	source_point_cloud.ref_render_style().CLOD = cgv::math::clamp(temp_clod, 1.0f, 5.0f);
	update_member(&source_point_cloud.ref_render_style().pointSize);
	update_member(&source_point_cloud.ref_render_style().CLOD);
}

vec3 pointcloud_labeling_tool::bezier_point(double t, const vec3 p0, const vec3 p1, const vec3 p2, const vec3 p3)
{
	double u = 1 - t;
	double tt = t * t;
	double uu = u * u;
	double uuu = uu * u;
	double ttt = tt * t;

	vec3 point = p0 * uuu;    // First term
	point = point + p1 * (3 * uu * t); // Second term
	point = point + p2 * (3 * u * tt); // Third term
	point = point + p3 * ttt;    // Fourth term
	return point;
}

// Function to interpolate between two points
vec3 pointcloud_labeling_tool::interpolate(const vec3& p0, const vec3& p1, float t) {
	return {
		(1 - t) * p0.x() + t * p1.x(),
		(1 - t) * p0.y() + t * p1.y(),
		(1 - t) * p0.z() + t * p1.z()
	};
}

// Generate Bezier surface points
point_cloud pointcloud_labeling_tool::generateBezierSurface(const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3, int numPoints) {
	//std::vector<vec3> surfacePoints;
	point_cloud surfacePTS;
	for (int i = 0; i < numPoints; ++i) {
		float u = i / (float)(numPoints - 1);
		for (int j = 0; j < numPoints; ++j) {
			float v = j / (float)(numPoints - 1);

			// Interpolate the points
			vec3 p01 = interpolate(p0, p1, u);
			vec3 p32 = interpolate(p3, p2, u);
			point_cloud_types::Pnt p = interpolate(p01, p32, v);
			rgb c = rgb(1.0, 0.0, 0.0);
			surfacePTS.add_point(p, c);
		}
	}
	return surfacePTS;
}

void pointcloud_labeling_tool::load_labels_from(const std::string& fn) {
	std::fstream file = std::fstream(fn, std::fstream::in);
	if (!file.good()) {
		std::cerr << "can't open file " << fn << std::endl;
		return;
	}
	//load file name, and output label name and color
	std::unordered_map<unsigned int, std::string> names;
	std::unordered_map<unsigned int, cgv::media::color<float>> colors;
	parse_label_mappings(file, &names, &colors);
	set_palette_label_text_map(names);
	set_palette_label_color_map(colors);
}

void pointcloud_labeling_tool::on_load_palette_labels()
{
	std::string fn = cgv::gui::file_open_dialog("label mappings(*.palette;*.txt)", "*.palette;*.txt;");
	if (fn.empty())
		return;
	load_labels_from(fn);
}
///
void pointcloud_labeling_tool::on_load_s3d_labels_cb()
{
	std::string fn = cgv::gui::file_open_dialog("label mappings(*.labels)", "*.labels;");
	if (fn.empty())
		return;
	load_label_nr_from(fn);
}

void pointcloud_labeling_tool::on_store_s3d_labels_cb() {
	std::string fn = cgv::gui::file_save_dialog("label mappings(*.labels)", "*.labels;");
	if (fn.empty())
		return;
	store_label_nr_to(fn);
	std::cout << "wrote labels to " << fn << std::endl;
}

void pointcloud_labeling_tool::on_generate_large_pc_cb()
{
	point_cloud large_pc;
	large_pc.create_colors();
	//large_pc.resize(1000000);
	point_cloud::Pnt point;
	point_cloud::Clr color;
	for (int i = 0; i< 10000000; ++i)
	{
		point.x() = 256 * rand() / (RAND_MAX + 1.0f);
		point.y() = 256 * rand() / (RAND_MAX + 1.0f);
		point.z() = 256 * rand() / (RAND_MAX + 1.0f);
		color = point_cloud::Clr(256 * rand() / (RAND_MAX + 1.0f), 256 * rand() / (RAND_MAX + 1.0f), 256 * rand() / (RAND_MAX + 1.0f));
		large_pc.add_point(point);
		large_pc.clr(i) = color;
	}
	std::string fn = cgv::gui::file_save_dialog("point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	large_pc.write(fn);
}
///
void pointcloud_labeling_tool::parallel_saving() {
	std::string fn = cgv::gui::file_save_dialog("point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;
	source_point_cloud.write(fn);
	std::cout << "saved!" << std::endl;
}
///
void pointcloud_labeling_tool::on_save_point_cloud_cb()
{
	auto& chunked_points = point_server_ptr->ref_chunks();

	if (chunked_points.num_points() == 0) {
		std::cout << "save point cloud: nothing to save\n";
		return;
	}
	//copy label buffer from gpu memory to point_labels
	sync_data();
	//copy data from chunks to source_pc for the case anything was changed, also applies point deletions
	copy_chunks_to_point_cloud(source_point_cloud);
	if (writing_thread && writing_thread->joinable()) {
		writing_thread->join();
		delete writing_thread;
	}
	writing_thread = new std::thread(&pointcloud_labeling_tool::parallel_saving, this);
}
///
void pointcloud_labeling_tool::on_clear_point_cloud_cb()
{
	source_point_cloud.clear();
	history_ptr->remove_all();
	label_attribute_id = -1;
	normal_attribute_id = -1;
	prepare_point_cloud();
	overview_mode = false;
	post_redraw();
}
///
void pointcloud_labeling_tool::on_point_cloud_style_cb()
{
	post_redraw();
}
///
void pointcloud_labeling_tool::on_lod_mode_change()
{
	
}
///
void pointcloud_labeling_tool::on_move_to_center() {
	dvec4 centroid = dvec4(0, 0, 0, 1);
	for (int i = 0; i < source_point_cloud.get_nr_points(); ++i) {
		const vec3& pnt = source_point_cloud.pnt(i);
		centroid += dvec4(pnt.x(), pnt.y(), pnt.z(), 0.0);
	}
	centroid = centroid / source_point_cloud.get_nr_points();

	source_point_cloud.ref_point_cloud_position() = vec3(-centroid.x(),-centroid.y(),-centroid.z());
}

void pointcloud_labeling_tool::on_toggle_show_labeled_points_only()
{
	if (visible_point_groups & 1) {
		visible_point_groups = ~(~0u << 32) & (~0u << 16);
	}
	else {
		visible_point_groups = (GLint)point_label_group::VISIBLE;
	}
	post_recreate_gui();
}

void pointcloud_labeling_tool::on_clear_all_labels()
{
	if (point_server_ptr)
		point_server_ptr->label_all_points(make_label(0, point_label_group::VISIBLE), (GLint)point_label_group::VISIBLE, 0);
}


void pointcloud_labeling_tool::fetch_from_rgbd(bool x) {
	if (x) {
		auto* point_cloud_provider = point_cloud_providers.size() > 0 ? point_cloud_providers[0] : nullptr;
		point_cloud_registration.set_provider(point_cloud_provider);
	}
	else {
		point_cloud_registration.set_provider(nullptr);
	}
}

void pointcloud_labeling_tool::on_rechunk() {
	auto& chunked_points = point_server_ptr->ref_chunks();

	if (chunks_disabled) {
		//rechunk reenables usage of chunks
		set("skip_frustum_culling", false);
		chunks_disabled = false;
		point_cloud_interaction_settings.use_chunks = true;
		update_member(&skip_frustum_culling);
	}
	if (preparation_settings.auto_chunk_cube_size) {
		preparation_settings.chunk_cube_size = point_server_ptr->compute_chunk_size(source_point_cloud, preparation_settings);
		update_member(&preparation_settings.chunk_cube_size);
	}

	point_server_ptr->rechunk_point_cloud(preparation_settings.chunk_cube_size);
}

void pointcloud_labeling_tool::on_make_infinite_chunk() {
	auto& chunked_points = point_server_ptr->ref_chunks();

	//effectivly disables chunks, thus disable optimizations that require chunks
	set("skip_frustum_culling", true);
	chunks_disabled = true;
	point_cloud_interaction_settings.use_chunks = false;
	chunked_points.download_buffers();
	chunked_points.disable_chunks();
}

void pointcloud_labeling_tool::load_label_names_from(const std::string& fn)
{
	std::fstream file = std::fstream(fn, std::fstream::in);
	if (!file.good()) {
		std::cerr << "can't open file " << fn << std::endl;
		return;
	}
	auto name_mapping = parse_label_mappings(file);
	set_palette_label_text_map(name_mapping);
}
///
void pointcloud_labeling_tool::load_label_nr_from(const std::string& fn)
{
	auto& chunked_points = point_server_ptr->ref_chunks();

	//synchronize point data
	sync_data();

	//open file
	std::fstream file = std::fstream(fn, std::fstream::in);
	if (!file.good()) {
		std::cerr << "can't open file " << fn << std::endl;
		return;
	}
	
	//read line by line
	std::string line;
	int i = 0;
	auto& point_labels = chunked_points.get_attribute(label_attribute_id);
	while (file.good() && i < source_point_cloud.get_nr_points()) {
		if (point_labels.size() <= i) {
			std::cerr << "The number of labels defined in the label file exceed the number of points in the point cloud! Any excess labels will be discarded\n";
			break;
		}
		getline(file, line);
		int l = std::stoi(line);
		//assign labels based on file content
		point_labels.data<GLint>()[i++] = make_label(l, point_label_group::VISIBLE);
	}
	if (point_labels.size() > 0) {
		point_labels.upload();
	}

	std::cout << "finish" << std::endl;
}

void pointcloud_labeling_tool::store_label_nr_to(const std::string& fn) {
	auto& chunked_points = point_server_ptr->ref_chunks();
	
	//synchronize point data
	sync_data();
	//open file
	std::fstream file = std::fstream(fn, std::fstream::out);
	if (!file.good()) {
		std::cerr << "can't open file " << fn << std::endl;
		return;
	}
	auto& point_labels = chunked_points.get_attribute(label_attribute_id);
	for (int i = 0; i < chunked_points.num_points();++i) {
		file << remove_label_group(point_labels.data<GLint>()[i]) << '\n';
	}
	file.close();
}


std::pair<std::vector<pointcloud_labeling_tool::LODPoint>, std::vector<GLuint>> pointcloud_labeling_tool::collect_points(cgv::render::context& ctx, int label_groups)
{
	std::pair<std::vector<pointcloud_labeling_tool::LODPoint>, std::vector<GLuint>> collected;
	
	auto& chunked_points = point_server_ptr->ref_chunks();

	sync_data();
	label_shader_manager& sm = ref_label_shader_manager(ctx);
	auto& prog = sm.get_point_collector(ctx);
	prog.set_uniform(ctx, "point_group_mask", label_groups);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, label_shader_manager::labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	std::unordered_map<ivec3, GLuint, int24_ivec3_hasher> result_buffers;

	//run compute shader
	prog.enable(ctx);

	for (auto i_v_pair : chunked_points) {
		GLuint size_data = 0;
		auto& ix = i_v_pair.first;
		auto& ch = i_v_pair.second;
		glCreateBuffers(1, &result_buffers[ix]);
		glNamedBufferData(result_buffers[ix], (ch->size() + 1) * sizeof(GLuint), nullptr, GL_DYNAMIC_READ);
		glNamedBufferSubData(result_buffers[ix], 0, sizeof(GLuint), &size_data);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, result_buffers[ix]);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, label_shader_manager::point_id_pos, ch->id_buffer());
		prog.set_uniform(ctx, "batch_size", ch->size());
		glDispatchCompute((ch->size() / 256) + 1, 1, 1);
	}

	glMemoryBarrier(GL_ALL_BARRIER_BITS);
	
	//collect compute shader results
	for (auto i_v_pair : chunked_points) {
		auto& ix = i_v_pair.first;
		auto& ch = i_v_pair.second;
		//auto buf = result_buffers[ix];
		GLuint* results = static_cast<GLuint*>(glMapNamedBufferRange(result_buffers[ix], 0, sizeof(GLuint) * (ch->size() + 1), GL_MAP_READ_BIT));
		GLuint results_size = results[0]; ++results;

		//const unsigned old_tmp_size = tmp->get_nr_points();
		const unsigned old_tmp_size = collected.first.size();
		collected.first.resize(old_tmp_size + results_size);
		collected.second.resize(old_tmp_size + results_size);

		auto& labels_ref = chunked_points.get_attribute(label_attribute_id);

		for (int i = 0; i < results_size; ++i) {
			int local_ix = results[i];
			collected.first[old_tmp_size + i] = ch->point_at(local_ix);
			collected.second[old_tmp_size + i] = ch->id_at(local_ix);
		}

		glUnmapNamedBuffer(result_buffers[ix]);
		glDeleteBuffers(1, &result_buffers[ix]);
	}

	return collected;
}

void pointcloud_labeling_tool::update_clod_parameters_label()
{
	//update text label
	std::array<char, 512> buff;
	auto& style = source_point_cloud.ref_render_style();
	std::snprintf(buff.data(), buff.size(), clod_parameters_template_str.data(), style.CLOD, style.scale, style.spacing, style.pointSize, style.min_millimeters);
	text_labels.update_label_text(clod_parameters_label_id, std::string(buff.data()));
	text_labels.place_label(clod_parameters_label_id,
		left_tool_label_offset, tool_label_ori, CS_LEFT_CONTROLLER, LA_RIGHT, tool_label_scale);
}

///
void pointcloud_labeling_tool::move_points_to_clipboard(std::vector<LODPoint>& points, std::vector<GLuint>& point_ids, GLint* point_labels) {
	// allocate space in clipboard
	std::unique_ptr<point_cloud_record> tmp_pcr = std::make_unique<point_cloud_record>();
	auto& serv = ref_point_cloud_server(*this->get_context());
	tmp_pcr->scale = serv.ref_point_cloud_scale();
	tmp_pcr->rotation = quat(cgv::math::rotate3_rpy(serv.ref_point_cloud_rotation()));
	pct::point_cloud_record* pcr_ptr = tmp_pcr.get();
	
	// create point_cloud object later stored in the point_cloud_record
	std::unique_ptr<point_cloud> tmp = std::make_unique<point_cloud>();
	tmp->resize(std::min(points.size(), point_ids.size()));
	tmp->create_colors();
	tmp->resize_lods();
	tmp->resize_labels();
	tmp->ref_point_cloud_scale() = serv.ref_point_cloud_scale();
	tmp->ref_point_cloud_position() = source_point_cloud.ref_point_cloud_position();
	tmp->ref_point_cloud_rotation() = serv.ref_point_cloud_rotation();
	int nr_points = tmp->get_nr_points();
	
	for (int i = 0; i < nr_points;++i) {
		tmp->pnt(i) = points[i].position();
		tmp->clr(i) = points[i].color();
		tmp->label(i) = make_label(remove_label_group(point_labels[point_ids[i]]), point_label_group::VISIBLE);
	}
	
	//create a scaled preview
	std::vector<vec3> point_positions;
	std::vector<rgb8> point_colors;
	palette.generate_preview(*tmp, point_positions, point_colors, 0);

	//move lod points to clipboard 
	pcr_ptr->cached_lod_points.swap(points);
	
	//store preview in palette
	palette.replace_pointcloud(palette_clipboard_point_cloud_id, point_positions.data(), point_colors.data(), point_positions.size());
	//store selected points in clipboard
	pcr_ptr->points.swap(tmp);
	pcr_ptr->compute_centroid();

	// move point cloud to clip board
	try {
		palette_clipboard_record_id = clipboard_ptr->move_from(tmp_pcr);
	}
	catch (record_alloc_error& err) {
		//replace instead
		pct::point_cloud_record* pcr_rptr = clipboard_ptr->get_by_id(palette_clipboard_record_id);
		if (pcr_rptr) {
			*pcr_rptr = std::move(*err.moved_record);
			pcr_rptr->compute_centroid();
			clipboard_ptr->update(palette_clipboard_record_id);
			std::cout << "clipboard: Full! Can't allocate new slot, replaced last entry instead\n";
		}
		else {
			std::cerr << "clipboard: Full and failed to replace invalid/deleted record with id=" << palette_clipboard_record_id << "\n";
			return;
		}
	}
}

void pointcloud_labeling_tool::copy_points_to_clipboard(std::vector<LODPoint>& points, std::vector<GLuint>& point_ids, GLint* point_labels)
{
	std::unique_ptr<pct::point_cloud_record> pcr_ptr = std::make_unique<point_cloud_record>();
	std::unique_ptr<point_cloud> tmp = std::make_unique<point_cloud>();

	tmp->resize(std::min(points.size(), point_ids.size()));
	tmp->create_colors();
	tmp->resize_lods();
	tmp->resize_labels();
	tmp->ref_point_cloud_scale() = source_point_cloud.ref_point_cloud_scale();
	tmp->ref_point_cloud_position() = source_point_cloud.ref_point_cloud_position();
	tmp->ref_point_cloud_rotation() = source_point_cloud.ref_point_cloud_rotation();
	int nr_points = tmp->get_nr_points();

	for (int i = 0; i < nr_points; ++i) {
		tmp->pnt(i) = points[i].position();
		tmp->clr(i) = points[i].color();
		tmp->label(i) = make_label(remove_label_group(point_labels[point_ids[i]]), point_label_group::VISIBLE);
	}
	
	pcr_ptr->points.swap(tmp);
	
	std::vector<vec3> point_positions;
	std::vector<rgb8> point_colors;

	pcr_ptr->compute_centroid();

	palette::generate_preview(*pcr_ptr->points, point_positions, point_colors);

	//store preview in palette since the label palette has no event listener yet for the clipboard 
	palette.replace_pointcloud(palette_clipboard_point_cloud_id, point_positions.data(), point_colors.data(), point_positions.size());
	try {
		palette_clipboard_record_id = clipboard_ptr->move_from(pcr_ptr);
	}
	catch (record_alloc_error& err) {
		//replace instead
		pct::point_cloud_record* pcr_rptr = clipboard_ptr->get_by_id(palette_clipboard_record_id);
		if (pcr_rptr) {
			*pcr_rptr = std::move(*err.moved_record);
			pcr_rptr->compute_centroid();
			clipboard_ptr->update(palette_clipboard_record_id);
		}
	}
}

void pointcloud_labeling_tool::on_copy_points() {
	auto& chunked_points = point_server_ptr->ref_chunks();

	auto& collected = collect_points(*this->get_context(), (int)point_label_group::SELECTED_BIT);
	auto& labels_ref = chunked_points.get_attribute(label_attribute_id);
	labels_ref.download();
	//copy_points_to_clipboard(collected.first, collected.second, labels_ref.data<GLint>());
	move_points_to_clipboard(collected.first,collected.second, labels_ref.data<GLint>());
}

void pointcloud_labeling_tool::on_load_clipboard_point_cloud()
{
	point_cloud tmp;
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//read new pointcloud	
	tmp.read(fn);
	tmp.create_colors();

	std::vector<indexed_point> i_lod_points(tmp.get_nr_points());
	int nr_points = i_lod_points.size();
	std::vector<GLint> labels(nr_points);

	for (int i = 0; i < nr_points; ++i) {
		indexed_point p;
		p.position() = tmp.pnt(i);
		p.color() = tmp.clr(i);
		p.index = i;
		i_lod_points[i] = p;
		if (tmp.has_labels())
			labels[i] = tmp.label(i);
		else
			labels[i] = make_label(0, point_label_group::VISIBLE);
	}

	auto& lod_generator = cgv::pointcloud::octree::ref_octree_lod_generator<pct::indexed_point>();
	i_lod_points = lod_generator.generate_lods(i_lod_points);

	std::vector<LODPoint> lod_points(i_lod_points.size());
	std::vector<GLuint> point_ids(i_lod_points.size());
	int nr_lod_points = lod_points.size();
	for (int i = 0; i < nr_points; ++i) {
		lod_points[i] = i_lod_points[i];
		point_ids[i] = i_lod_points[i].index;
	}
	move_points_to_clipboard(lod_points, point_ids, labels.data());
	//copy_points_to_clipboard(lod_points, point_ids, labels.data());
}

///
void pointcloud_labeling_tool::rollback_last_operation(cgv::render::context& ctx) {
	if (point_cloud_interaction_settings.enable_history)
		history_ptr->rollback_last_operation(ctx);
}
///
void pointcloud_labeling_tool::sync_data(int flags)
{
	auto& chunked_points = point_server_ptr->ref_chunks();

	chunked_points.download_buffers();
}

void pointcloud_labeling_tool::copy_chunks_to_point_cloud(point_cloud& dest, std::vector<unsigned>* id_translation_table) {
	int chunks_i = 0, pc_i = 0;
	
	auto& chunked_points = point_server_ptr->ref_chunks();

	size_t num_points = chunked_points.num_points();
	dest.resize(num_points);
	if (!dest.has_lods())
		dest.resize_lods();
	if (!dest.has_labels())
		dest.resize_labels();
	if (!dest.has_colors())
		dest.create_colors();
	
	auto& point_labels = chunked_points.get_attribute(label_attribute_id);
	//translation table for point ids
	if (id_translation_table)
		id_translation_table->resize(point_labels.size(), -1);

	if (dest.has_colors() && dest.has_labels() && dest.has_lods()) {
		for (auto& index_chunk_pair : chunked_points) {
			auto& ch = *index_chunk_pair.second;
			for (int i = 0; i < ch.size();++i) {
				auto& pnt = ch.points[i];
				auto& id = ch.point_ids[i];
				if (id != -1 && point_labels.data<GLint>()[id] != (GLint)point_label_group::DELETED) {
					dest.pnt(pc_i) = pnt.position();
					dest.clr(pc_i) = pnt.color();
					dest.lod(pc_i) = pnt.level();
				
					if (id_translation_table)
						(*id_translation_table)[id] = pc_i;
					dest.label(pc_i) = remove_label_group(point_labels.data<GLint>()[id]);
					++pc_i;
				}
			}
		}
		dest.resize(pc_i);
	}
}

void pointcloud_labeling_tool::update_interaction_mode(const InteractionMode im) {
	interaction_mode = (int)im;
	// set state for CONFIG mode
	if (im == InteractionMode::CONFIG) {
		tool_help_label_allways_drawn[InteractionMode::CONFIG][1] = true;
		tool_help_label_allways_drawn[InteractionMode::CONFIG][0] = true;

		spacing_tool_enabled = (config_mode_tool == (int)ConfigModeTools::CMT_PointSpacingProbe);
		if (spacing_tool_enabled) {
			tool_help_label_id[InteractionMode::CONFIG][1] = spacing_tool_label_id;
			tool_help_label_id[InteractionMode::CONFIG][0] = -1;
		} 
		else {
			tool_help_label_id[InteractionMode::CONFIG][1] = config_help_label_id;
			tool_help_label_allways_drawn[InteractionMode::CONFIG][1] = false;
			tool_help_label_id[InteractionMode::CONFIG][0] = clod_parameters_label_id;
			update_clod_parameters_label();
		}
	}
	else {
		spacing_tool_enabled = false;
	}
	
	
	update_help_labels(im);
	update_controller_labels();
}

void pointcloud_labeling_tool::rescale_point_cloud(float new_scale)
{
	float old_scale = point_server_ptr->ref_point_cloud_scale();
	float old_clod_scale = source_point_cloud.ref_render_style().scale;
	float old_min_millis = source_point_cloud.ref_render_style().min_millimeters;
	//hint: do not use the transformation information from source_pc, these are not directly used for the model transform
	point_server_ptr->ref_point_cloud_scale() = std::max<float>(0.0001f, new_scale);
	source_point_cloud.ref_render_style().scale = std::max<float>(0.0001f, new_scale);
	source_point_cloud.ref_render_style().min_millimeters = source_point_cloud.ref_render_style().min_millimeters * ((double)point_server_ptr->ref_point_cloud_scale() / (double)old_scale);

	update_member(&source_point_cloud.ref_render_style().min_millimeters);
	update_member(&source_point_cloud.ref_render_style().scale);
}

bool pointcloud_labeling_tool::teleport_ray(const vec3& direction, const vec3& origin, const float ray_radius)
{
	float t;
	vec3 p;
	if (point_server_ptr->raycast_points(direction, origin, ray_radius, t, p)) {
		if (overview_mode) {
			mat4 inv_model_transform = (mat4)inv(point_server_ptr->get_point_cloud_model_transform());
			point_server_ptr->restore_transformation_state(stored_state.first);
			//stored_state.first.restore_state(source_pc);
			source_point_cloud.ref_render_style() = stored_state.second;
			// build model_transform
			mat4 new_model_transform = cgv::math::translate4(point_server_ptr->ref_point_cloud_position())
				* cgv::math::rotate4<float>(point_server_ptr->ref_point_cloud_rotation())
				* cgv::math::scale4(point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale(), point_server_ptr->ref_point_cloud_scale());


			p = new_model_transform * inv_model_transform * p.lift();
			overview_mode = false;
		}

		vec3 new_position = vr_view_ptr->get_tracking_origin() + (p - origin);
		vr_view_ptr->set_tracking_origin(new_position + vec3(0.f, 0.5, 0.f));
		//std::cout << "teleported to " << p.x() << "," << p.y() << "," << p.y() << "\n";
		return true;
	}
	return false;
}

void pointcloud_labeling_tool::swap_point_cloud(point_cloud_record& other) {
	sync_data();
	//swap name
	std::swap(source_name, point_cloud_registration.ref_name());
	//copy points from chunks
	std::vector<unsigned> id_to_offset;
	copy_chunks_to_point_cloud(source_point_cloud, &id_to_offset);
	//swap point cloud
	{
		point_cloud tmp;
		tmp = std::move(source_point_cloud);
		source_point_cloud = std::move(*other.points);
		*other.points = std::move(tmp);
		prepare_point_cloud();
	}
	// swap history
	auto tmp_h = history_ptr->make_storeable_copy();
	if (other.stored_history.capacity() == 0) {
		other.stored_history.set_capacity(history_ptr->capacity());
	}
	history_ptr->restore(other.stored_history);
	other.stored_history = std::move(tmp_h);
	//fix point id in history buffer
	for (int i = 0; i < other.stored_history.history_buffer.size(); ++i) {
		auto& op_ref = other.stored_history.history_buffer[i];
		if (op_ref.index >= id_to_offset.size())
			op_ref.index = -1;
		else
			op_ref.index = id_to_offset[op_ref.index];

	}
}

void pointcloud_labeling_tool::on_traject_hmd_cb()
{
	if (!tra_hmd)
		tra_hmd = true;
	else
		tra_hmd = false;
}

bool pointcloud_labeling_tool::hasEnding(std::string const& str, std::string const& ending) {
	if (str.length() >= ending.length()) {
		return str.compare(str.length() - ending.length(), ending.length(), ending) == 0;
	}
	return false;
}

void pointcloud_labeling_tool::on_save_hmd_tra_cb()
{
	const std::string file_ending = ".vrhmd";
	std::string fn = cgv::gui::file_save_dialog("base file name", "hmd configurations(vrhmd):*.vrhmd");
	if (!hasEnding(fn, file_ending)) {
		fn.append(file_ending);
	}
	if (fn.empty())
		return;
	save_hmd_tra(fn);
}

/// generate a new drawing file name
std::string pointcloud_labeling_tool::get_new_tra_file_name()
{
	int i = (int)draw_file_names.size() - 1;
	std::string file_name;
	do {
		++i;
		file_name = "hmd_tra_";
		std::string s = cgv::utils::to_string(i);
		while (s.length() < 5)
			s = std::string("0") + s;
		file_name += s + ".vrhmd";
	} while (cgv::utils::file::exists(draw_file_path + "/" + file_name));
	return file_name;
}

bool pointcloud_labeling_tool::save_hmd_tra(const std::string& fn)
{
	std::stringstream data;

	for (size_t i = 0; i < trajectory_points.size(); ++i) {
		//format: BOX <box.min_p> <box.max_p> <trans> <rot> <col>
		const vec3& hmd_trans = trajectory_points[i];
		const quat& hmd_rot = trajectory_orientation[i];
		const vec4& hmd_color = trajectory_color[i];
		std::string direction = "";
		/*float diff = std::abs(trajectory_points.at(i).y() - tra_las_points.at(i).y());
		quat qd = trajectory_orientation.at(i).conj() * tra_las_ori.at(i);
		float phi = atan2(2 * (qd.w() * qd.x() + qd.y() * qd.z()), 1 - 2 * (qd.x() * qd.x() + qd.y() * qd.y()));  // X axis
		float theta = asin(2 * (qd.w() * qd.y() - qd.z() * qd.x()));        // y axis
		float psi = atan2(2 * (qd.w() * qd.z() + qd.x() * qd.y()), 1 - 2 * (qd.y() * qd.y() + qd.z() * qd.z()));   // z axis
		if (theta > 0.f)
		{
			direction = "right";
		}
		else if (theta <= 0.f)
		{
			direction = "left";
		}*/
		data << "HMD "
			<< hmd_trans << " "
			<< hmd_rot << " "
			<< hmd_color << " "
			<< '\n';
	}
	std::string s = data.str();
	if (!cgv::utils::file::write(fn, s.data(), s.size())) {
		std::cerr << "vr_hmd_tra::save_hmd_tra: failed writing data to file: " << fn;
	}
	return true;
}

bool pointcloud_labeling_tool::load_hmd_tra(const std::string fn) {
	clear_hmd_tra();
	std::string data;
	if (!cgv::utils::file::read(fn, data)) {
		std::cerr << "vr_hmd_tra::load_hmd_tra: failed reading data from file: " << fn << '\n';
		return false;
	}
	std::istringstream f(data);
	std::string line;

	while (!f.eof()) {
		std::getline(f, line); 	//read a line
		std::istringstream l(line);
		std::string sym;

		vec3 last_trajectory_point;
		quat last_trajectory_orientation;

		if (!l.eof()) {
			getline(l, sym, ' '); //get the first symbol determing the type
			if (sym == "HMD") { //in case of a HMD
				std::string direction;
				vec3 pos;
				quat rot;
				cgv::math::fvec<float, 4> clr;
				l >> pos;
				l >> rot;
				l >> clr;

				trajectory_points.push_back(pos);
				trajectory_orientation.push_back(rot);
				trajectory_color.push_back(clr);
			}
		}
	}

	//recreate tra_las_point and tra_las_ori
	if (trajectory_points.size() > 0) {
		tra_las_points.resize(trajectory_points.size());
		tra_las_ori.resize(trajectory_points.size());
		int i = trajectory_points.size() - 1;
		while (i > 0) {
			tra_las_points[i] = trajectory_points[i - 1];
			tra_las_ori[i] = trajectory_orientation[i - 1];
			--i;
		}
		tra_las_points[0] = trajectory_points[0];
		tra_las_ori[0] = trajectory_orientation[0];
		return true;
	}
	return false;
}

void pointcloud_labeling_tool::draw_hmd_tra(std::vector<vec3>& hmd_translations, std::vector<vec4>& hmd_colors) {
	vec4 color;
	//add start and end point of each correspondence in world coordinates to points
	for (int i = 0; i < trajectory_points.size(); ++i) {
		hmd_translations.push_back(tra_las_points.at(i));
		hmd_translations.push_back(trajectory_points.at(i));
		float diff = trajectory_points.at(i).y() - tra_las_points.at(i).y();
		/*quat qd = trajectory_orientation.at(i).conj() * tra_las_ori.at(i);
		float phi = atan2(2 * (qd.w() * qd.x() + qd.y() * qd.z()), 1 - 2 * (qd.x() * qd.x() + qd.y() * qd.y()));  // X axis
		float theta = asin(2 * (qd.w() * qd.y() - qd.z() * qd.x()));        // y axis
		float psi = atan2(2 * (qd.w() * qd.z() + qd.x() * qd.y()), 1 - 2 * (qd.y() * qd.y() + qd.z() * qd.z()));   // z axis*/
		if (diff > 0.0f)
		{
			color = vec4(1.0, 1.0, 0.0, 0.8);
			hmd_colors.push_back(color);
			hmd_colors.push_back(color);
		}
		else if (diff <= 0.0f)
		{
			color = vec4(0.0, 1.0, 0.0, 0.8);
			hmd_colors.push_back(color);
			hmd_colors.push_back(color);
		}
	}
}

void pointcloud_labeling_tool::clear_hmd_tra() {
	tra_las_points.clear();
	tra_las_ori.clear();
	trajectory_points.clear();
	trajectory_orientation.clear();
	trajectory_color.clear();
}

void pointcloud_labeling_tool::restore_trajectory_file_list(const std::string& dir)
{
	void* hdl = cgv::utils::file::find_first(dir + "/*.vrhmd");
	while (hdl) {
		if (!cgv::utils::file::find_directory(hdl)) {
			draw_file_names.push_back(cgv::utils::file::find_name(hdl));
		}
		hdl = cgv::utils::file::find_next(hdl);
	}
}

void pointcloud_labeling_tool::read_depth_buffer() {
	GLuint fbo_nml = -1;
	glGenFramebuffers(1, &fbo_nml);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_nml);
	// Bind G-buffer framebuffers and textures
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 1, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, 2, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 3, 0);

	// Set viewport to the size of your buffers
	glViewport(0, 0, 720, 1080);

	// Use your deferred rendering shader program for the first pass
	//glUseProgram(deferredRenderingShader);

	// Render your scene
	// In the vertex shader, pass position and normal to the fragment shader
	// In the fragment shader, output position, normal, and depth to the G-buffer textures
	// The depth can be obtained using gl_FragCoord.z, which contains the depth value [0, 1]

	// After rendering, unbind G-buffer framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

void pointcloud_labeling_tool::create_gui()
{
	//auto start_g = std::chrono::steady_clock::now();

	add_decorator("Point cloud", "heading", "level=1");
	connect_copy(add_button("load point cloud","tooltip='read a pointcloud from file'")->click, rebind(this, &pointcloud_labeling_tool::on_load_point_cloud_cb));

	add_member_control(this, "old label format", file_contains_label_groups, "check",
			"tooltip='Affects only loading. Previous versions of this application stored the point labels as they were in memory to the file. But newer versions remove temporary markings before storing, which leads to a different interpretation of the label attribute.'");
	std::string mode_defs = "enums='random=2;octree=1'";
	connect_copy(add_control("lod generator", (DummyEnum&)preparation_settings.lod_mode, "dropdown", mode_defs)->value_change,
		rebind(this, &pointcloud_labeling_tool::on_lod_mode_change));

	connect_copy(add_button("save point cloud")->click, rebind(this, &pointcloud_labeling_tool::on_save_point_cloud_cb));
	connect_copy(add_button("clear point cloud", 
		"tooltip='replaces the active pointcloud with an empty one'")->click, rebind(this, &pointcloud_labeling_tool::on_clear_point_cloud_cb));
	add_member_control(this, "dedupe points on lod generation", preparation_settings.allow_point_deduplication, "check",
		"tooltip='remove/merge points with almost the same coordinates when adding LoD'");
	add_decorator("Labels", "heading", "level=2");
	connect_copy(add_button("load s3d labels", "tooltip='read labels from a file'")->click, rebind(this, &pointcloud_labeling_tool::on_load_s3d_labels_cb));
	connect_copy(add_button("store s3d labels", "tooltip='write labels line wise as ASCII text to a file'")->click, rebind(this, &pointcloud_labeling_tool::on_store_s3d_labels_cb));
	connect_copy(add_button("load label palette", "tooltip='load different palette labels and colors'")->click, rebind(this, &pointcloud_labeling_tool::on_load_palette_labels));
	connect_copy(add_button("random lpc", "tooltip='create a random cube shaped point cloud'")->click, rebind(this, &pointcloud_labeling_tool::on_generate_large_pc_cb));

	add_member_control(this, "adapt CLOD", adapt_clod, "check",
		"tooltip='adapt the clod parameters to current point cloud and GPU'");
	add_view("fps", frame_rate, "view");
	
	std::string toggle_show_labeled_points_only_txt = (visible_point_groups & 1) ? "show labeled point only" : "show all points";
	connect_copy(add_button(toggle_show_labeled_points_only_txt)->click, rebind(this, &pointcloud_labeling_tool::on_toggle_show_labeled_points_only));
	//show surrounding environment
	add_decorator("Environment", "heading", "level=1");
	add_member_control(this, "show environment", world.show_environment, "toggle");
	add_member_control(this, "show table", world.show_table, "toggle");
	add_member_control(this, "show floor", world.show_floor, "toggle");
	add_member_control(this, "show help", always_show_help, "toggle");
	add_member_control(this, "show button labels", show_controller_labels, "toggle");
	add_member_control(this, "show left bar", show_left_bar, "toggle");
	// show lod color and disable main view
	add_decorator("Other", "heading", "level=2");
	add_member_control(this, "disable_main_view", disable_main_view, "toggle");
	connect_copy(add_button("colorize with height")->click, rebind(this, &pointcloud_labeling_tool::colorize_with_height));
	connect_copy(add_button("print point cloud info")->click, rebind(this, &pointcloud_labeling_tool::print_point_cloud_info));

	if (begin_tree_node("Performance", gui_performance, gui_performance)) {
		align("\a");
		align("\b");
	}
	
	if (begin_tree_node("Teleport", gui_teleport, gui_teleport)) {
		align("\a");
		add_member_control(this, "radius", teleport_spere_radius_factor, "value_slider", "min=0.0f;max=0.2f;log=false;ticks=true");
		add_member_control(this, "ray radius", teleport_ray_radius, "value_slider", "min=0.0f;max=0.2f;log=false;ticks=true");
		add_member_control(this, "show ray", show_teleport_ray, "toggle");
		align("\b");
	}
	if (begin_tree_node("Rendering", gui_rendering, gui_rendering)) {
		align("\a");
		if (begin_tree_node("CLOD Rendering", gui_clod_rendering, gui_clod_rendering)) {
			align("\a");
			add_member_control(this, "show LODs", color_based_on_lod, "toggle", "tooltip='colorize points based on their LOD, red = high, blue = low'");
			add_member_control(this, "use vr kit as pivot", use_vr_kit_as_pivot, "toggle","tooltip='change the relevant reference point use by the reduction step'");
			add_member_control(this, "point limit", max_points, "value_slider", "min=10000;max=100000000;log=true;ticks=true");

			if (begin_tree_node("CLOD render style", source_point_cloud.ref_render_style(), true)) {
				align("\a");
				add_gui("clod style", source_point_cloud.ref_render_style());
				align("\b");
				end_tree_node(source_point_cloud.ref_render_style());
			}
			add_member_control(this, "keep filtered point count at", clod_controller.targeted_visible_points, "value_slider", "min=1.0;max=4000000.0;log=true;");
			add_member_control(this, "share reduction pass", share_reduction_pass, "check","tooltip='reduce points once per frame and use the result for both eyes'");
			add_member_control(this, "skip chunk frustum culling", skip_frustum_culling, "check", "tooltip='disable chunk level frustum culling for rendering'");
			add_member_control(this, "ext. frustum size", extended_frustum_size, "value_slider", "min=1.0;max=3.0;log=false;ticks=true;tooltip='multiplier for extended frustum if share reduction pass and/or distribute reduction is enabled'");
			add_member_control(this, "distribute\n reduction[frames]", max_reduction_epoch, "value_slider", "min=0;max=10;log=false;ticks=true;tooltip='run incremental reduction spread over multiple frame computations'");
			add_member_control(this, "lights", enable_lights, "check","tooltip = 'enable shading[Broken]'");
			align("\b");
		}
		add_decorator("Point labels", "heading", "level=2");
		add_member_control(this, "use label shader", use_label_prog, "check");
		if (begin_tree_node("Point Groups", visible_point_groups)) {
			align("\a");
			auto ptr_v = add_control<bool>("default group", &default_point_group_control, "check");
			auto ptr_s = add_control<bool>("selected points", &selected_point_group_control, "check");
			auto ptr_l = add_control<bool>("labeled points", &labeled_point_group_control, "check");
			align("\b");
		}
		align("\b");
	}

	if (begin_tree_node("Auto Point Cloud Positioning", gui_scaling, gui_scaling)) {
		add_member_control(this, "auto-scale pointcloud", pointcloud_fit_table, "toggle");
		connect_copy(add_button("put on table")->click, rebind(this, &pointcloud_labeling_tool::on_point_cloud_fit_table));
		//connect_copy(add_button("move point cloud to center")->click, rebind(this, &pointcloud_cleaning_tool::on_move_to_center));
	}

	if (begin_tree_node("Point Cloud Chunking", preparation_settings.auto_chunk_cube_size)) {
		align("\a");
		add_member_control(this, "automatic chunk size", preparation_settings.auto_chunk_cube_size, "check");
		add_member_control(this, "chunks in one direction", preparation_settings.auto_chunk_max_num_chunks, "value_slider", "min=1;max=1000;log=false;ticks=true");
		add_member_control(this, "show chunk bounding boxes", draw_chunk_bounding_boxes, "check");
		add_member_control(this, "chunk cube size", preparation_settings.chunk_cube_size, "value_slider", "min=0.1;max=5.0;log=false;ticks=true");
		connect_copy(add_button("rechunk")->click, rebind(this, &pointcloud_labeling_tool::on_rechunk));
		connect_copy(add_button("disable chunks",
			"tooltip='merge all points into an infinite chunk, this automatically enables skip frustum culling under the render settings'")->click, rebind(this, &pointcloud_labeling_tool::on_make_infinite_chunk));
		
		if (begin_tree_node("cone style", cone_style, false)) {
			align("\a");
			add_gui("cone style", cone_style);
			align("\b");
			end_tree_node(cone_style);
		}
		align("\b");
	}

	if (begin_tree_node("Point Cloud Marking", gui_marking, gui_marking)) {
		align("\a");
		add_member_control(this, "use chunks", point_cloud_interaction_settings.use_chunks, "check",
			"tooltip='This only affects tools! If enabled tools will use the chunk accelleration structure. For rendering without chunks go to \"Point Cloud Chunking\" and make a monolithic chunk'");
		connect_copy(add_button("clear all labels")->click, rebind(this, &pointcloud_labeling_tool::on_clear_all_labels));
		connect_copy(add_button("rollback one step")->click, rebind(this, &pointcloud_labeling_tool::on_rollback_cb));
		add_member_control(this, "enable label history", point_cloud_interaction_settings.enable_history, "check");
		
		if (begin_tree_node("Palette", palette, false)) {
			align("\a");
			add_gui("Palette", palette);
			align("\b");
		}

		if (begin_tree_node("point cloud cleaning", gui_cleaning_tools, false)) {
			add_gui("sphere_style_rhand", sphere_style_rhand);
			add_gui("sphere_style_lhand", sphere_style_lhand);
			end_tree_node(gui_cleaning_tools);
		}
		align("\b");
		end_tree_node(gui_marking);
	}

	if (begin_tree_node("Manual Positioning", source_point_cloud)) {
		add_member_control(this, "model scale", source_point_cloud.ref_point_cloud_scale(), "value_slider", "min=0.1;max=5.0;log=false;ticks=true");
		add_member_control(this, "model position x", source_point_cloud.ref_point_cloud_position().x(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model position y", source_point_cloud.ref_point_cloud_position().y(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model position z", source_point_cloud.ref_point_cloud_position().z(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model rotation x", source_point_cloud.ref_point_cloud_rotation().x(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
		add_member_control(this, "model rotation y", source_point_cloud.ref_point_cloud_rotation().y(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
		add_member_control(this, "model rotation z", source_point_cloud.ref_point_cloud_rotation().z(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
	}
}
#include "lib_begin.h"
#include <cgv/base/register.h>

cgv::base::object_registration<pointcloud_labeling_tool> pointcloud_cleaning_tool_reg("");