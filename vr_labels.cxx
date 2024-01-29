#include "vr_labels.h"
#include <cgv/math/pose.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>

void vr_labels::draw(cgv::render::context& ctx)
{
	// draw labels
	
	// compute label poses in lab coordinate system
	update_coordinate_systems();

	for (int32_t i = 0; i < get_num_labels(); ++i) {
		if (!label_visibilities[i] || !valid[label_coord_systems[i]])
			continue;

		draw(ctx, i);
	}
}

void vr_labels::draw(cgv::render::context& ctx, int li)
{
	if (!valid[label_coord_systems[li]])
		return;
	
	mat34 label_pose = cgv::math::pose_construct(label_orientations[li], label_positions[li]);
	cgv::math::pose_transform(pose[label_coord_systems[li]], label_pose);
	vec3 point = cgv::math::pose_position(label_pose);
	quat rotation = quat(cgv::math::pose_orientation(label_pose));
	vec2 extent = label_extents[li];
	vec4 tex_coords = label_textures_range[li];
	
	label_textures[li].enable(ctx,0); //using a texture atlas would be better
	
	{
		rect_renderer.set_position_array(ctx, &point, 1);
		rect_renderer.set_rotation_array(ctx, &rotation, 1);
		rect_renderer.set_extent_array(ctx, &extent, 1);
		rect_renderer.set_texcoord_array(ctx, &tex_coords, 1);
		rect_renderer.render(ctx, 0, 1);
	}

	label_textures[li].disable(ctx);
}

void vr_labels::draw_multiple(cgv::render::context& ctx, int li, vec3* positions, quat* orientations, size_t size)
{
	if (!valid[label_coord_systems[li]])
		return;

	std::vector<vec3> points(size);
	std::vector<quat> rotations(size);

	for (size_t instance = 0; instance < size; ++instance) {
		mat34 label_pose = cgv::math::pose_construct(orientations[instance], positions[instance]);
		cgv::math::pose_transform(pose[label_coord_systems[li]], label_pose);
		points[instance] = cgv::math::pose_position(label_pose);
		rotations[instance] = quat(cgv::math::pose_orientation(label_pose));
	}

	vec2 extent = label_extents[li];
	vec4 tex_coords = label_textures_range[li];

	label_textures[li].enable(ctx, 0); //using a texture atlas would be better

	for (size_t instance = 0; instance < size; ++instance)
	{
		rect_renderer.set_position_array(ctx, &points[instance], 1);
		rect_renderer.set_rotation_array(ctx, &rotations[instance], 1);
		rect_renderer.set_extent_array(ctx, &extent, 1);
		rect_renderer.set_texcoord_array(ctx, &tex_coords, 1);
		rect_renderer.render(ctx, 0, 1);
	}

	label_textures[li].disable(ctx);
}

void vr_labels::update_coordinate_systems()
{
	valid[CS_LAB] = true;
	pose[CS_LAB].identity();
	valid[CS_HEAD] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->hmd.status == vr::VRS_TRACKED;
	if (valid[CS_HEAD])
		pose[CS_HEAD] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->hmd.pose[0]);
	valid[CS_LEFT_CONTROLLER] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[0].status == vr::VRS_TRACKED;
	if (valid[CS_LEFT_CONTROLLER])
		pose[CS_LEFT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[0].pose[0]);
	valid[CS_RIGHT_CONTROLLER] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[1].status == vr::VRS_TRACKED;
	if (valid[CS_RIGHT_CONTROLLER])
		pose[CS_RIGHT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[1].pose[0]);
}

void vr_labels::init(cgv::render::context& ctx)
{
	ctx_ptr = &ctx;
	rect_renderer.init(ctx);
}

void vr_labels::init_frame(cgv::render::context& ctx)
{
	if (!run_init_frame)
		return;
	for (int i = 0; i < get_num_labels(); ++i) {
		int label_resolution = label_textures_resolution[i].x();
		//if (label_fbos[i].get_width() != label_resolution) {
			if (label_textures[i].is_created()) {
				label_textures[i].destruct(ctx);
			}
			if (label_fbos[i].is_created())
				label_fbos[i].destruct(ctx);
		//}
		if (!label_textures[i].is_created()) {
			label_textures[i].create(ctx, cgv::render::TT_2D);
		}
		if (!label_fbos[i].is_created()) {
			label_fbos[i].create(ctx, label_resolution, label_resolution);
			label_fbos[i].attach(ctx, label_textures[i]);
		}

		//if (label_changed[i] && label_fbos[i].is_complete(ctx)) {
			glPushAttrib(GL_COLOR_BUFFER_BIT);
			label_fbos[i].enable(ctx);
			label_fbos[i].push_viewport(ctx);
			ctx.push_pixel_coords();
			glClearColor(label_color[i][0], label_color[i][1], label_color[i][2], 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);


			glColor4f(1.f, 1.f, 1.f, 1.f);
			ctx.enable_font_face(label_font_faces[i], label_font_size[i]);
			
			ctx.set_cursor(
				(label_resolution - 1) * label_textures_range[i](0) + label_borders[i].x(),
				(label_resolution - 1) * label_textures_range[i](3) - label_borders[i].y() - label_font_size[i]);

			ctx.output_stream() << label_text[i];
			ctx.output_stream().flush();

			ctx.pop_pixel_coords();
			label_fbos[i].pop_viewport(ctx);
			label_fbos[i].disable(ctx);
			glPopAttrib();
			label_changed[i] = false;

			label_textures[i].generate_mipmaps(ctx);
		//}
	}
	run_init_frame = false;
}

int32_t vr_labels::add_label(const std::string& text, const rgba& bg_clr, int border_x, int border_y, int width, int height)
{
	uint32_t label_id = label_positions.size();
	schedule_init_frame();

	resize(get_num_labels() + 1);
	set_label(label_id, text, bg_clr, border_x, border_y, width, height);

	return label_id;
}

bool vr_labels::check_handle(int32_t label_index)
{
	return label_index >= 0 && (size_t)label_index < label_positions.size();
}

void vr_labels::resize(uint32_t size)
{
	if (size < get_num_labels()) {
		//free resources
		for (unsigned i = label_textures.size()-1; i >= size; --i) {
			if (label_textures[i].is_created())
				label_textures[i].destruct(*ctx_ptr);
			if (label_fbos[i].is_created())
				label_fbos[i].destruct(*ctx_ptr);
		}
	}
	label_textures.resize(size);
	label_fbos.resize(size);

	label_positions.resize(size);
	label_orientations.resize(size);
	label_extents.resize(size, vec2(1.0f));
	label_size.resize(size);
	label_borders.resize(size);
	label_textures_range.resize(size);
	label_textures_resolution.resize(size);
	label_text.resize(size);
	label_color.resize(size);
	label_visibilities.resize(size);
	label_changed.resize(size);
	label_coord_systems.resize(size);

	label_face_types.resize(size);
	label_font_size.resize(size);
	label_font_faces.resize(size);
	label_font_idx.resize(size);
}

void vr_labels::set_label(int32_t li, const std::string& text, const rgba& bg_clr, int border_x, int border_y, int width, int height)
{	
	schedule_init_frame();
	//OpenGL only accepts texture resolutions that are a power of two and ge_pow2 returns a greater or equal power of two of its argument.
	// For example ge_pow2(300) will return 512. Since label_size is given in pixels, ge_pow2 is used to allocate a texture with enough space for the label.
	auto ge_pow2 = [](unsigned i) {
		unsigned r = 1;
		while (i > 0) {
			i = i >> 1;
			r = r << 1;
		}
		return r;
	};
	
	label_size[li] = ivec2(width, height);
	label_positions[li] = vec3(0.0);
	label_orientations[li] = quat(1.0, 0.0, 0.0, 0.0);
	label_visibilities[li] = true;
	label_coord_systems[li] = CS_LAB;
	label_text[li] = text;
	label_color[li] = vec3(bg_clr.R(), bg_clr.G(), bg_clr.B());

	label_changed[li] = true;

	//add fixed border
	label_borders[li] = vec2(border_x,border_y);

	//find font

	//set fixed font type for now
	const std::string font = "calibri";
	cgv::media::font::FontFaceAttributes label_face_type = cgv::media::font::FFA_BOLD;
	label_face_types[li] = label_face_type;
	label_font_size[li] = 25;

	for (unsigned i = 0; i < font_names.size(); ++i) {
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == font) {
			label_font_faces[li] = (cgv::media::font::find_font(fn)->get_font_face(label_face_type));
			label_font_idx[li] = i;
			break;
		}
	}
	if (width < 0 || height < 0) {
		compute_label_size(li);
	}
	
	if (!label_textures[li].is_created()) {
		label_textures[li] = cgv::render::texture("uint8[R,G,B,A]");
		label_textures[li].set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_textures[li].set_mag_filter(cgv::render::TF_LINEAR);
	}
	cgv::render::texture& tex = label_textures[li];
	unsigned res = ge_pow2(std::max(std::abs(label_size[li].x()), std::abs(label_size[li].y())));
	tex.set_resolution(0, res);
	tex.set_resolution(1, res);
	
	label_textures_resolution[li] = ivec2(res, res);
	
	label_textures_range[li] = vec4(
		0.f, 0.f, abs(((float)label_size[li].x()) / (float)res), abs((float)(label_size[li].y()) / (float)res)
	);
}

void vr_labels::place_label(int32_t li, const vec3& pos, const quat& ori, CoordinateSystem coord_system, LabelAlignment align, float scale)
{
	schedule_init_frame();
	label_extents[li] = vec2(scale * pixel_scale * std::abs(label_size[li].x()), scale * pixel_scale * std::abs(label_size[li].y()));
	static vec2 offsets[5] = { vec2(0.0f,0.0f), vec2(0.5f,0.0f), vec2(-0.5f,0.0f), vec2(0.0f,0.5f), vec2(0.0f,-0.5f) };
	label_positions[li] = pos + ori.get_rotated(vec3(offsets[align] * label_extents[li], 0.0f));
	label_orientations[li] = ori;
	label_coord_systems[li] = coord_system;
}

const std::string& vr_labels::get_label_text(int32_t li)
{
	return label_text[li];
}

void vr_labels::compute_label_size(int li)
{
	const vec2& label_border = label_borders[li];

	int nr_lines = -1;
	if (label_size[li].x() < 0) {
		std::vector<cgv::utils::token> toks;
		cgv::utils::split_to_tokens(label_text[li], toks, "", false, "", "", "\n");
		nr_lines = int(toks.size());
		int w = -1;
		for (auto t : toks) {
			int new_w = (int)ceil(label_font_faces[li]->measure_text_width(cgv::utils::to_string(t), label_font_size[li]));
			w = std::max(w, new_w);
		}
		label_size[li].x() = -(w + 2 * label_border.x());
	}
	if (label_size[li].y() < 0) {
		if (nr_lines == -1) {
			std::vector<cgv::utils::token> toks;
			cgv::utils::split_to_tokens(label_text[li], toks, "\n", false, "", "", "");
			nr_lines = int(toks.size());
		}
		int h = (int)ceil(1.0f * label_font_size[li]);
		label_size[li].y() = -(int)(2 * label_border.y() + nr_lines * h + 0.2f * label_font_size[li] * (nr_lines - 1));
	}
}

void vr_labels::set_vr_view(vr_view_interactor* ptr)
{
	vr_view_ptr = ptr;
}

void vr_labels::clear(cgv::render::context& ctx) {
	rect_renderer.clear(ctx);
	ctx_ptr = nullptr;

	for (int i = 0; i < get_num_labels(); ++i) {
		label_fbos[i].destruct(ctx);
		label_textures[i].destruct(ctx);
		label_changed[i] = true;
	}
}


vr_labels::vr_labels() : ctx_ptr(nullptr), vr_view_ptr(nullptr)
{
	//enumerate font names for later search
	cgv::media::font::enumerate_font_names(font_names);
}

size_t vr_labels::get_num_labels() const
{
	return label_fbos.size();
}


