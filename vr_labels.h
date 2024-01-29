#pragma once
#include <cgv/render/render_types.h>
#include <cgv/render/context.h>
#include <vr_view_interactor.h>
#include <cgv_gl/rectangle_renderer.h>
#include <cgv/render/texture.h>
#include <cgv/render/frame_buffer.h>

/// different text label alignments
enum LabelAlignment
{
	LA_CENTER,
	LA_LEFT,
	LA_RIGHT,
	LA_BOTTOM,
	LA_TOP
};
/// coordinate systems based on objects. used for text labels
enum CoordinateSystem
{
	CS_LAB,
	CS_HEAD,
	CS_LEFT_CONTROLLER,
	CS_RIGHT_CONTROLLER,
	NUM_COORDINATE_SYSTEMS
};

struct place_label_parameter_pack : public cgv::render::render_types {
	vec3& pos;
	quat& ori;
	CoordinateSystem coord_system;
	LabelAlignment align;
	float scale;
};

class vr_labels : public cgv::render::render_types {
public:
	/// draw all labels marked as visible
	void draw(cgv::render::context& ctx);
	/// draw the label specified by label_index, may call update_coordinate_systems() before
	void draw(cgv::render::context& ctx, int label_index);
	/// draw the label specified by label_index multiple times, override position and orientation with the data given by the positions and orientations pointers
	void draw_multiple(cgv::render::context& ctx, int label_index, vec3* positions ,quat* orientations, size_t size);

	/// sets origins and orientation of the used coordinate systems based on given vr_view.
	/// needs to be called again if the objects associated with these Coordinate systems moved since the last call (e.g a controller moved)
	void update_coordinate_systems();

	void init(cgv::render::context& ctx);
	void init_frame(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	/// returns a handle/label_index for use with the other methods in this class
	int32_t add_label(const std::string& text, const rgba& bg_clr, int border_x=4, int border_y=4, int width=-1, int height=-1);
	
	bool check_handle(int32_t label_index);

	void resize(uint32_t size);
	/// set color, text and size, set width and height = -1 for automatic sizing
	void set_label(int32_t label_index, const std::string& text, const rgba& bg_clr, int border_x = 4, int border_y = 4, int width = -1, int height = -1);
	/// place the label in the given coordinate system
	void place_label(int32_t li, const vec3& pos, const quat& ori, CoordinateSystem coord_system, LabelAlignment align, float scale);

	
	inline void hide_label(int32_t li) {
		if (li >= 0 && li < label_visibilities.size())
			label_visibilities[li] = false;
	}
	
	inline void show_label(int32_t li) { 
		if (li >= 0 && li < label_visibilities.size())
			label_visibilities[li] = true;
	}


	inline void update_label_text(int32_t li, const std::string& text) { 
		set_label(li, text, rgba(label_color[li].x(), label_color[li].y(), label_color[li].z(), 1.0));
	}

	const std::string& get_label_text(int32_t li);

	/// computes the labels size based on the text stored in label_text[li], writes result to label_size[li],
	/// scaled texture coordinates will be negative
	void compute_label_size(int li);
	 
	vec2 get_label_extent(int32_t li) {
		return label_extents[li];
	}
	/// required for placing labels in vr kit related coordinate systems
	void set_vr_view(vr_view_interactor* ptr);

	vr_labels();

	size_t get_num_labels() const;

	inline void schedule_init_frame() { run_init_frame = true; }
private:
	cgv::render::context* ctx_ptr;
	vr_view_interactor* vr_view_ptr;

	cgv::render::rectangle_renderer rect_renderer;

	float pixel_scale = 0.002f;

	//state
	bool run_init_frame = true;

	//labels
	std::vector<cgv::render::texture> label_textures;
	std::vector<cgv::render::frame_buffer> label_fbos;

	std::vector<vec3> label_positions;
	std::vector<quat> label_orientations;
	std::vector<vec2> label_extents;
	std::vector<ivec2> label_size;
	std::vector<ivec2> label_borders;
	//label_textures_range contains two 2d points describing a rectangle in the texture. This rectangle should contain the label.
	std::vector<vec4> label_textures_range;
	std::vector<ivec2> label_textures_resolution;
	std::vector<std::string> label_text;
	std::vector<vec3> label_color;
	std::vector<bool> label_visibilities;
	std::vector<bool> label_changed;
	std::vector<CoordinateSystem> label_coord_systems;

	std::vector<cgv::media::font::font_face_ptr> label_font_faces;
	std::vector<cgv::media::font::FontFaceAttributes> label_face_types;
	std::vector<int> label_font_idx;
	std::vector<int> label_font_size;

	std::vector<const char*> font_names;
	
	// rendering
	mat34 pose[NUM_COORDINATE_SYSTEMS];
	bool valid[NUM_COORDINATE_SYSTEMS]; //

};