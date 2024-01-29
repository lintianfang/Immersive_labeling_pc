#pragma once

#include <cgv/render/shader_program.h>
#include <unordered_map>
#include <array>
#include <unordered_set>

enum selection_shape {
	SS_NONE = 0,
	SS_SPHERE = 1,
	SS_PLANE = 2,
	SS_CUBOID = 3,
	SS_ALL = 4,
	NUM_OF_SHAPES
};

enum light_model {
	LM_NONE = 0,
	LM_LOCAL = 1,
	NUM_OF_LIGHT_MODELS
};

class label_shader_manager {
public:
	/// buffer binding point used by the managed shaders
	static constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4;

	label_shader_manager() = default;

	void manage_singleton(cgv::render::context& ctx, int& ref_count, int ref_count_change);

	//build shader programs
	bool init(cgv::render::context& ctx);

	//delete shader programs
	void clear(cgv::render::context& ctx);

	/// returns a shader program for the requested shape
	cgv::render::shader_program& get_labeling_tool(const selection_shape& shape);

	/// returns a shader program for the requested point pushing tool
	cgv::render::shader_program& get_pushing_tool(const selection_shape& shape);

	// returns a shader program that counts points
	cgv::render::shader_program& get_counting_tool(cgv::render::context& ctx, const selection_shape& shape);

	bool has_labeling_tool(const selection_shape) const;

	bool has_pushing_tool(const selection_shape) const;

	/// Point collection shader for copy paste operations of selections.
	///	Main shader file is point_collector.glcs
	///	Uniforms are int batch size and int point_group_mask.
	///	shader storage buffers(binding point): ssInputIdBuffer(4), ssResultBuffer(0), ssLabels(6)
	/// This program should run for all points with a group size of 256
	cgv::render::shader_program& get_point_collector(cgv::render::context& ctx);

	cgv::render::shader_program& get_clod_render_shader(cgv::render::context& ctx, const selection_shape& shape, const light_model lights);

	bool is_initialized() const;

protected:
	//attaches selector shader code and returns a list of file names from the attached codes. This will not aattach any code from a file which name is already inside prog_files
	void add_selector_code_to(cgv::render::context& ctx, cgv::render::shader_program& prog, int selection_shape_id, std::unordered_set<std::string>* prog_files = nullptr) const;

private:
	std::array<cgv::render::shader_program, (size_t)selection_shape::NUM_OF_SHAPES> labeling_tool_shaders;
	// sets of file names from files already added to shaders. Used to avoid adding the same shader codes twice
	std::array<std::unordered_set<std::string>, (size_t)selection_shape::NUM_OF_SHAPES> labeling_tool_shaders_files;

	std::array<cgv::render::shader_program, (size_t)selection_shape::NUM_OF_SHAPES> pushing_tool_shaders;
	std::array<cgv::render::shader_program, (size_t)selection_shape::NUM_OF_SHAPES> counting_probe_shaders;
	std::array<cgv::render::shader_program, (size_t)selection_shape::NUM_OF_SHAPES*(size_t)light_model::NUM_OF_LIGHT_MODELS> clod_render_shaders;
	cgv::render::shader_program collection_prog;



	static const std::unordered_map<int, std::vector<std::string>> map_of_selectors;
	static const std::unordered_map<int, std::string> label_shader_manager::map_of_shape_postfixes;

	static const std::set<selection_shape> pushing_shapes;
	static const std::set<selection_shape> labeling_shapes;

	//status
	bool is_initialized_p = false;
};

label_shader_manager& ref_label_shader_manager(cgv::render::context& ctx, int ref_count_change = 0);