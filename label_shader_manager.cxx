
#include "label_shader_manager.h"
#include <array>
#include <utility>
#include <set>

namespace {
	void shaderCheckError(cgv::render::shader_program& prog, const char name[]) {
#ifndef NDEBUG
		if (prog.last_error.size() > 0) {
			std::cerr << "error in " << name << "\n" << prog.last_error << '\n';
			prog.last_error = "";
		}
#endif // #ifdef NDEBUG
	}

	void attach_and_check(cgv::render::shader_program& shader_prog, const std::string file)
	{
		shader_prog.attach_file(*shader_prog.ctx_ptr, file, cgv::render::ST_COMPUTE);
		shaderCheckError(shader_prog, file.c_str());
	}
}

//add new shapes here
const std::unordered_map<int, std::vector<std::string>> label_shader_manager::map_of_selectors = {
		{selection_shape::SS_SPHERE,{"selection_sphere.glsl"}},
		{selection_shape::SS_CUBOID,{"selection_cuboid.glsl", "quaternion.glsl"}},
		{selection_shape::SS_PLANE,{"selection_plane.glsl"}},
		{selection_shape::SS_ALL,{"selection_all.glsl"}}
};

//ss: selection of sphere sp: selection of plane sc: selection of cube
const std::unordered_map<int, std::string> label_shader_manager::map_of_shape_postfixes = {
		{selection_shape::SS_NONE,"no_selection"},		
		{selection_shape::SS_SPHERE,"ss"},
		{selection_shape::SS_CUBOID,"sc"},
		{selection_shape::SS_PLANE,"sp"}
};

const std::set<selection_shape> label_shader_manager::pushing_shapes = {
		{selection_shape::SS_SPHERE},
		{selection_shape::SS_CUBOID}
};

// valid shapes for the labeling tool
const std::set<selection_shape> label_shader_manager::labeling_shapes = {
		{selection_shape::SS_SPHERE},
		{selection_shape::SS_CUBOID},
		{selection_shape::SS_PLANE},
		{selection_shape::SS_ALL}
};


void label_shader_manager::manage_singleton(cgv::render::context& ctx, int& ref_count, int ref_count_change)
{
	switch (ref_count_change) {
	case 1:
		if (ref_count == 0) {
			if (!init(ctx))
				ctx.error(std::string("unable to initialize label_shader_manager singleton"));
		}
		++ref_count;
		break;
	case 0:
		break;
	case -1:
		if (ref_count == 0)
			ctx.error(std::string("attempt to decrease reference count of label_shader_manager singleton below 0"));
		else {
			if (--ref_count == 0)
				clear(ctx);
		}
		break;
	default:
		ctx.error(std::string("invalid change reference count outside {-1,0,1} for label_shader_manager singleton"));
	}
}

bool label_shader_manager::init(cgv::render::context& ctx)
{
	std::vector<std::string> dependencies = { "point_labeler_tool.glcs", "point_label.glsl", "point_label_history.glsl", "point_inclusion.glsl", "quaternion.glsl" };

	for (int i = SS_NONE; i < selection_shape::NUM_OF_SHAPES;++i) {
		//add new shader program
		if (i != SS_NONE) {
			cgv::render::shader_program& label_prog = labeling_tool_shaders[i];
			//create shader program
			label_prog.create(ctx);
			for (auto& dep : dependencies) {
				attach_and_check(label_prog, dep);
				labeling_tool_shaders_files[i].insert(dep);
			}

			// add selector_code to program
			add_selector_code_to(ctx, label_prog, i, &labeling_tool_shaders_files[i]);
			label_prog.link(ctx);
		}
		//pushing tool prog
		if (pushing_shapes.find((selection_shape)i) != pushing_shapes.end()) {
			cgv::render::shader_program& push_prog = pushing_tool_shaders[i];
			push_prog.create(ctx);
			push_prog.attach_file(ctx, "point_pushing_tool.glcs", cgv::render::ST_COMPUTE);
			shaderCheckError(push_prog, "point_pushing_tool.glcs");
			push_prog.attach_file(ctx, "point_cloud_chunk.glsl", cgv::render::ST_COMPUTE);
			shaderCheckError(push_prog, "point_cloud_chunk.glsl");
			push_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
			push_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
			add_selector_code_to(ctx, push_prog, i);
			push_prog.link(ctx);
		}
		
		//clod shader
		if (map_of_shape_postfixes.find((selection_shape)i) != map_of_shape_postfixes.end()) {
			cgv::render::shader_program& draw_prog_1 = get_clod_render_shader(ctx, (selection_shape)i, light_model::LM_NONE);
			std::stringstream fn1;
			fn1 << "clod_point_labels_" << map_of_shape_postfixes.at(i) << ".glpr";
			draw_prog_1.build_program(ctx, fn1.str(), true);

			cgv::render::shader_program& draw_prog_2 = get_clod_render_shader(ctx, (selection_shape)i, light_model::LM_LOCAL);
			std::stringstream fn2;
			fn2 << "clod_point_labels_lights_" << map_of_shape_postfixes.at(i) << ".glpr";
			draw_prog_2.build_program(ctx, fn2.str(), true);
		}
	}
	is_initialized_p = true;
	return true;
}

void label_shader_manager::clear(cgv::render::context& ctx)
{
	//destruct shader programs
	for (auto& prog : labeling_tool_shaders) {
		if (prog.is_created())
			prog.destruct(ctx);
	}
	for (auto& prog : pushing_tool_shaders) {
		if (prog.is_created())
			prog.destruct(ctx);
	}
	for (auto& prog : counting_probe_shaders) {
		if (prog.is_created())
			prog.destruct(ctx);
	}
	is_initialized_p = false;
}

cgv::render::shader_program& label_shader_manager::get_labeling_tool(const selection_shape& shape)
{
	return labeling_tool_shaders.at(shape);
}

cgv::render::shader_program& label_shader_manager::get_pushing_tool(const selection_shape& shape)
{
	return pushing_tool_shaders.at(shape);
}

cgv::render::shader_program& label_shader_manager::get_counting_tool(cgv::render::context& ctx, const selection_shape& shape)
{
	if (counting_probe_shaders[(unsigned)shape].is_created()) {
		return counting_probe_shaders[(unsigned)shape];
	}
	//init program
	cgv::render::shader_program& count_prog = counting_probe_shaders[(unsigned)shape];
	count_prog.create(ctx);
	count_prog.attach_file(ctx, "point_counting_probe.glcs", cgv::render::ST_COMPUTE);
	shaderCheckError(count_prog, "point_counting_probe.glcs");
	add_selector_code_to(ctx, count_prog, (unsigned)shape);
	count_prog.link(ctx);
	return count_prog;
}

bool label_shader_manager::has_pushing_tool(const selection_shape shape) const
{
	return pushing_shapes.find(shape) != pushing_shapes.end();
}

cgv::render::shader_program& label_shader_manager::get_point_collector(cgv::render::context& ctx)
{
	if (!collection_prog.is_created()){
		collection_prog.create(ctx);
		collection_prog.attach_file(ctx, "point_collector.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(collection_prog, "point_collector.glcs");
		collection_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		collection_prog.link(ctx);
	}
	return collection_prog;
}

cgv::render::shader_program& label_shader_manager::get_clod_render_shader(cgv::render::context& ctx, const selection_shape& shape, const light_model lights)
{
	int index = shape * light_model::NUM_OF_LIGHT_MODELS + lights;
	return clod_render_shaders[index];
}

bool label_shader_manager::is_initialized() const
{
	return is_initialized_p;
}

void label_shader_manager::add_selector_code_to(cgv::render::context& ctx, cgv::render::shader_program& prog, int selection_shape_id, std::unordered_set<std::string>* prog_files) const
{
	auto& selector_code = map_of_selectors.at(selection_shape_id);
	for (auto& file : selector_code) {
		if (prog_files == nullptr || prog_files->find(file) == prog_files->end()) {
			prog.attach_file(ctx, file, cgv::render::ST_COMPUTE);
			if (prog_files)
				prog_files->insert(file);
		}
	}
}




label_shader_manager& ref_label_shader_manager(cgv::render::context& ctx, int ref_count_change)
{
	static int ref_count = 0;
	static label_shader_manager lsm;
	lsm.manage_singleton(ctx, ref_count, ref_count_change);
	return lsm;
}
