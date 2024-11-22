#include "point_cloud_server.h"

#include <cgv/render/shader_program.h>
#include <cgv/math/ftransform.h>
#include <cgv/render/shader_program.h>

namespace {

	static cgv::pointcloud::utility::WorkerPool pool(std::thread::hardware_concurrency() - 1);

	template<typename Point>
	void generate_lods_poisson(std::vector<Point>& input_buffer_data)
	{
		static constexpr int mean = 8;
		bool run_parralel = (input_buffer_data.size() > 10'000);

		if (run_parralel) {

			struct Task {
				Point* start;
				int num_points;
			};
			struct Tasks {
				std::atomic_int next_task = 0;
				std::vector<Task> task;
			} tasks;

			int64_t points_distributed = 0;
			int64_t points_total = input_buffer_data.size();
			constexpr int64_t batch_size = 500000;

			while (points_distributed < points_total) {
				int64_t batch = std::min(batch_size, points_total - points_distributed);
				tasks.task.push_back({ &input_buffer_data[points_distributed],(int)batch });
				points_distributed += batch;
			}

			pool.run([&input_buffer_data, &tasks](int thread_id) {
				std::poisson_distribution<int> dist(mean);
				std::random_device rdev;

				while (true) {
					//fetch task
					int tid = tasks.next_task.fetch_add(1, std::memory_order_relaxed);
					if (tid < tasks.task.size()) {
						Task& task = tasks.task[tid];

						Point* end = task.start + task.num_points;
						for (Point* p = task.start; p < end; p++) {
							p->level() = std::min(2 * mean, std::max(0, mean - abs(dist(rdev) - mean)));
						}
					}
					else {
						return;
					}
				}
				});
		}
		else {
			std::poisson_distribution<int> dist(8);
			std::random_device rdev;

			for (auto& v : input_buffer_data) {
				v.level() = std::min(2 * mean, std::max(0, mean - abs(dist(rdev) - mean)));
			}
		}
	}


	void set_constraint_uniforms(cgv::render::context& ctx, cgv::render::shader_program& prog, pct::labeling_constraint& constraint) {
		prog.set_uniform(ctx, "constraint_box_min_p", constraint.label_constraint_box.aabb_min_p);
		prog.set_uniform(ctx, "constraint_box_max_p", constraint.label_constraint_box.aabb_max_p);
		prog.set_uniform(ctx, "constraint_box_translation", constraint.label_constraint_box.translation);
		prog.set_uniform(ctx, "constraint_box_rotation", constraint.label_constraint_box.rotation);
		prog.set_uniform(ctx, "label_constraint_bits", constraint.label_constraint_bits);
		prog.set_uniform(ctx, "label_constraint_solid_inverted", constraint.label_constraint_solid_inverted);
	}
}

void pct::point_cloud_server::configure_timers(bool gpu_timers, bool cpu_timers) {
	gpu_timers_enabled = gpu_timers;
	cpu_timers_enabled = cpu_timers;
}

pct::point_cloud_preparation_settings::point_cloud_preparation_settings() :
	lod_mode(LoDMode::OCTREE),
	allow_point_deduplication(true),
	auto_chunk_cube_size(true),
	chunk_cube_size(1.0),
	auto_chunk_max_num_chunks(10)
{};

pct::point_cloud_server& pct::ref_point_cloud_server(cgv::render::context& ctx, int ref_count_change)
{
	static int ref_count = 0;
	static pct::point_cloud_server r;
	r.manage_singelton(ctx, "point_cloud_server", ref_count, ref_count_change);
	return r;
}


void pct::point_cloud_server::manage_singelton(cgv::render::context& ctx, const std::string& name, int& ref_count, int ref_count_change)
{
	switch (ref_count_change) {
	case 1:
		if (ref_count == 0) {
			if (!init(ctx))
				ctx.error(std::string("unable to initialize ") + name + " singelton");
		}
		++ref_count;
		break;
	case 0:
		break;
	case -1:
		if (ref_count == 0)
			ctx.error(std::string("attempt to decrease reference count of ") + name + " singelton below 0");
		else {
			if (--ref_count == 0)
				clear(ctx);
		}
		break;
	default:
		ctx.error(std::string("invalid change reference count outside {-1,0,1} for ") + name + " singelton");
	}
}

void pct::point_cloud_server::swap_history(std::unique_ptr<history>& ptr)
{
	history_ptr.swap(ptr);
}



cgv::mat4 pct::point_cloud_server::get_point_cloud_model_transform()
{
	return cgv::math::translate4(point_cloud_position)
		* cgv::math::rotate4_rpy<float>(point_cloud_rotation)
		* cgv::math::scale4(point_cloud_scale, point_cloud_scale, point_cloud_scale);
}

void pct::point_cloud_server::use_point_cloud(point_cloud& source_pc, const point_cloud_preparation_settings& settings)
{
	chunks_disabled = false;
	chunked_points = chunks<LODPoint>();

	std::vector<indexed_point> source_points(source_pc.get_nr_points());

	rgb default_color(1.0, 0.0, 0.0);
	size_t num_source_points = source_pc.get_nr_points();

	//the octree_lod_generator expects the input points to be an array of structs, so we need to reshape the data
	for (int i = 0; i < num_source_points; ++i) {
		source_points[i].position() = source_pc.pnt(i);
		source_points[i].index = i;
		if (source_pc.has_colors()) {
			source_points[i].color() = source_pc.clr(i);
		}
		else {
			//set a default color if its missing in the source point cloud
			source_points[i].color() = rgb8(default_color);
		}
		if (source_pc.has_lods()) {

			source_points[i].level() = source_pc.lod(i);

		}
	}

	{
		std::vector<indexed_point> indexed_points_with_lod;

		if (source_pc.has_lods()) {
			// skip the lod computation, reuse the attributes from file 
			indexed_points_with_lod.swap(source_points);
		}
		else {
			// have to (re-)compute level of details for each point 
			std::chrono::duration<double> diff_lod;
			auto start_lod_gen = std::chrono::steady_clock::now();
			if (settings.lod_mode == LoDMode::OCTREE) {
				auto& lod_generator = cgv::pointcloud::ref_octree_lod_generator<indexed_point>(0);
				lod_generator.allow_dedup() = settings.allow_point_deduplication; //disable deduplication to preserve the number of points / enable for a faster point cloud
				indexed_points_with_lod = std::move(lod_generator.generate_lods(source_points));
				//free memory of source_points
				source_points.swap(std::vector<indexed_point>());
			}
			else {
				//quick and cheap lod generation by assiging lods randomly (will also look cheap)
				generate_lods_poisson(source_points);
				indexed_points_with_lod.swap(source_points);
			}

			// write back the lods, order is changed , resize if needed
			source_pc.resize_lods();
			for (int i = 0; i < indexed_points_with_lod.size(); ++i) {
				source_pc.lod(indexed_points_with_lod[i].index) = indexed_points_with_lod[i].level();
			}
			auto stop_lod_gen = std::chrono::steady_clock::now();
			diff_lod = stop_lod_gen - start_lod_gen;
			//std::cout << "diff_lod: lod generation: " << diff_lod.count() << std::endl;
		}

		// 
		std::chrono::duration<double> diff_chunk;
		auto start_draw_chunk = std::chrono::steady_clock::now();

		if (settings.auto_chunk_cube_size) {
			chunk_cube_size = compute_chunk_size(source_pc, settings);
		}

		//split point data into chunks, convert indexed_point to LODPoint
		chunked_points = chunks<LODPoint>(indexed_points_with_lod, chunk_cube_size);

		//apply permutation defined by chunking to restore original point indices
		struct index_permute_adapter {
			indexed_point* points;
			inline const unsigned operator[](unsigned i) const {
				return points[i].index;
			}
		} ipa;
		ipa.points = indexed_points_with_lod.data();
		chunked_points.permute_point_ids(ipa);

		auto stop_draw_chunk = std::chrono::steady_clock::now();
		diff_chunk = stop_draw_chunk - start_draw_chunk;
		std::cout << "chunk size: " << settings.auto_chunk_max_num_chunks << std::endl;
		//std::cout << "diff_chunk: chunks: " << diff_chunk.count() << std::endl;
	}
	//read other point attribute arrays, in this case the point labels.
	size_t num_points = chunked_points.num_points();

	//add labels to chunked point cloud
	//label_attribute_id = chunked_points.add_point_attribute("label", point_labels.data(), point_labels_layout_pos, point_labels.size(), sizeof(GLint));
	if (source_pc.has_labels()) {
		std::vector<GLint> labels = std::vector<GLint>(source_pc.get_nr_points());
		for (int i = 0; i < source_pc.get_nr_points(); ++i)
			labels[i] = make_label(source_pc.label(i), point_label_group::VISIBLE);
		label_attribute_id = chunked_points.add_point_attribute("label", labels.data(), point_labels_layout_pos, source_pc.get_nr_points(), sizeof(GLint));
	}
	else {
		std::vector<GLint> labels;
		labels.resize(source_pc.get_nr_points(), (GLint)point_label_group::VISIBLE);
		label_attribute_id = chunked_points.add_point_attribute("label", labels.data(), point_labels_layout_pos, source_pc.get_nr_points(), sizeof(GLint));
	}
	//if the source point cloud has normals, add normal as attribution
	if (source_pc.has_normals()) {
		std::vector<vec4> normals(source_pc.get_nr_points());
		for (int i = 0; i < normals.size(); ++i) {
			memcpy(&normals[i], &source_pc.nml(i), sizeof(vec3));
		}
		normal_attribute_id = chunked_points.add_point_attribute("normal", normals.data(), 0, normals.size(), sizeof(vec4));
	}

	//initialize chunk buffers
	chunked_points.init_buffers(*ctx_ptr);
	
	//reserve some space in case points must be relocated to a new chunk(e.g. pushed with the point pushing tool)
	chunked_points.swapping_chunk().resize_reserved(std::max<size_t>(32000, chunked_points.num_points() / 20));
	chunked_points.swapping_chunk().upload_to_buffers();

	//copy transform
	ref_point_cloud_position() = source_pc.ref_point_cloud_position();
	ref_point_cloud_rotation() = source_pc.ref_point_cloud_rotation();
	ref_point_cloud_scale() = source_pc.ref_point_cloud_scale();

	std::cout << "generated chunks with " << chunked_points.num_points() << " points from " << source_pc.get_nr_points() << " points\n";
}

float pct::point_cloud_server::compute_chunk_size(point_cloud& source_pc, const point_cloud_preparation_settings& settings)
{
	vec3 pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	for (int i = 0; i < source_pc.get_nr_points(); i++) {
		pmin.x() = std::min(source_pc.pnt(i).x(), pmin.x());
		pmin.y() = std::min(source_pc.pnt(i).y(), pmin.y());
		pmin.z() = std::min(source_pc.pnt(i).z(), pmin.z());

		pmax.x() = std::max(source_pc.pnt(i).x(), pmax.x());
		pmax.y() = std::max(source_pc.pnt(i).y(), pmax.y());
		pmax.z() = std::max(source_pc.pnt(i).z(), pmax.z());
	}
	vec3 ext = (pmax - pmin);
	float max_element = static_cast<float>(*std::max_element(ext.begin(), ext.end()));

	return  max_element / settings.auto_chunk_max_num_chunks;
}

void pct::point_cloud_server::rechunk_point_cloud(float chunk_size)
{
	if (chunk_size <= 0.f) {
		throw std::runtime_error("chunk size is negativ ot zero");
	}

	//get data from buffers and rechunk
	chunked_points.download_buffers();
	chunked_points.rechunk(chunk_size);
	chunk_cube_size = chunk_size;
}

void pct::point_cloud_server::compute_lod_adding_points(point_cloud& source_pc, const point_cloud_preparation_settings& settings)
{
	std::vector<indexed_point> source_points(source_pc.get_nr_points());

	rgb default_color(1.0, 0.0, 0.0);
	size_t num_source_points = source_pc.get_nr_points();

	//the octree_lod_generator expects the input points to be an array of structs, so we need to reshape the data
	for (int i = 0; i < num_source_points; ++i) {
		source_points[i].position() = source_pc.pnt(i);
		source_points[i].index = i;
		if (source_pc.has_colors()) {
			source_points[i].color() = source_pc.clr(i);
		}
		else {
			//set a default color if its missing in the source point cloud
			source_points[i].color() = rgb8(default_color);
		}
		if (source_pc.has_lods()) {

			source_points[i].level() = source_pc.lod(i);

		}
		else {
			source_points[i].level() = 1;
		}
	}

	std::vector<indexed_point> indexed_points_with_lod;
	// have to (re-)compute level of details for each point 
	/*if (settings.lod_mode == LoDMode::OCTREE) {
		auto& lod_generator = cgv::pointcloud::ref_octree_lod_generator<indexed_point>(0);
		lod_generator.allow_dedup() = settings.allow_point_deduplication; //disable deduplication to preserve the number of points / enable for a faster point cloud
		indexed_points_with_lod = std::move(lod_generator.generate_lods(source_points));
		//free memory of source_points
		source_points.swap(std::vector<indexed_point>());
	}
	else {
		//quick and cheap lod generation by assiging lods randomly (will also look cheap)
		generate_lods_poisson(source_points);
		indexed_points_with_lod.swap(source_points);
	}*/
	indexed_points_with_lod.swap(source_points);
	// write back the lods, order is changed , resize if needed
	source_pc.resize_lods();
	for (int i = 0; i < indexed_points_with_lod.size(); ++i) {
		source_pc.lod(indexed_points_with_lod[i].index) = indexed_points_with_lod[i].level();
	}
	std::cout << "finished lod" << std::endl;
}


std::vector<cgv::rgba> pct::point_cloud_server::make_lod_to_color_lut(const float lod_min_level_hue, const float lod_max_level_hue)
{
	std::vector<rgba> col_lut;
	// generate a lod color lookup table for the show lods feature
	int max_lod = 0;
	int num_points = 0;
	// to find the max lod
	for (auto iter = chunked_points.points_begin(); iter != chunked_points.points_end(); ++iter) {
		max_lod = std::max((int)iter->level(), max_lod);
		++num_points;
	}

	for (int lod = 0; lod <= max_lod; ++lod) {
		cgv::media::color<float, cgv::media::HLS> col;
		col.L() = 0.5f;
		col.S() = 1.f;
		col.H() = lod_min_level_hue + (lod_max_level_hue - lod_min_level_hue) * ((float)lod / (float)max_lod);
		col_lut.push_back(col);
	}

	return col_lut;
}

pct::point_cloud_server::point_cloud_server() : 
	ctx_ptr(nullptr),
	chunks_disabled(false),
	last_id(0),
	gp_results_buffer(0),
	point_cloud_scale(1.0),
	gpu_timers_enabled(false),
	cpu_timers_enabled(false)
{
	cgv::pointcloud::ref_octree_lod_generator<indexed_point>(1);
}

pct::point_cloud_server::~point_cloud_server()
{
	cgv::pointcloud::ref_octree_lod_generator<indexed_point>(-1);
}

pointcloud_transformation_state  pct::point_cloud_server::get_transformation_state() {
	pointcloud_transformation_state state;
	state.position = ref_point_cloud_position();
	state.rotation = ref_point_cloud_rotation();
	state.scale = ref_point_cloud_scale();
	return state;
}

void  pct::point_cloud_server::restore_transformation_state(const pointcloud_transformation_state& state) {
	ref_point_cloud_position() = state.position;
	ref_point_cloud_rotation() = state.rotation;
	ref_point_cloud_scale() = state.scale;
}

bool pct::point_cloud_server::raycast_points(const vec3& dir, const vec3& origin, const float radius, float& res_t, vec3& res_p)
{
	auto num_chunks = chunked_points.num_chunks();
	std::vector<int> chunk_results(chunked_points.num_chunks(), -1); //each element stores the index of a point inside the chunk

	struct results {
		GLuint global_point_ix = -1; //index of point with min t
		GLfloat t = std::numeric_limits<GLfloat>::infinity(); // calculated parameter t
	};

	struct buffer_data {
		GLuint result_buffers[2];
		std::vector<results> work_group_results;
		GLuint best_group_ix = -1;
	};
	// storage for the parameter t where |t*ray_dir + ray_origin - best_p| < radius and best_t is also the smallest positiv float for that the ray intersects with the sphere around one of the point cloud's points
	float best_t = std::numeric_limits<GLfloat>::infinity();
	vec3 best_p = vec3(0);
	
	vec4 ray_dir = dir.lift();
	vec4 ray_origin = origin.lift();

	const mat4& model = get_point_cloud_model_transform();
	mat4 t_model = cgv::math::transpose(model);
	mat4 inv_model = cgv::math::inv(model);

	std::unordered_map<chunk<LODPoint>*, buffer_data> buffers;
	point_raycast_prog.set_uniform(*ctx_ptr, 1, radius);
	point_raycast_prog.set_uniform(*ctx_ptr, 2, ray_dir);
	point_raycast_prog.set_uniform(*ctx_ptr, 3, ray_origin);
	point_raycast_prog.set_uniform(*ctx_ptr, 4, model);

	point_raycast_prog.enable(*ctx_ptr);

	//find chunks passed by the ray
	std::vector<std::pair<ivec3, chunk<LODPoint>*>> intersecting_chunks;

	vec4 chunk_cs_ray_origin = inv_model * origin.lift();
	vec4 chunk_cs_ray_direction = dir.lift(); chunk_cs_ray_direction.w() = 0;
	chunk_cs_ray_direction = t_model * chunk_cs_ray_direction;
	chunked_points.intersect_ray(chunk_cs_ray_origin, chunk_cs_ray_direction, intersecting_chunks);

	//highlighted_chunks = intersecting_chunks; 	//only for debugging purposes here 

	//find nearest point sphere intersected by the ray per chunk
	for (auto& chunk_index_pair : intersecting_chunks) {
		static constexpr unsigned work_group_size = 128;

		auto& ch = *chunk_index_pair.second;
		auto& buf_data = buffers[&ch];

		glCreateBuffers(2, buf_data.result_buffers);

		buf_data.work_group_results.resize(ch.size());
		glNamedBufferData(buf_data.result_buffers[0], sizeof(GLuint), &buf_data.best_group_ix, GL_STATIC_DRAW);
		glNamedBufferData(buf_data.result_buffers[1], sizeof(results) * buf_data.work_group_results.size(), buf_data.work_group_results.data(), GL_STATIC_DRAW);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ch.point_buffer());
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, buf_data.result_buffers[0]);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 20, buf_data.result_buffers[1]);

		point_raycast_prog.set_uniform(*ctx_ptr, 22, (GLint)ch.size());

		glDispatchCompute((ch.size() / 128) + 1, 1, 1);
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	point_raycast_prog.disable(*ctx_ptr);

	for (auto& chunk_index_pair : intersecting_chunks) {
		auto& ch = *chunk_index_pair.second;
		//get results
		auto& ch_buf = buffers[&ch];
		results* res2 = static_cast<results*>(glMapNamedBufferRange(
			ch_buf.result_buffers[1], 0, sizeof(results) * ch_buf.work_group_results.size(), GL_MAP_READ_BIT));
		GLuint* res1 = static_cast<GLuint*>(glMapNamedBufferRange(
			ch_buf.result_buffers[0], 0, sizeof(GLuint), GL_MAP_READ_BIT));
		//glCheckError();
		ch_buf.best_group_ix = *res1;
		//find the best result
		if (ch_buf.best_group_ix != -1) {
			results best_chunk_result = res2[ch_buf.best_group_ix];
			if (best_chunk_result.t < best_t && best_chunk_result.t >= 0.f) {
				best_t = best_chunk_result.t;
				best_p = ch.point_at(best_chunk_result.global_point_ix).position();
			}

		}

		glUnmapNamedBuffer(ch_buf.result_buffers[0]);
		glUnmapNamedBuffer(ch_buf.result_buffers[1]);

		glDeleteBuffers(2, ch_buf.result_buffers);
	}

	res_p = model * best_p.lift();
	res_t = best_t;

	return best_t != std::numeric_limits<GLfloat>::infinity();
}

std::vector<GLuint> pct::point_cloud_server::count_points_in_sphere(const vec3& position, const float radius, const bool use_chunks_for_labeling)
{
	std::vector<GLuint> results(256, 0); // return value

//layout position constants
	constexpr int points_pos = 1, results_pos = 2;
	//get the shader program for sphere selection
	auto& label_shaders = ref_label_shader_manager(*ctx_ptr);
	cgv::render::shader_program& tool_prog = label_shaders.get_counting_tool(*ctx_ptr, SS_SPHERE);

	float scaled_radius = radius / ref_point_cloud_scale();
	dmat4 concat_trans = this->get_point_cloud_model_transform();

	thread_local std::vector<std::pair<const ivec3, chunk<LODPoint>*>> chunks; //make this static to keep memory allocated
	chunks.clear();

	auto& chunked_points = ref_chunks();

	if (use_chunks_for_labeling) {
		// find affected chunks using model space
		vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
		vec3 position_in_model_space_vec3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
		chunked_points.intersect_sphere(position_in_model_space_vec3, scaled_radius, chunks); // points, bboxes are not transformed 
#ifdef DEBUG
		std::cout << "scaled_radius: " << scaled_radius << std::endl;
		std::cout << "number of chunks intersected: " << chunks.size() << std::endl;
#endif // DEBUG
	}
	else {
		for (auto& kv_pair : chunked_points) {
			chunks.emplace_back(kv_pair.first, kv_pair.second.get());
		}
	}

	// pass to shader, model will be transformed, radius won't change
	vec4 sphere = dvec4(position.lift()); sphere.w() = radius;
	//std::cout << "sphere " << sphere << std::endl;
	mat4 float_trans_matrix = concat_trans;
	tool_prog.set_uniform(*ctx_ptr, "selection_enclosing_sphere", sphere, true);
	tool_prog.set_uniform(*ctx_ptr, "model_transform", float_trans_matrix, true);

	tool_prog.enable(*ctx_ptr);

	glNamedBufferData(gp_results_buffer, sizeof(GLuint) * 256, results.data(), GL_DYNAMIC_READ);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, results_pos, gp_results_buffer);

	for (auto& chunk_index_pair : chunks) {
		auto& ch = *(chunk_index_pair.second);
		if (ch.size() > 0) {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			tool_prog.set_uniform(*ctx_ptr, "batch_size", (GLint)ch.size(), true);
			tool_prog.set_uniform(*ctx_ptr, "batch_offset", (GLint)0, true);
			// run computation
			glDispatchCompute((ch.size() / 256) + 1, 1, 1);
		}
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	tool_prog.disable(*ctx_ptr);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

	GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		gp_results_buffer, 0, sizeof(GLuint) * 256, GL_MAP_READ_BIT));
	if (results_ptr) {
		std::memcpy(results.data(), results_ptr, results.size() * sizeof(GLuint));
		glUnmapNamedBuffer(gp_results_buffer);
	}
	else {
		std::cerr << "pointcloud_cleaning_tool::count_points_in_sphere(): can't read results buffer!\n";
	}
	return results;
}


void pct::point_cloud_server::label_points_in_sphere(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, vec3 position, const float radius, const point_label_operation op) {
	//layout position constants
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4;

	auto& chunked_points = ref_chunks();

	if (label_attribute_id == -1) {
		return;
	}

	//get the shader program for sphere selection
	auto& label_shaders = ref_label_shader_manager(*ctx_ptr);
	cgv::render::shader_program& labeling_tool_prog = label_shaders.get_labeling_tool(SS_SPHERE);

	float scaled_radius = radius / (ref_point_cloud_scale());
	dmat4 concat_trans = get_point_cloud_model_transform();


	thread_local std::vector<std::pair<const ivec3, chunk<LODPoint>*>> chunks; //make this static to keep memory allocated
	chunks.clear();

	if (interaction_settings.use_chunks) {
		// find affected chunks using model space
		vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
		vec3 position_in_model_space_vec3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
		chunked_points.intersect_sphere(position_in_model_space_vec3, scaled_radius, chunks); // points, bboxes are not transformed 
#ifdef DEBUG
		std::cout << "scaled_radius: " << scaled_radius << std::endl;
		std::cout << "number of chunks intersected: " << chunks.size() << std::endl;
#endif // DEBUG
	}
	else {
		for (auto& kv_pair : chunked_points) {
			chunks.emplace_back(kv_pair.first, kv_pair.second.get());
		}
	}


	// pass to labeling shader, model will be transformed, radius won't change
	vec4 sphere = dvec4(position.lift()); sphere.w() = radius;
#ifdef DEBUG
	std::cout << "sphere " << sphere << std::endl;
#endif
	mat4 float_trans_matrix = concat_trans;
	labeling_tool_prog.set_uniform(*ctx_ptr, "selection_enclosing_sphere", sphere, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_label", label, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "model_transform", float_trans_matrix, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_groups", point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "exclude_point_groups", exclude_point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "operation", (GLint)op);

	set_constraint_uniforms(*ctx_ptr, labeling_tool_prog, active_constraint);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	if (history_ptr && interaction_settings.enable_history)
		history_ptr->bind(*ctx_ptr);

	labeling_tool_prog.enable(*ctx_ptr);
	// enable gpu timer for labeling operation
	if (gpu_timers_enabled)
		labeling_stopwatch.start_gpu_timer();

	for (auto& chunk_index_pair : chunks) {
		auto& ch = *(chunk_index_pair.second);
		//std::cout << ch.id_buffer() << std::endl;
		if (ch.size() > 0) {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_size", (GLint)ch.size(), true);
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_offset", (GLint)0, true);
			// run computation
			glDispatchCompute((ch.size() / 128) + 1, 1, 1);
		}
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);
	
	if (gpu_timers_enabled)
		labeling_stopwatch.stop_gpu_timer();

	labeling_tool_prog.disable(*ctx_ptr);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pct::point_cloud_server::label_points_in_box(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, const box3& box, const vec3& position, const quat& box_orientation, const point_label_operation operation) {
	if (label_attribute_id == -1) {
		return;
	}
	//
	//float scaled_radius = radius / (source_pc.ref_point_cloud_scale() * auto_positioning_scale);
	dmat4 concat_trans = get_point_cloud_model_transform();

	auto& label_shaders = ref_label_shader_manager(*ctx_ptr);
	cgv::render::shader_program& labeling_tool_prog = label_shaders.get_labeling_tool(SS_CUBOID);

	// find affected chunks with transformed position and scale, model not changed  
	thread_local std::vector<std::pair<const ivec3, chunk<LODPoint>*>> chunks;
	chunks.clear();

	auto& chunked_points = ref_chunks();

	vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
	vec3 position_in_model_space3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());

	if (interaction_settings.use_chunks) {
		float scaled_radius = length(box.get_extent()) / (ref_point_cloud_scale());
		float box_scale_factor = 1.f / (ref_point_cloud_scale());
		box3 scaled_box = box3(box.get_min_pnt() * box_scale_factor, box.get_max_pnt() * box_scale_factor);
		chunked_points.intersect_sphere(position_in_model_space3, scaled_radius, chunks); //use sphere intersection for now
	}
	else {
		for (auto& kv_pair : chunked_points) {
			chunks.emplace_back(kv_pair.first, kv_pair.second.get());
		}
	}

	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4;

	mat4 float_trans_matrix = concat_trans;

	labeling_tool_prog.set_uniform(*ctx_ptr, "selection_box_rotation", box_orientation, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "selection_box_translation", position.lift(), true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "aabb_max_p", box.get_max_pnt(), true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "aabb_min_p", box.get_min_pnt(), true);

	labeling_tool_prog.set_uniform(*ctx_ptr, "point_label", label, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "model_transform", float_trans_matrix, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_groups", point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "exclude_point_groups", exclude_point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "operation", (int)operation);

	set_constraint_uniforms(*ctx_ptr, labeling_tool_prog, active_constraint);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	if (history_ptr && interaction_settings.enable_history)
		history_ptr->bind(*ctx_ptr);

	labeling_tool_prog.enable(*ctx_ptr);

	labeling_stopwatch.start_gpu_timer();

	for (auto& index_chunk_pair : chunks) {
		auto& ch = *index_chunk_pair.second;
		if (ch.size() > 0) {

			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_size", (GLint)ch.size(), true);
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_offset", (GLint)0, true);

			// run computation
			glDispatchCompute((ch.size() / 128) + 1, 1, 1);
		}
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	labeling_stopwatch.stop_gpu_timer();

	labeling_tool_prog.disable(*ctx_ptr);
	//if (enable_label_history)
	//	label_history.add_rollback_operation();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pct::point_cloud_server::label_points_by_clipping(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, vec3 position, mat3 plane_ori, const bool invert, const point_label_operation operation) {
	//buffer layout position constants 
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4;
	//
	if (label_attribute_id == -1) {
		return;
	}

	auto& label_shaders = ref_label_shader_manager(*ctx_ptr);
	cgv::render::shader_program& labeling_tool_prog = label_shaders.get_labeling_tool(SS_PLANE);
	//
	cgv::dmat4 concat_trans = get_point_cloud_model_transform();

	// find affected chunks with transformed position and scale
	std::vector<std::pair<const ivec3, chunk<LODPoint>*>> chunks;
	vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
	mat4 concat_trans_f = concat_trans;

	vec3 position_in_model_space3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
	vec3 plane_normal3 = plane_ori * vec3(((invert) ? -1.f : 1.f), 0.f, 0.f);
	vec4 plane_normal = plane_normal3.lift(); plane_normal.w() = 0;
	vec4 plane_normal_in_model_space = inv(transpose((mat4)inv(concat_trans))) * plane_normal;
	vec3 plane_normal_in_model_space3 = vec3(plane_normal_in_model_space.x(), plane_normal_in_model_space.y(), plane_normal_in_model_space.z());

	auto& chunked_points = ref_chunks();

	if (interaction_settings.use_chunks)
		chunked_points.intersect_clip_plane(plane_normal_in_model_space3, position_in_model_space3.length(), chunks); // points, bboxes are not transformed 
	else
		chunked_points.get_all(chunks);


	labeling_tool_prog.set_uniform(*ctx_ptr, "point_label", label, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "model_transform", concat_trans_f, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_groups", point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "exclude_point_groups", exclude_point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	//give plane in world space
	labeling_tool_prog.set_uniform(*ctx_ptr, "selection_plane_origin", position.lift(), true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "selection_plane_normal", plane_normal, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "operation", (GLint)operation);

	set_constraint_uniforms(*ctx_ptr, labeling_tool_prog, active_constraint);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	if (history_ptr && interaction_settings.enable_history)
		history_ptr->bind(*ctx_ptr);

	labeling_tool_prog.enable(*ctx_ptr);

	labeling_stopwatch.start_gpu_timer();

	for (auto& index_chunk_pair : chunks) {
		auto& ch = *index_chunk_pair.second;
		if (ch.size() > 0) {

			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_size", (GLint)ch.size(), true);
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_offset", (GLint)0, true);

			// run computation
			glDispatchCompute((ch.size() / 128) + 1, 1, 1);
		}
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	labeling_stopwatch.stop_gpu_timer();

	labeling_tool_prog.disable(*ctx_ptr);
	//if (enable_label_history)
	//	label_history.add_rollback_operation();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pct::point_cloud_server::label_all_points(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, const point_label_operation operation)
{
	//buffer layout position constants 
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6, point_id_pos = 4;
	//
	if (label_attribute_id == -1) {
		return;
	}

	auto& label_shaders = ref_label_shader_manager(*ctx_ptr);
	cgv::render::shader_program& labeling_tool_prog = label_shaders.get_labeling_tool(SS_ALL);
	//
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_label", label, true);
	labeling_tool_prog.set_uniform(*ctx_ptr, "point_groups", point_group_mask & (GLint)point_label_group::GROUP_MASK);
	labeling_tool_prog.set_uniform(*ctx_ptr, "exclude_point_groups", exclude_point_group_mask & (int32_t)point_label_group::GROUP_MASK);
	//give plane in world space
	labeling_tool_prog.set_uniform(*ctx_ptr, "operation", (GLint)operation);

	labeling_tool_prog.set_uniform(*ctx_ptr, "enable_label_history", interaction_settings.enable_history);

	set_constraint_uniforms(*ctx_ptr, labeling_tool_prog, active_constraint);

	auto& chunked_points = ref_chunks();

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());

	if (history_ptr && interaction_settings.enable_history)
		history_ptr->bind(*ctx_ptr);

	labeling_tool_prog.enable(*ctx_ptr);

	labeling_stopwatch.start_gpu_timer();

	for (auto& index_chunk_pair : chunked_points) {
		auto& ch = *index_chunk_pair.second;
		if (ch.size() > 0) {

			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, point_id_pos, ch.id_buffer());
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, ch.point_buffer());
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_size", (GLint)ch.size(), true);
			labeling_tool_prog.set_uniform(*ctx_ptr, "batch_offset", (GLint)0, true);

			// run computation
			glDispatchCompute((ch.size() / 128) + 1, 1, 1);
		}
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	labeling_stopwatch.stop_gpu_timer();

	labeling_tool_prog.disable(*ctx_ptr);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pct::point_cloud_server::label_selected_points(const GLint new_label, const GLint expected_label, const int32_t point_group_mask, const GLuint points, const unsigned num_points)
{

	static constexpr int
		new_point_label_loc = 10,
		point_group_mask_loc = 11,
		point_exclude_group_mask_loc = 13,
		batch_offset_loc = 21,
		batch_size_loc = 22,
		selected_point_label_loc = 12;

	constexpr int labels_pos = 6;
	constexpr int rollback_pos = 7;

	if (num_points == 0) {
		return;
	}

	GLint batch_offset = 0;
	GLint batch_size = num_points;

	selection_relabel_prog.set_uniform(*ctx_ptr, new_point_label_loc, new_label);
	selection_relabel_prog.set_uniform(*ctx_ptr, point_group_mask_loc, point_group_mask);
	selection_relabel_prog.set_uniform(*ctx_ptr, selected_point_label_loc, expected_label);
	selection_relabel_prog.set_uniform(*ctx_ptr, batch_size_loc, batch_size);
	selection_relabel_prog.set_uniform(*ctx_ptr, batch_offset_loc, batch_offset);

	set_constraint_uniforms(*ctx_ptr, selection_relabel_prog, active_constraint);

	auto& chunked_points = ref_chunks();

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, chunked_points.get_attribute(label_attribute_id).buffer_name());
	if (history_ptr && interaction_settings.enable_history)
		history_ptr->bind(*ctx_ptr);

	if (selection_relabel_prog.enable(*ctx_ptr)) {
		// run computation
		glDispatchCompute((num_points / 128) + 1, 1, 1);
		// synchronize
		glMemoryBarrier(GL_ALL_BARRIER_BITS);
		selection_relabel_prog.disable(*ctx_ptr);
	}

	//write operation to history 
	if (history_ptr && interaction_settings.enable_history)
		history_ptr->add_rollback_operation();

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pct::point_cloud_server::set_constraint_box(GLint or_label, box3 box, vec3 translation, quat rotation, bool box_inverted)
{
	active_constraint.label_constraint_bits = or_label;
	active_constraint.label_constraint_box.aabb_max_p = box.get_max_pnt().lift();
	active_constraint.label_constraint_box.aabb_min_p = box.get_min_pnt().lift();
	active_constraint.label_constraint_box.rotation = rotation;
	active_constraint.label_constraint_box.translation = vec4(translation.x(),translation.y(),translation.z(),0.f);
	active_constraint.label_constraint_solid_inverted = box_inverted;
}

void pct::point_cloud_server::clear_constraints()
{
	active_constraint.label_constraint_bits = 0;
}

bool pct::point_cloud_server::poll_labeling_time(GLuint64& time) {
	return labeling_stopwatch.poll_gpu_timer(time);
}


void pct::point_cloud_server::fuse_point_cloud(point_cloud& source, mat4& model_matrix, bool copy_labels)
{
	static constexpr int default_label = 0; //assigned to new points

	if (!source.has_labels())
		copy_labels = false;

	//merge pointclouds
	auto& pc = source;
	size_t nr_points = pc.get_nr_points();
	cgv::mat4 transform = inv(get_point_cloud_model_transform()) * model_matrix;

	auto& chunked_points = ref_chunks();
	//download all chunked points from GPU
	chunked_points.download_buffers();

	//create attributes
	std::vector<generic_point_attribute> attributes;
	std::vector<int> attribute_ids;
	int attr_normals = chunked_points.find_attribute_id_by_name("normal");
	int attr_labels = chunked_points.find_attribute_id_by_name("label");
	assert(attr_labels != -1);
	//labels
	attribute_ids.emplace_back(attr_labels);
	attributes.emplace_back(sizeof(GLint));
	//generate new labels for the points
	std::vector<GLint> new_labels(pc.get_nr_points(), (GLint)make_label(default_label, point_label_group::VISIBLE));

	if (copy_labels) {
		std::memcpy(new_labels.data(), &pc.label(0), new_labels.size() * sizeof(GLint));
	}

	attributes[0].load(new_labels.data(), new_labels.size());
	//normals
	if (attr_normals != -1) {
		attribute_ids.emplace_back(attr_normals);
		attributes.emplace_back(sizeof(vec3));
		if (!pc.has_normals())
			pc.create_normals();
		//copy from point cloud
		attributes.back().load(&pc.nml(0), pc.get_nr_points());
	}
	// add points
	{
		memory_mapped_history mapped_history = history_ptr->make_mapping();
		for (int i = 0; i < nr_points; ++i) {
			LODPoint pnt;
			pnt.position() = transform * pc.pnt(i).lift();
			pnt.color() = pc.clr(i);
			label_operation op;
			op.index = chunked_points.add_point(pnt, attributes.data(), attribute_ids.data(), attributes.size(), i);
			op.label = make_label(0, point_label_group::DELETED);
			mapped_history.add_label_operation(op);
		}
	}
	history_ptr->add_rollback_operation();
	//find modified chunks
	for (auto& iv_pair : chunked_points) {
		auto& ch = iv_pair.second;
		if (ch->host_data_changed()) {
			//extend points
			std::vector<indexed_point> pnts;
			pnts.resize(ch->size());
			for (int i = 0; i < ch->size(); ++i) {
				indexed_point p;
				auto& cp = ch->point_at(i);
				p.position() = cp.position();
				p.color() = cp.color();
				p.level() = 0;
				p.index = ch->id_at(i);
				pnts[i] = p;
			}
			//recompute lods
			auto& lod_generator = cgv::pointcloud::octree_lod_generator<indexed_point>();
			std::vector<indexed_point> new_pnts = std::move(lod_generator.generate_lods(pnts));
			//rebuild chunk
			ch->resize(new_pnts.size());
			for (int i = 0; i < new_pnts.size(); ++i) {
				LODPoint p;
				p.position() = new_pnts[i].position();
				p.color() = new_pnts[i].color();
				p.level() = new_pnts[i].level();
				ch->point_at(i) = p;
				ch->id_at(i) = new_pnts[i].index;
			}
		}
	}
	chunked_points.upload_changes_to_buffers();
}

bool pct::point_cloud_server::init(cgv::render::context& ctx)
{
	ctx_ptr = &ctx;

	//create raycast shader
	if (!point_raycast_prog.is_created()) {
		point_raycast_prog.create(ctx);
		point_raycast_prog.attach_file(ctx, "raycast_points.glcs", cgv::render::ST_COMPUTE);
		point_raycast_prog.link(ctx, true);
	}
	
	if (!selection_relabel_prog.is_created()) {
		selection_relabel_prog.create(ctx);
		selection_relabel_prog.attach_file(ctx, "selection_labeler.glcs", cgv::render::ST_COMPUTE);
		//shaderCheckError(selection_relabel_prog, "selection_labeler.glcs");
		selection_relabel_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		//shaderCheckError(selection_relabel_prog, "point_label.glsl");
		selection_relabel_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		//shaderCheckError(selection_relabel_prog, "point_label_history.glsl");
		selection_relabel_prog.link(ctx, true);
	}

	std::array<GLuint,2> gl_buffers;
	glCreateBuffers(2, &gp_results_buffer);
	
	gp_results_buffer = gl_buffers[0];
	box_constraint_buffer = gl_buffers[1];

	ref_label_shader_manager(ctx, 1);

	labeling_stopwatch.init();

	return true;
}

void pct::point_cloud_server::clear(cgv::render::context& ctx)
{
	selection_relabel_prog.destruct(ctx);
	point_raycast_prog.destruct(ctx);

	ref_label_shader_manager(ctx, -1);
	labeling_stopwatch.clear();
	glDeleteBuffers(1, &gp_results_buffer);
	glDeleteBuffers(1, &box_constraint_buffer);
}

pct::point_interaction_settings::point_interaction_settings() :
	use_chunks(true),
	enable_history(true)
{
}
