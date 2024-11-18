#pragma once

#include <tuple>
#include <point_cloud/concurrency.h>
#include "util.h"
#include <cgv/math/constants.h>
#include <mutex>
#include <chrono>
#include "intersection.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <memory>

#include "generic_attribute_buffer.h"

//chunk meta data block
struct chunk_meta_data {
	cgv::vec4 aabb_pmin;	//vec4 because opengl extents vec3 to 16 Bytes
	cgv::vec4 aabb_pmax;
	GLuint size;
	GLuint max_size; 			//max_size is not expected to be changed on device side
};




template <typename POINT>
struct chunk {
	std::vector<POINT> points;
	//is used as index in other point attribute arrays
	std::vector<GLuint> point_ids;
private:
	/// stores points of type POINT
	GLuint point_buffer_p = 0;
	GLuint point_buffer_size = 0; //actual capacity of the allocated buffer, most of the time equal to reserved_size_p
	/// used as a reference to attributes associated to the stored points
	GLuint point_id_buffer_p = 0;

	GLuint meta_data_buffer_p = 0;
	cgv::box3 aabb_p;
	GLuint reserved_size_p = 0; //max number of points held by this chunk without reallocating buffers
	
	cgv::render::context* ctx_ptr = nullptr;

	//error handling
	bool gpu_point_buffer_overflow = false;
	bool host_dirty_p = false;
	bool device_dirty_p = false;
public:

	~chunk() {
		if (ctx_ptr) {
			clear_buffers();
		}
	}

	chunk() = default;

	chunk(const chunk& c) = delete;

	void resize(const size_t size) {
		this->points.resize(size);
		this->point_ids.resize(size, -1);
	}

	void add_points(const POINT* pnts, const GLuint* pnt_ids, const unsigned size, const bool update_aabb = false) {
		if (update_aabb) {
			for (unsigned i = 0; i < size; ++i) {
				aabb_p.add_point(pnts[i].position());
			}
		}
		size_t new_size = this->size() + size;
		size_t old_size = this->size();
		this->resize(new_size);
		memcpy(points.data() + old_size, pnts, size*sizeof(POINT));
		memcpy(point_ids.data() + old_size, pnt_ids, size*sizeof(GLuint));
	}

	void resize_reserved(const size_t size) {
		reserved_size_p = size;
	}

	GLuint point_buffer() const {
		return point_buffer_p;
	}

	GLuint id_buffer() const {
		return point_id_buffer_p;
	}

	GLuint meta_buffer() const {
		return meta_data_buffer_p;
	}

	// actual number of points stored
	uint32_t size() const {
		return points.size();
	}
	
	/// number of points storable without resizing 
	uint32_t reserved_size() const {
		return reserved_size_p;
	}

	/// size of associated buffer
	uint32_t buffer_size() const {
		return point_buffer_size;
	}
	
	//iterates valid points
	POINT* points_begin() {
		return points.data();
	}

	POINT* points_end() {
		return points.data()+size();
	}
	/// access the aabb of the chunk
	box3& bounding_box() {
		return aabb_p;
	}

	POINT& point_at(const size_t& i) {
		return points[i];
	}

	unsigned int& id_at(const size_t& i) {
		return point_ids[i];
	}

	//set this if data was changed on gpu side
	bool& device_buffers_changed() {
		return device_dirty_p;
	}
	//set this if data was changed on cpu side
	bool& host_data_changed() {
		return host_dirty_p;
	}

	void clear_dirty_bits() {
		device_dirty_p = host_dirty_p = false;
	}

	bool is_initialized() const {
		return point_buffer_p != 0;
	}

	void init_buffers(cgv::render::context& ctx) {
		ctx_ptr = &ctx;
		if (point_buffer_p == 0){
			glCreateBuffers(1, &point_buffer_p);
			glCreateBuffers(1, &point_id_buffer_p);
			glCreateBuffers(1, &meta_data_buffer_p);
			upload_to_buffers();
		}
	}
	
	void clear_buffers() {
		if (point_buffer_p != 0) {
			glDeleteBuffers(1, &point_buffer_p);
			glDeleteBuffers(1, &point_id_buffer_p);
			glDeleteBuffers(1, &meta_data_buffer_p);
		}
		point_buffer_size = 0;
		point_buffer_p = point_id_buffer_p = meta_data_buffer_p = 0;
	}
	
	void clear_host_data() {
		points.clear();
		point_ids.clear();
		host_dirty_p = true;
	}

	void allocate_dynamic_buffers() {
		point_buffer_size = this->reserved_size();
		glNamedBufferData(point_buffer_p, point_buffer_size * sizeof(POINT), nullptr, GL_STATIC_READ);
		glNamedBufferData(point_id_buffer_p, point_buffer_size * sizeof(GLuint), nullptr, GL_STATIC_READ);
	}

	void upload_to_buffers() {
		assert(point_buffer_p != 0 && point_id_buffer_p != 0 && meta_data_buffer_p != 0);

		// resize buffers
		if (size() > reserved_size())
			resize_reserved(size());
		
		if (buffer_size() != reserved_size()) {
			allocate_dynamic_buffers();
		}
		
		assert(points.size() == this->size());
		assert(reserved_size() == buffer_size());

		glNamedBufferSubData(point_buffer_p, 0, points.size() * sizeof(POINT), points.data());
		glNamedBufferSubData(point_id_buffer_p, 0, points.size() * sizeof(GLuint), point_ids.data());

		chunk_meta_data meta;
		meta.aabb_pmin = aabb_p.get_min_pnt().lift();
		meta.aabb_pmax = aabb_p.get_max_pnt().lift();
		meta.size = (GLuint)size();
		meta.max_size = (GLuint)buffer_size();
		glNamedBufferData(meta_data_buffer_p, sizeof(chunk_meta_data), &meta, GL_STREAM_DRAW);
		
		clear_dirty_bits();
	}

	/// checks the overflow flag set by download_buffers()
	bool check_overflow() const {
		return gpu_point_buffer_overflow;
	}

	// maps buffers internaly and copies data, this will overwrite points and point_ids content
	void download_buffers() {
		//
		//get meta data
		auto chMeta = download_meta_data();
		//do some checks, fix overflows
		gpu_point_buffer_overflow = chMeta.size > chMeta.max_size;
		
		if (gpu_point_buffer_overflow) {
			chMeta.size = chMeta.max_size;
		}

		aabb_p = box3(chMeta.aabb_pmin, chMeta.aabb_pmax);
		auto size_p = chMeta.size;
		resize(size_p);
		
		//get point data
		POINT* chPoints = static_cast<POINT*>(glMapNamedBufferRange(point_buffer_p, 0, point_buffer_size * sizeof(POINT), GL_MAP_READ_BIT));
		GLuint* chPointsIds = static_cast<GLuint*>(glMapNamedBufferRange(point_id_buffer_p, 0, point_buffer_size * sizeof(GLuint), GL_MAP_READ_BIT));
		
		memcpy(points.data(), chPoints, sizeof(POINT) * size());
		memcpy(point_ids.data(), chPointsIds, sizeof(GLuint) * size());
		glUnmapNamedBuffer(point_buffer_p);
		glUnmapNamedBuffer(point_id_buffer_p);
		
		clear_dirty_bits();
	}

	chunk_meta_data download_meta_data() {
		chunk_meta_data* chMetaPtr = static_cast<chunk_meta_data*>(glMapNamedBufferRange(meta_data_buffer_p, 0, sizeof(chunk_meta_data), GL_MAP_READ_BIT));
		if (chMetaPtr == nullptr) {
			std::cerr << "failed to map buffer! GL_ERROR: " << glutil::get_glError_as_string() << "\n";
			return chunk_meta_data();
		}
		chunk_meta_data chMeta = *chMetaPtr;
		glUnmapNamedBuffer(meta_data_buffer_p);
		return chMeta;
	}

	//input is an array containing new point ids located at the offset of the old id's value
	//this method takes any type as input with an [] operator defined returning something castable to unsigned int
	template<typename T>
	void permute_point_ids(T& permutation) {
		for (auto& id : point_ids) {
			id = (unsigned)permutation[id];
		}
	}
};

class int24_ivec3_hasher {
public:
	inline size_t operator()(const cgv::ivec3& v) const {
		size_t sx = v.x() & 0x00FFFFFF;
		size_t sy = (size_t)(v.y() & 0x00FFFFFF) << 24;
		size_t sz = (size_t)(v.z() & 0x00FFFFFF) << 48;
		return (sx | sy) | sz;
	}
};

template<typename POINT>
struct chunk_point_iterator{
	typename std::unordered_map<cgv::ivec3, std::shared_ptr<chunk<POINT>>, int24_ivec3_hasher>::iterator chunk_iterator;
	typename std::unordered_map<cgv::ivec3, std::shared_ptr<chunk<POINT>>, int24_ivec3_hasher>::iterator chunks_end;
	POINT* pnt_ptr = nullptr;
	POINT* pnts_end = nullptr;
	
	chunk_point_iterator& operator++() {
		++pnt_ptr;
		auto& points = chunk_iterator->second->points;
		if (pnt_ptr == pnts_end) {
			++chunk_iterator;
			if (chunk_iterator != chunks_end) {
				auto& points = chunk_iterator->second->points;
				pnt_ptr = points.data();
				pnts_end = pnt_ptr + points.size();
			}
			else {
				pnt_ptr = nullptr;
			}
		}
		return *this;
	}

	POINT& operator*() {
		return *pnt_ptr;
	}

	POINT* operator->() {
		return pnt_ptr;
	}

	// two iterators equal if they point to the same element
	bool operator==(const chunk_point_iterator& b) const {
		return b.pnt_ptr == this->pnt_ptr;
	}

	bool operator!=(const chunk_point_iterator& b) const {
		return b.pnt_ptr != this->pnt_ptr;
	}
};

template <typename POINT>
class chunks{
public:
	//types in cgv namespace
	using ivec2 = cgv::ivec2;
	using ivec3 = cgv::ivec3;
	using dvec2 = cgv::dvec2;
	using vec2 = cgv::vec2;
	using vec3 = cgv::vec3;
	using vec4 = cgv::vec4;
	using mat3 = cgv::mat3;
	using mat4 = cgv::mat4;
	using mat34 = cgv::mat34;
	using dmat4 = cgv::dmat4;
	using quat = cgv::quat;
	using rgb = cgv::rgb;
	using rgba = cgv::rgba;
	using rgb8 = cgv::rgb8;
	using rgba8 = cgv::rgba8;
	using box3 = cgv::box3;

	using chunk_type = chunk<POINT>;
	using chunk_map_t = std::unordered_map<ivec3, chunk_type, int24_ivec3_hasher>;
	using chunk_iterator_t = typename std::unordered_map<ivec3, std::shared_ptr<chunk_type>, int24_ivec3_hasher>::iterator;
	using point_iterator_t = chunk_point_iterator<POINT>;
	using index_chunk_pair_type = std::pair<const ivec3, chunk<POINT>>;

private:
	/// maps from 3d chunk coordinates to chunks, prefer only keeping filled chunks here
	std::unordered_map<ivec3, std::shared_ptr<chunk_type>, int24_ivec3_hasher> chunk_map;
	/// swapping chunk for point moving operations
	chunk_type swapping_chunk_p; 

	cgv::render::context* ctx_ptr;

	float chunk_extents_p;
	box3 point_cloud_bounding_box_p;
	mutable int num_points_c = -1;

	std::unordered_map<std::string, int> attribute_name_map;
	std::vector<generic_point_attribute> additional_attributes;
	GLuint next_point_id; //point id of a newly added point

	void invalidate_cached_values() {
		num_points_c = -1;
	}

	void init_swapping_chunk() {
		//construct and assign chunk object
		constexpr float infinity = std::numeric_limits<float>::infinity();
		swapping_chunk_p.bounding_box() = box3(vec3(-infinity, -infinity, -infinity), vec3(infinity, infinity, infinity));
	}
public:

	chunks() {
		init_swapping_chunk();
	};

	
	template <typename OTHER_POINT>
	chunks(const std::vector<OTHER_POINT>&src, float chunk_cube_size) {
		create_chunks_from<OTHER_POINT>(src.data(), src.size(), chunk_cube_size);
		init_swapping_chunk();
	}
	

	int add_point_attribute(const std::string& name, void* data,const GLuint binding_point, const size_t num_elements, const size_t element_size) {
		int attr_id = additional_attributes.size();
		attribute_name_map[name] = attr_id;
		additional_attributes.emplace_back(element_size);
		additional_attributes[attr_id].load(data, num_elements);
		additional_attributes[attr_id].set_binding_point(binding_point);
		return attr_id;
	}

	generic_point_attribute& get_attribute(int id) {
		return additional_attributes.at(id);
	}

	int find_attribute_id_by_name(const std::string& attr_name) {
		auto res = attribute_name_map.find(attr_name);
		if (res != attribute_name_map.end()) {
			return res->second;
		}
		return -1;
	}

	//returns an infinite global chunk usable for point relocating operations
	chunk<POINT>& swapping_chunk() {
		return swapping_chunk_p;
	}

	const ivec3 global_chunk_dummy_index() const{
		return ivec3(0x7FFFFFFF);
	}

	size_t num_filled_chunks() const {
		return chunk_map.size();
	}

	size_t num_chunks() const {
		return chunk_map.size();
	}

	// number of points stored by this data structure
	size_t num_points() const {
		if (num_points_c == -1) {
			int acc = 0;
			for (auto& ch : chunk_map) {
				acc += ch.second->size();
			}
			num_points_c = acc;
		}
		return num_points_c;
	}

	/// shared cube edge length of the stored chunks 
	float chunk_cube_size() const {
		return chunk_extents_p;
	}

	chunk<POINT>& at(const int x, const int y, const int z) {
		return *chunk_map.at(cgv::ivec3(x, y, z));
	}

	chunk<POINT>& at(const cgv::ivec3 &v) {
		return *chunk_map.at(v);
	}

	//can also create a new chunk on demand
	chunk<POINT>& get_chunk(const cgv::ivec3& v) {
		auto& chunk_ptr = chunk_map[v];
		//initialize chunk on demand
		if (!chunk_ptr) {
			chunk_ptr = std::make_shared<chunk_type>();
			chunk_ptr->bounding_box() = bounding_box_of(v);
		}
		return *chunk_ptr;
	}

	cgv::ivec3 chunk_index_from_point(const cgv::vec3& pnt) const {
		cgv::dvec3 tmp = cgv::dvec3(pnt) / chunk_cube_size();
		tmp.floor();
		return cgv::ivec3(tmp.x(), tmp.y(), tmp.z());
	}

	void download_buffers() {
		if (ctx_ptr) {
			for (auto& index_chunk_pair : chunk_map) {
				index_chunk_pair.second->download_buffers();
			}
			for (auto& attr : additional_attributes) {
				attr.download();
			}
		}
	}

	void upload_to_buffers() {
		if (ctx_ptr) {
			for (auto& index_chunk_pair : chunk_map) {
				index_chunk_pair.second->upload_to_buffers();
			}
			upload_attributes_to_buffers();
		}
	}

	void upload_changes_to_buffers() {
		if (ctx_ptr) {
			for (auto& index_chunk_pair : chunk_map) {
				auto& ch = index_chunk_pair.second;
				//initialize new chunks / upload data
				if (!ch->is_initialized()) {
					ch->init_buffers(*ctx_ptr);
				}
				else if (ch->host_data_changed()) {
					ch->upload_to_buffers();
				}
				ch->clear_dirty_bits();
			}
			upload_attributes_to_buffers();
		}
	}

	void upload_attributes_to_buffers() {
		for (auto& attr : additional_attributes) {
			attr.upload();
		}
	}

	template<typename POINT_B>
	void create_chunks_from(const POINT_B* src_points, const size_t num_points, float chunk_cube_extents) {
		invalidate_cached_values();
		const POINT_B* src_points_end = src_points + num_points;
		chunk_extents_p = chunk_cube_extents;
			
		//copy points
		{
			unsigned idx = 0;
			for (const POINT_B* p = src_points; p < src_points_end; ++p) {
				auto index = chunk_index_from_point(p->position());
				//convert to chunk point format
				//auto& chunk_ref = *chunk_map[index];
				auto& chunk_ref = get_chunk(index);
				chunk_ref.points.push_back((POINT)*p);
				chunk_ref.point_ids.push_back(idx++);
				chunk_ref.bounding_box() = bounding_box_of(index);
				point_cloud_bounding_box_p.add_point(p->position());
			}
			next_point_id = idx;
		}
	}

	template<typename POINT_B, typename POINT_ID>
	void create_chunks_from(const POINT_B* src_points, const POINT_ID* src_point_ids, const size_t num_points, float chunk_cube_extents) {
		invalidate_cached_values();
		const POINT_B* src_points_end = src_points + num_points;
		chunk_extents_p = chunk_cube_extents;
		//copy points
		{
			unsigned idx = 0;
			for (const POINT_B* p = src_points; p < src_points_end; ++p) {
				auto index = chunk_index_from_point(p->position());
				auto& chunk_ref = get_chunk(index);
				chunk_ref.points.push_back((POINT)*p);
				chunk_ref.point_ids.push_back(src_point_ids[idx++]);
				chunk_ref.bounding_box() = bounding_box_of(index);
				point_cloud_bounding_box_p.add_point(p->position());
			}
		}
	}

	//deletes allocated opengl buffers but keeps the chunks
	void clear_buffers(cgv::render::context& ctx) {
		if (ctx_ptr) {
			for (auto& p : chunk_map) {
				p.second->clear_buffers();
			}

			for (auto& attr : additional_attributes) {
				attr.clear();
			}
		}
	}
	// creates opengl buffers for each chunk
	void init_buffers(cgv::render::context& ctx) {
		ctx_ptr = &ctx;
		swapping_chunk_p.init_buffers(ctx);
		for (auto& p : chunk_map) {
			p.second->init_buffers(ctx);
		}

		for (auto& attr : additional_attributes){
			attr.init();
		}

	}

	//this method takes any type as input with an [] operator defined returning something castable to unsigned int
	template<typename T>
	void permute_point_ids(T& permutation) {
		swapping_chunk_p.permute_point_ids(permutation);
		for (auto& p : chunk_map) {
			p.second->permute_point_ids(permutation);
		}
	}

	//returns an iterator over the chunk map, excludes global swapping chunk
	chunk_iterator_t begin() {
		return chunk_map.begin();
	}

	chunk_iterator_t end() {
		return chunk_map.end();
	}
 
	GLuint add_point(POINT& pnt, generic_point_attribute* attributes, int* attribute_ids, int num_attributes, int attr_index) {
		invalidate_cached_values();
		//find chunk
		auto ix = chunk_index_from_point(pnt.position());
		auto& ch = get_chunk(ix);
		//add point
		ch.add_points(&pnt, &next_point_id, 1, false);
		//add attributes
		for (int i = 0; i < num_attributes; ++i) {
			//add element
			auto& attr_ch = get_attribute(attribute_ids[i]);
			attr_ch.append_element(attributes[i].element(attr_index));
		}
		ch.host_data_changed() = true;

		return next_point_id++;
	}

//move points from global chunk based on their position into the chunk map
	void distribute_global_chunk() {
		invalidate_cached_values();
		auto& swap_chunk = swapping_chunk();
		chunk_meta_data meta = swap_chunk.download_meta_data();

		if (meta.size == 0)
			return;

		assert(meta.size <= swap_chunk.buffer_size());
		if (meta.size > swap_chunk.buffer_size()) {
			std::cerr << "overflow occured in global swapping chunk, points may not be properly relocated!\n";
		}

		POINT* points = (POINT*)glMapNamedBuffer(swap_chunk.point_buffer(), GL_READ_ONLY);
		GLuint* ids = (GLuint*)glMapNamedBuffer(swap_chunk.id_buffer(), GL_READ_ONLY);
		std::vector<chunk<POINT>*> changed_chunks_tmp;

		for (int i = 0; i < meta.size; ++i) {
			POINT p = points[i];
			auto ix = chunk_index_from_point(p.position());
			auto& ch = get_chunk(ix);
			if (ch.device_buffers_changed()) {
				ch.download_buffers();
			}

			ch.points.push_back(p);
			ch.point_ids.push_back(ids[i]);

			changed_chunks_tmp.push_back(&ch);
			ch.host_data_changed() = true;
		}

		glUnmapNamedBuffer(swap_chunk.point_buffer());
		glUnmapNamedBuffer(swap_chunk.id_buffer());

		for (auto* ch_ptr : changed_chunks_tmp) {
			if (!ch_ptr->is_initialized()) {
				ch_ptr->init_buffers(*ctx_ptr);
			}
			else if (ch_ptr->host_data_changed()) {
				ch_ptr->upload_to_buffers();
			}
		}
		//delete chunk content
		swap_chunk.resize(0);
		swap_chunk.upload_to_buffers();
	}

	// returns an iterator over all points except the global swapping chunk
	point_iterator_t points_begin() {
		auto r = point_iterator_t();
		r.chunk_iterator = chunk_map.begin();
		r.chunks_end = chunk_map.end();
		if (!chunk_map.empty()) {
			auto& c = chunk_map.begin()->second;
			r.pnt_ptr = c->points_begin();
			r.pnts_end = c->points_end();
		}
		return r;
	}

	point_iterator_t points_end() {
		auto r = point_iterator_t();
		r.chunk_iterator = chunk_map.end();
		r.pnt_ptr = nullptr;
		return r;
	}

	box3 bounding_box_of(ivec3 chunk_addr) {
		vec3 minp = vec3(chunk_addr.x()++ * chunk_extents_p, chunk_addr.y()++ * chunk_extents_p, chunk_addr.z()++ * chunk_extents_p);
		vec3 maxp = vec3(chunk_addr.x() * chunk_extents_p, chunk_addr.y() * chunk_extents_p, chunk_addr.z() * chunk_extents_p);
		return box3(minp, maxp);
	}

	std::array<int, 3> xyz_indices_from_point(const vec3& pnt) {
		vec3 tmp = (pnt) / chunk_cube_size();
		tmp.floor();
		return std::array<int, 3>{(int)tmp.x(), (int)tmp.y(), (int)tmp.z()};
	}
	
	void get_all(std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		for (auto& ch_index_pair : chunk_map) {
			intersecting_chunks.emplace_back(ch_index_pair.first, ch_index_pair.second.get());
		}
	}

	/// intersect frustum defined by a model view projection matrix
	void intersect_frustum(const cgv::math::fmat<float, 4, 4>& mvp_matrix, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		std::array<vec4, 6> planes;
		vec4 p4 = mvp_matrix.row(3);
		for (int i = 0; i < 3; ++i) {
			planes[(i << 1)] = p4 - mvp_matrix.row(i);
			planes[(i << 1) + 1] = p4 + mvp_matrix.row(i);
		}
		intersect_frustum(planes.data(), intersecting_chunks);
	}
	
	static bool intersects_frustum(const vec4* planes, chunk<POINT>& chunk) {
		bool accept = true;
		//check planes
		for (int j = 0; j < 6; ++j) {
			bool rejected_by_plane = true;
			const vec4& plane = planes[j];
			//check box corners
			const box3& aabb = chunk.bounding_box();
			for (int k = 0; k < 8; ++k) {
				vec4 corner = aabb.get_corner(k).lift();
				if (dot(corner, plane) > 0.0) {
					rejected_by_plane = false;
					break;
				}
			}
			if (rejected_by_plane) {
				accept = false;
				break;
			}
		}
		return accept;
	}
	

	// writes the ids and pointers to the chunks inside the frustum to the vector given by intersecting_chunks
	void intersect_frustum(const vec4* planes, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		//intersecting_chunks.emplace_back(global_chunk_dummy_index(), &swapping_chunk());
		for (auto&  kv_pair : chunk_map) {
			chunk_type& chunk_ref = *kv_pair.second;
			const ivec3& chunk_id = kv_pair.first;

			if (intersects_frustum(planes, chunk_ref)) {
				intersecting_chunks.emplace_back(kv_pair.first, kv_pair.second.get());
				//intersecting_chunks.push_back(std::make_pair<const cgv::render::render_types::ivec3, chunk<POINT>*>(kv_pair.first,&kv_pair.second));
			}
		}
	}

	// writes the pointers to the chunks inside the frustum to the vector given by intersecting_chunks
	void intersect_frustum(const vec4* planes, std::vector<chunk<POINT>*>& intersecting_chunks) {
		//intersecting_chunks.emplace_back(global_chunk_dummy_index(), &swapping_chunk());
		for (auto& kv_pair : chunk_map) {
			chunk_type& chunk_ref = *kv_pair.second;
			if (intersects_frustum(planes, chunk_ref)) {
				intersecting_chunks.emplace_back(kv_pair.second.get());
			}
		}
	}
	template<typename Container>
	void copy_chunk_ptrs(Container& chunks) {
		for (auto& kv_pair : chunk_map) {
			chunk_type& chunk_ref = *kv_pair.second;
			chunks.emplace_back(kv_pair.second.get());
		}
	}

	// writes the ids of the chunks intersecting with the box to the vector in chunk_ids
	void intersect_box(box3& box, const vec3& translation, const quat& orientation, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		float radius = length(box.get_extent());
		auto base = xyz_indices_from_point(translation);

		auto min_xyz = xyz_indices_from_point(translation - radius);
		auto max_xyz = xyz_indices_from_point(translation + radius);
		
		for (int z = min_xyz[2]; z <= max_xyz[2]; ++z) {
			for (int y = min_xyz[1]; y <= max_xyz[1]; ++y) {
				for (int x = min_xyz[0]; x <= max_xyz[0]; ++x) {
					auto kv_pair = chunk_map.find(ivec3(x, y, z));
					if (kv_pair != chunk_map.end()) {
						//TODO do box intersection test

						intersecting_chunks.emplace_back(kv_pair->first, kv_pair->second.get());
					}
				}
			}
		}
	}

	// writes the ids of the chunks intersecting with the sphere to the vector in chunk_ids, current implementation is basically intersect cube
	void intersect_range(std::array<int,3>& min_xyz, std::array<int, 3>& max_xyz, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		for (int z = min_xyz[2]; z <= max_xyz[2]; ++z) {
			for (int y = min_xyz[1]; y <= max_xyz[1]; ++y) {
				for (int x = min_xyz[0]; x <= max_xyz[0]; ++x) {
					auto kv_pair = chunk_map.find(ivec3(x, y, z));
					if (kv_pair != chunk_map.end()) {
						intersecting_chunks.emplace_back(kv_pair->first, kv_pair->second.get());
					}
				}
			}
		}
	}

	//position is center of the cube
	void intersect_cube(const vec3& position, const float a, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		float radius = a * 0.5f;
		auto base = xyz_indices_from_point(position);

		// iterates a bounding volume enclosing the cube

		auto min_xyz = xyz_indices_from_point(position - radius);
		auto max_xyz = xyz_indices_from_point(position + radius);
		intersect_range(min_xyz, max_xyz, intersecting_chunks);
	}

	void intersect_sphere(const vec3& sphere_pos, const float radius, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		float sqradius = radius * radius;
		auto base = xyz_indices_from_point(sphere_pos);
		
		//std::vector<std::pair<const ivec3, chunk<POINT>*>> tmp;
		auto min_xyz = xyz_indices_from_point(sphere_pos - radius);
		auto max_xyz = xyz_indices_from_point(sphere_pos + radius);
		intersect_range(min_xyz, max_xyz, intersecting_chunks);

		/*for (auto& kv_pair : tmp) {
			auto& ch = kv_pair.second;
			vec3 p = cgv::math::clamp(sphere_pos, ch->bounding_box().get_min_pnt(), ch->bounding_box().get_max_pnt());
			vec3 diff = sphere_pos - p;
			if (cgv::math::dot(diff, diff) < radius) {
				intersecting_chunks.emplace_back(kv_pair);
			}
		}*/
	}

	//selects every chunk bellow and intersecting with the clipping plane
	void intersect_clip_plane(const vec3& plane_normal, const float plane_distance, std::vector<std::pair<const ivec3, chunk<POINT>*>>& intersecting_chunks) {
		// intersecting_chunks.emplace_back(global_chunk_dummy_index(), &global_chunk());
		for (auto& index_chunk_pair : chunk_map) {
			auto& ch_ptr = index_chunk_pair.second;
			// Convert AABB to center-extents representation
			
			vec3 c = ch_ptr->bounding_box().get_center();
			vec3 e = ch_ptr->bounding_box().get_max_pnt() - c; // Compute positive extents

			// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
			float proj_radius = dot(abs(plane_normal), e);

			// Compute distance of box center from plane
			float s = dot(plane_normal, c) - plane_distance;

			// Intersection occurs when distance s falls within [-inf,+r] interval
			if (s <= proj_radius) {
				intersecting_chunks.emplace_back(index_chunk_pair.first, index_chunk_pair.second.get());
			}
		}
	}

	//test ray against chunk bounding boxes
	void intersect_ray(const vec3& origin, const vec3& dir, std::vector<std::pair<ivec3, chunk<POINT>*>>& intersecting_chunks) {
		for (auto& index_chunk_pair : chunk_map) {
			auto& ch_ptr = index_chunk_pair.second;
			float t;
			vec3 p, n;
			if (cgv::media::ray_axis_aligned_box_intersection(origin, dir, ch_ptr->bounding_box(),t,p,n, 0.f)) {
				intersecting_chunks.emplace_back(index_chunk_pair.first, index_chunk_pair.second.get());
			}
		}
		
	}


	void collect_points(std::vector<POINT>& points, std::vector<GLuint>& point_ids) {
		for (auto& index_chunk_pair : chunk_map) {
			chunk<POINT>& ch_ref = *index_chunk_pair.second;
			size_t start = points.size();
			points.resize(start + ch_ref.points.size());
			point_ids.resize(start + ch_ref.points.size());
			memcpy(points.data() + start, ch_ref.points.data(), ch_ref.points.size() * sizeof(POINT));
			memcpy(point_ids.data() + start, ch_ref.point_ids.data(), ch_ref.points.size() * sizeof(GLuint));
		}
	}

	void disable_chunks() {
		point_cloud_bounding_box_p = box3();
		if (ctx_ptr)
			clear_buffers(*ctx_ptr);
		std::vector<POINT> points;
		std::vector<GLuint> point_ids;
		collect_points(points, point_ids);
		chunk_map.clear();
		
		invalidate_cached_values();
		chunk_extents_p = std::numeric_limits<float>::max();


		auto& chunk_ref = get_chunk(ivec3(0));
		
		unsigned idx = 0;
		for (auto p = points.begin(); p < points.end(); ++p) {
			chunk_ref.points.push_back(*p);
			chunk_ref.point_ids.push_back(point_ids[idx++]);
			point_cloud_bounding_box_p.add_point(p->position());
		}
		if (ctx_ptr)
			init_buffers(*ctx_ptr);
	}

	//takes all points recreates chunks with the given size, this will clear the buffers
	void rechunk(const float new_chunk_cube_size) {
		point_cloud_bounding_box_p = box3();
		if (ctx_ptr)
			clear_buffers(*ctx_ptr);

		std::vector<POINT> points;
		std::vector<GLuint> point_ids;
		collect_points(points, point_ids);
		chunk_map.clear();

		//create new chunks
		create_chunks_from<POINT, GLuint>(points.data(), point_ids.data(), points.size(), new_chunk_cube_size);
		if (ctx_ptr)
			init_buffers(*ctx_ptr);
	}
};