#pragma once

#include <cgv/render/render_types.h>
#include <cgv_gl/renderer.h>
#include <deque>



struct history_meta_data
{
	GLuint history_start = 0;
	GLuint history_capacity = 0; // allways (power of 2) -1, allows to use & as replacement for mod
	GLuint points_written = 0; // points_written > capacity means the ringbuffer overflowed and the history got corrupted
	/// set new capacity and resets history_start and points_written to zero if capacity shrinked
	void set_capacity(const GLuint capacity); 
};

struct label_operation {
	GLint label;
	GLuint index;
};

struct operation_span_element : public cgv::render::render_types {
	unsigned op_start;
	unsigned op_size;

	operation_span_element(const unsigned start=0, const unsigned size=0);
};

/// main memory stored history
struct inactive_history {
	std::vector<label_operation> history_buffer;
	std::vector<operation_span_element> operations_ptrs;
	history_meta_data history_meta_data_buffer;
public:
	inline inactive_history() {};

	template<typename OpIter>
	inactive_history(label_operation* h_buf_begin, size_t h_buf_size, OpIter op_begin, OpIter op_end, history_meta_data* meta_data) {
		history_buffer.resize(h_buf_size);
		std::memcpy(history_buffer.data(), h_buf_begin, sizeof(label_operation)*h_buf_size);
		for (auto op = op_begin; op != op_end; ++op) {
			operations_ptrs.push_back(*op);
		}
		history_meta_data_buffer = *meta_data;
	}

	inline unsigned capacity() const {
		return history_buffer.size();
	}

	void set_capacity(const unsigned c);
};

/// ment as a temporaly existing object to modify the label history from host side, buffers are unmapped on destruction
class memory_mapped_history {
	label_operation* label_ops_ptr;
	history_meta_data* meta_data_ptr;
	GLuint label_op_buffer;
	GLuint meta_data_buffer;
	history_meta_data meta_data;
public:
	memory_mapped_history(GLuint history_buffer, GLuint history_meta_data_buffer, unsigned history_buffer_capacity);
	//no copy allowed
	memory_mapped_history(const memory_mapped_history&) = delete;
	memory_mapped_history& operator=(const memory_mapped_history&) = delete;

	memory_mapped_history(memory_mapped_history&&) noexcept;
	memory_mapped_history& operator=(memory_mapped_history&&) noexcept;

	~memory_mapped_history();

	void add_label_operation(const label_operation& op);

	void unmap();

	bool is_good(); //returns true if mappings were successfull

	inline label_operation* get_label_ops() {
		return label_ops_ptr;
	}

	inline history_meta_data& get_meta_data() {
		return meta_data;
	}


protected:
	void move(memory_mapped_history& h) noexcept;
	
	void sync_meta_data();

	inline GLuint global_index_of(GLuint local_ix) {
		return (local_ix + meta_data.history_start) & meta_data.history_capacity;
	}
};



class history : public cgv::render::render_types{
	GLuint history_buffer = 0;
	GLuint history_meta_data_buffer = 0;
	unsigned history_buffer_capacity = 0;
	std::deque<operation_span_element> operations_ptrs;

	static constexpr GLuint history_buffer_pos = 7;
	static constexpr GLuint history_meta_data_pos = 8;

	cgv::render::shader_program* label_rollback_prog = nullptr;
	//store meta data read by add_rollback_operation
	history_meta_data last_op_meta;
public:
	// stores data commited by shaders to the history as one operation
	void add_rollback_operation();

	history() = default;
	inline history(unsigned logarithmic_size) {
		history_buffer_capacity = (1 << logarithmic_size) - 1;
	}

	// create buffers and shader program
	void init(cgv::render::context& ctx);

	//bind buffers history buffers
	inline void bind(cgv::render::context& ctx) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, history_buffer_pos, history_buffer);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, history_meta_data_pos, history_meta_data_buffer);
	}
	
	//allow setting a shader prog used for apply rollbacks
	inline void set_rollback_shader_prog(cgv::render::shader_program* prog) {
		label_rollback_prog = prog;
	}

	//deletes meta data buffers
	void reset_meta_data(cgv::render::context& ctx, uint32_t start = 0, uint32_t size = 0);

	const history_meta_data& get_last_operation_meta_data() const;

	// write labels that are not yet compiled into an operation to ops, return value is true if successfull
	bool get_stored_labels(std::vector<label_operation>& ops);

	//rollback last labeling operation and assign points to group given by point_group
	bool rollback_last_operation(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	//remove all stored operations, buffers stay unchanged
	void remove_all();

	memory_mapped_history make_mapping();

	inactive_history make_storeable_copy();

	bool restore(inactive_history& h);

	unsigned capacity() const {
		return history_buffer_capacity;
	}

	unsigned num_operations() const;
};