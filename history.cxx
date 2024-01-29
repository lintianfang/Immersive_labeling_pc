#include "history.h"

namespace {
	bool interval_intersects(const int& start_a, const int& end_a,const int& start_b, const int& end_b) {
		int left_bound = std::max(start_a, start_b);
		int right_bound = std::min(end_a, end_b);
		return right_bound - left_bound > 0;
	}

	constexpr GLenum history_buffer_flags = GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_WRITE_BIT;
}


operation_span_element::operation_span_element(const unsigned start, const unsigned size) :
	op_start(start), op_size(size) {}


memory_mapped_history::memory_mapped_history(GLuint history_buffer, GLuint history_meta_data_buffer, unsigned history_buffer_capacity):
	label_op_buffer(history_buffer), meta_data_buffer(history_meta_data_buffer)
{
	meta_data_ptr = (history_meta_data*)glMapNamedBufferRange(meta_data_buffer, 0, sizeof(history_meta_data), GL_MAP_READ_BIT | GL_MAP_WRITE_BIT);
	label_ops_ptr = (label_operation*)glMapNamedBufferRange(label_op_buffer, 0, history_buffer_capacity * sizeof(label_operation), GL_MAP_READ_BIT | GL_MAP_WRITE_BIT);
	if (meta_data_ptr)
		meta_data = *meta_data_ptr;
}

memory_mapped_history::memory_mapped_history(memory_mapped_history&& h) noexcept {
	move(h);
}

memory_mapped_history& memory_mapped_history::operator=(memory_mapped_history&& h) noexcept{
	move(h);
	return *this;
}

void memory_mapped_history::move(memory_mapped_history& h) noexcept {
	this->label_ops_ptr = h.label_ops_ptr;
	this->meta_data_ptr = h.meta_data_ptr;
	this->label_op_buffer = h.label_op_buffer;
	this->meta_data_buffer = h.meta_data_buffer;
	this->meta_data = h.meta_data;
	h.label_ops_ptr = nullptr;
	h.meta_data_ptr = nullptr;
}

memory_mapped_history::~memory_mapped_history() {
	unmap();
}

bool memory_mapped_history::is_good() {
	return (label_ops_ptr != nullptr) & (meta_data_ptr != nullptr) & (meta_data.history_capacity != 0);
}

void memory_mapped_history::sync_meta_data() {
	if (meta_data_ptr)
		*meta_data_ptr = meta_data;
}

void memory_mapped_history::unmap() {
	sync_meta_data();
	if (label_ops_ptr)
		glUnmapNamedBuffer(label_op_buffer);
	if (meta_data_ptr)
		glUnmapNamedBuffer(meta_data_buffer);
}

void memory_mapped_history::add_label_operation(const label_operation& op) {
	GLuint ix = global_index_of(meta_data.points_written++);
	label_ops_ptr[ix] = op;
}

void history::add_rollback_operation()
{
	auto* meta_ptr = (history_meta_data*)glMapNamedBuffer(history_meta_data_buffer, GL_READ_WRITE);
	if (meta_ptr == NULL)
		return;
	last_op_meta = *meta_ptr;
	//auto meta = *meta_ptr;
	
	// check if there is anything to store
	if (last_op_meta.points_written > history_buffer_capacity) {
		//invalidate history if there is too many to store
		operations_ptrs.clear();
		//reset start and size information on device side
		meta_ptr->points_written = 0;
		meta_ptr->history_start = 0;
	}
	else if (last_op_meta.points_written > 0) {
		//reset start and size information on device side
		meta_ptr->points_written = 0;
		meta_ptr->history_start = (last_op_meta.history_start + last_op_meta.points_written) & last_op_meta.history_capacity;
		
		//write operation span to buffer
		operation_span_element op = operation_span_element(last_op_meta.history_start, last_op_meta.points_written);
		//check if anything was overwritten and remove overwritten operations if there were any
		{
			int start_op = op.op_start;
			int end_op = (op.op_start + op.op_size) & history_buffer_capacity;
			int unbounded_end_op = op.op_start + op.op_size;

			while (!operations_ptrs.empty()) {
				auto chk = operations_ptrs.front();
				
				int start_chk = chk.op_start;
				int end_chk = (chk.op_start + chk.op_size) & history_buffer_capacity;
				int unbounded_end_chk = start_chk + chk.op_size;
				//is fully enclosed

				bool op_wraps = unbounded_end_op > history_buffer_capacity;
				bool chk_wraps = unbounded_end_chk > history_buffer_capacity;

				//fully enclosed, none cross cap
				if (!(op_wraps || chk_wraps)) {
					if (interval_intersects(start_op, end_op, start_chk, end_chk)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				//op crossed cap
				else if (op_wraps && !chk_wraps) {
					int shift = history_buffer_capacity + 1;
					if (interval_intersects(start_op, unbounded_end_op, start_chk, end_chk) ||
						interval_intersects(start_op, unbounded_end_op, start_chk + shift, end_chk + shift)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				//chk crossed cap
				else if (!op_wraps && chk_wraps) {
					int shift = history_buffer_capacity + 1;
					if (interval_intersects(start_chk, unbounded_end_chk, start_op, end_op) ||
						interval_intersects(start_chk, unbounded_end_chk, start_op + shift, end_op + shift)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				else if (op_wraps && chk_wraps) {
					if (interval_intersects(start_op, unbounded_end_op, start_chk, unbounded_end_chk)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}

			}
		}
		operations_ptrs.push_back(op);
	}
	glUnmapNamedBuffer(history_meta_data_buffer);
}

void history::init(cgv::render::context& ctx)
{
	GLuint buffers[2];
	glCreateBuffers(2, buffers);
	history_buffer = buffers[0];
	history_meta_data_buffer = buffers[1];
	glNamedBufferStorage(history_buffer, history_buffer_capacity*sizeof(label_operation), nullptr, history_buffer_flags);
	glNamedBufferStorage(history_meta_data_buffer, sizeof(history_meta_data), nullptr, history_buffer_flags);
	reset_meta_data(ctx);
}

void history::reset_meta_data(cgv::render::context& ctx, uint32_t start, uint32_t size)
{
	history_meta_data meta;
	meta.history_capacity = history_buffer_capacity;
	meta.history_start = start;
	meta.points_written = size;
	glNamedBufferSubData(history_meta_data_buffer, 0, sizeof(history_meta_data), &meta);
}

const history_meta_data& history::get_last_operation_meta_data() const
{
	return last_op_meta;
}

bool history::get_stored_labels(std::vector<label_operation>& ops)
{
	if (history_meta_data_buffer == 0 || history_buffer == 0) {
		return false;
	}

	auto* meta_ptr = (history_meta_data*)glMapNamedBuffer(history_meta_data_buffer, GL_READ_ONLY);
	auto* history_ptr = (label_operation*)glMapNamedBuffer(history_buffer, GL_READ_ONLY);

	auto err = glGetError();
	if (err != GL_NO_ERROR) {
		return false;
	}

	auto meta = *meta_ptr;
	glUnmapNamedBuffer(history_meta_data_buffer);

	for (int i = 0; i < meta.points_written; ++i) {
		unsigned index = (i+meta.history_start) & meta.history_capacity;
		ops.push_back(history_ptr[index]);
	}
	glUnmapNamedBuffer(history_buffer);
	return true;
}

bool history::rollback_last_operation(cgv::render::context& ctx)
{
	if (operations_ptrs.empty()) {
		return false;
	}
	
	operation_span_element op = operations_ptrs.back();
	operations_ptrs.pop_back();
	//set meta data
	reset_meta_data(ctx, op.op_start, op.op_size);

	bind(ctx);

	//run rollback
	if (label_rollback_prog && label_rollback_prog->enable(ctx)) {
		// run computation
		glDispatchCompute(16, 1, 1);

		// synchronize
		glMemoryBarrier(GL_ALL_BARRIER_BITS);
		label_rollback_prog->disable(ctx);

		if (!operations_ptrs.empty()) {
			reset_meta_data(ctx, op.op_start, 0);
		}
		else {
			reset_meta_data(ctx, 0, 0);
		}
	}
	else {
		std::cerr << "history::rollback_last_operation: can't enable shader program\n";
		if (!label_rollback_prog)
			std::cerr << "history::rollback_last_operation: use set_rollback_shader_prog to set a shader program\n";
		return false;
	}
	return true;
}

void history::clear(cgv::render::context& ctx)
{
	glDeleteBuffers(1, &history_buffer);
	glDeleteBuffers(1, &history_meta_data_buffer);
	history_buffer = history_meta_data_buffer = 0;
}

void history::remove_all()
{
	operations_ptrs.clear();
}


inactive_history history::make_storeable_copy() {
	memory_mapped_history m = make_mapping();
	return inactive_history(m.get_label_ops(), (size_t)history_buffer_capacity, operations_ptrs.begin(), operations_ptrs.end(), &m.get_meta_data());
}

bool history::restore(inactive_history& h) {
	// delete existing buffer if capacity is different
	if (history_buffer != 0 && h.capacity() != this->capacity()) {
		glDeleteBuffers(1, &history_buffer);
		history_buffer = 0;
	}
	
	history_buffer_capacity = (unsigned)h.history_buffer.size();
	operations_ptrs = std::deque<operation_span_element>(h.operations_ptrs.cbegin(), h.operations_ptrs.cend());
	
	//initialize if nessescary
	if (history_buffer == 0) {
		glCreateBuffers(1, &history_buffer);
		glNamedBufferData(history_buffer, history_buffer_capacity, nullptr, history_buffer_flags);
	}
	if (history_meta_data_buffer == 0) {
		glCreateBuffers(1, &history_meta_data_buffer);
		glNamedBufferData(history_meta_data_buffer, sizeof(history_meta_data), nullptr, history_buffer_flags);
	}

	if (history_buffer && history_meta_data_buffer) {
		glNamedBufferSubData(history_buffer, 0, history_buffer_capacity, h.history_buffer.data());
		glNamedBufferSubData(history_meta_data_buffer, 0, sizeof(history_meta_data), &h.history_meta_data_buffer);
		return true;
	}
	return false;
}


unsigned history::num_operations() const {
	return operations_ptrs.size();
}

memory_mapped_history history::make_mapping() {
	return memory_mapped_history(history_buffer, history_meta_data_buffer, history_buffer_capacity);
}

void inactive_history::set_capacity(const unsigned c)
{
	if (c < capacity())
		operations_ptrs.clear();
	history_buffer.resize(c);
	history_meta_data_buffer.set_capacity(c);
}

void history_meta_data::set_capacity(const GLuint capacity)
{
	if (capacity < this->history_capacity) {
		history_start = 0;
		points_written = 0;
	}
	this->history_capacity = capacity;
}
