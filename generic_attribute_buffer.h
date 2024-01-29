#pragma once
#include <vector>
#include <unordered_map>
#include <glew/GL/glew.h>
#include <cassert>
#include <memory>

// stores data in opengl buffers and main memory without knowing the type
class generic_point_attribute {
	GLint binding_point; //binding point for shader storage buffer
	GLuint buffer = 0;
	size_t element_size;
	size_t size_p; //number of elements
	void* data_p;
	std::vector<uint8_t> raw_data;
	size_t size_in_bytes;

public:

	generic_point_attribute() = default;

	generic_point_attribute(const unsigned element_size);
	
	generic_point_attribute(generic_point_attribute&& other) noexcept;
	
	generic_point_attribute(const generic_point_attribute& other) = delete;

	~generic_point_attribute();

	inline void set_binding_point(GLuint p) {
		binding_point = p;
	}

	void init();

	void clear();

	void bind();

	/// number of stored elements
	inline size_t size() const {
		return this->size_p;
	}

	inline GLuint buffer_name() const {
		return buffer;
	}

	inline bool is_initialized() const {
		return buffer != 0;
	}

	template<typename T=char>
	T* data() {
		return static_cast<T*>(data_p);
	}

	//copy data from buffer to main memory
	void download();
	
	//copy data from main memory to buffer
	void upload();

	/// copies data into the object, @param size is the number of elements
	void load(void* data, size_t size);

	/// adds an element, element size is given by size()
	void append_element(void* data);

	/// access element
	template<typename T = char>
	T* element(size_t i) {
		return (T*)((uintptr_t)data_p+element_size*i);
	}
};