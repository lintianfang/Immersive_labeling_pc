#include "generic_attribute_buffer.h"
#include <cstdint>

generic_point_attribute::generic_point_attribute(const unsigned element_size) : element_size(element_size)
{
	size_p = size_in_bytes = 0;
	binding_point = 0;
	data_p = nullptr;
}

generic_point_attribute::generic_point_attribute(generic_point_attribute&& other) noexcept
{
	size_p = other.size_p;
	size_in_bytes = other.size_in_bytes;
	element_size = other.element_size;
	binding_point = other.binding_point;
	buffer = other.buffer;
	data_p = other.data_p;
	other.data_p = nullptr;
	other.buffer = 0;
	raw_data = std::move(other.raw_data);
}

generic_point_attribute::~generic_point_attribute()
{
	if (buffer != 0)
		clear();
}

void generic_point_attribute::init()
{
	glCreateBuffers(1, &buffer);
	upload();
}

void generic_point_attribute::clear()
{
	glDeleteBuffers(1, &buffer);
}

void generic_point_attribute::bind()
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_point, buffer_name());
}

void generic_point_attribute::download()
{
	assert(buffer != 0 && data());
	void* mapped_data = glMapNamedBufferRange(buffer, 0, size_p * element_size, GL_MAP_READ_BIT);
	memcpy(data_p, mapped_data, size_in_bytes);
	glUnmapNamedBuffer(buffer);
}

void generic_point_attribute::upload()
{
	assert(buffer != 0 && data());
	glNamedBufferData(buffer, size_in_bytes, data_p, GL_STATIC_READ);
}

void generic_point_attribute::load(void* data, size_t size)
{
	size_in_bytes = size * element_size;
	raw_data.resize(size_in_bytes + element_size - 1);
	//align start address
	uintptr_t max_start_addr = ((uintptr_t)raw_data.data() + element_size - 1);
	data_p = (void*) ( max_start_addr - (max_start_addr % element_size) );
	if (data_p) {
		memcpy(data_p, data, size_in_bytes);
		size_p = size;
	}
	else {
		size_p = 0;
		throw std::bad_alloc();
	}

}

void generic_point_attribute::append_element(void* data)
{
	unsigned old_size = size_p++;
	size_in_bytes = size_p * element_size;
	raw_data.resize(size_in_bytes + element_size - 1);
	//align start address and reset data pointer
	uintptr_t max_start_addr = ((uintptr_t)raw_data.data() + element_size - 1);
	data_p = (void*)(max_start_addr - (max_start_addr % element_size));
	//copy data
	if (data_p) {
		memcpy((char*)data_p + old_size*element_size, data, element_size);
	}
	else {
		size_p = 0;
		throw std::bad_alloc();
	}
}
