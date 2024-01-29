#version 430

// point format
struct Vertex{
	float x;
	float y;
	float z;
	uint colors;
};

//buffers for secondary chunk
layout(std430, binding = 10) buffer ssChunkBPointBuffer {
	Vertex chunk_b_pnts[];
};

layout(std430, binding = 11) buffer ssChunkBInputIdBuffer{
	uint chunk_b_ids[];
};

/*
//stores chunk ids
layout(std430, binding = 12) buffer ssChunkBAdditionalData{
	ivec4 data[];
} chunk_b_additional_data;
*/

// chunk metadata

struct chunk_meta_data {
	vec4 aabb_pmin;
	vec4 aabb_pmax;
	uint size;
	uint max_size;
};

layout(std430, binding = 29) buffer ssChunkAMetaDataBuffer {
	chunk_meta_data meta_a;
};

layout(std430, binding = 30) buffer ssChunkBMetaDataBuffer {
	chunk_meta_data meta_b;
};

/*
//***** begin interface of point_cloud_chunk.glsl **********************************
bool is_inside_chunk_a(in vec3 pnt);
uint move_point_to_chunk_b(inout Vertex vert, inout uint point_id);
//***** end interface of point_cloud_chunk.glsl **********************************
*/

//checks if a point can lays inside or at the surface of a bounding box
bool is_inside_aabb(in vec3 pnt,in vec3 aabb_pmin, in vec3 aabb_pmax){
    // check whether all elements of a boolean vector are true
	// return aabb_pmax == aabb_pmin;
	return all(lessThanEqual(pnt, aabb_pmax)) && all(greaterThanEqual(pnt, aabb_pmin));
}

bool is_inside_chunk_a(in vec3 pnt){
	return is_inside_aabb(pnt, meta_a.aabb_pmin.xyz, meta_a.aabb_pmax.xyz);
}

//takes the points index in chunk a and returns the points local index in chunk b
uint move_point_to_chunk_b(inout Vertex vert, inout uint point_id) {
	uint i = atomicAdd(meta_b.size,1);
	if ( i <  meta_b.max_size) {
		chunk_b_pnts[i] = vert;
		chunk_b_ids[i] = point_id;
		point_id = -1; /*insert reference to global deleted/moved/free point */
		return i;
	} else {
		//failed getting a free spot for placing the point
		return -1;
	}
}
