#version 150

//***** begin interface of quaternion.glsl ***********************************
vec4 unit_quaternion();
vec3 rotate_vector_with_quaternion(in vec3 preimage, in vec4 q);
vec3 inverse_rotate_vector_with_quaternion(in vec3 v, in vec4 q);
void quaternion_to_axes(in vec4 q, out vec3 x, out vec3 y, out vec3 z);
void quaternion_to_matrix(in vec4 q, out mat3 M);
void rigid_to_matrix(in vec4 q, in vec3 t, out mat4 M);
//***** end interface of quaternion.glsl ***********************************

/* //***** begin interface of point_inclusion.glsl ***********************************
bool is_behind_plane(in vec3 position, vec3 plane_normal, vec3 plane_origin);
bool is_in_sphere(in vec3 position, in vec3 sphere_translation, in float sphere_radius);
bool is_in_cuboid(in vec3 position, in vec3 aabb_min_p, in vec3 aabb_max_p, in vec3 translation, in vec4 rotation);
*/ //***** end interface of point_inclusion.glsl ***********************************

bool is_behind_plane(in vec3 position, vec3 plane_normal, vec3 plane_origin){
	return dot(position.xyz-plane_origin.xyz,plane_normal.xyz) <= 0;
}

// check position against sphere definition given by translation and radius
bool is_in_sphere(in vec3 position, in vec3 translation, in float radius) {
	vec3 d = position.xyz-translation.xyz;
	return (dot(d,d) < radius*radius);
}

//rotation is a quaternion
bool is_in_cuboid(in vec3 position, in vec3 aabb_min_p, in vec3 aabb_max_p, in vec3 translation, in vec4 rotation) {
	vec3 box_space_pos = inverse_rotate_vector_with_quaternion(position.xyz-translation.xyz, rotation);
	bvec3 b1 = greaterThan(box_space_pos.xyz,aabb_min_p.xyz);
	bvec3 b2 = lessThan(box_space_pos.xyz,aabb_max_p.xyz);
	return all(b1) && all(b2);
}