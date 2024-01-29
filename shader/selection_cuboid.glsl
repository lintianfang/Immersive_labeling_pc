#version 450

//min and max point of the box
uniform vec3 aabb_min_p;
uniform vec3 aabb_max_p;
//box translation and rotation in world coordinates
uniform vec4 selection_box_translation;
uniform vec4 selection_box_rotation; //this is a quaternion

/*
//***** begin interface selection_xxxxxx.glsl
bool is_in_selection(in vec4 view_position);
bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result);
//***** end interface selection_xxxxxx.glsl *********************************
*/


//***** begin interface of quaternion.glsl ***********************************
vec4 unit_quaternion();
vec3 rotate_vector_with_quaternion(in vec3 preimage, in vec4 q);
vec3 inverse_rotate_vector_with_quaternion(in vec3 v, in vec4 q);
void quaternion_to_axes(in vec4 q, out vec3 x, out vec3 y, out vec3 z);
void quaternion_to_matrix(in vec4 q, out mat3 M);
void rigid_to_matrix(in vec4 q, in vec3 t, out mat4 M);
//***** end interface of quaternion.glsl ***********************************

const float infinity = uintBitsToFloat(0x7F800000);

vec3 to_box_space(in vec3 world_position) {
	return inverse_rotate_vector_with_quaternion(world_position.xyz-selection_box_translation.xyz, selection_box_rotation);
}

bool is_in_selection(in vec4 position){
	//vec3 box_space_pos = inverse_rotate_vector_with_quaternion(world.xyz-selection_box_translation.xyz, selection_box_rotation);
	vec3 box_space_pos = to_box_space(position.xyz);
	bvec3 b1 = greaterThan(box_space_pos.xyz,aabb_min_p.xyz);
	bvec3 b2 = lessThan(box_space_pos.xyz,aabb_max_p.xyz);
	return all(b1) && all(b2);
}

//time discrete collision resolution of a point as dynamic body against the selection shape as static body
//idea is to push the point the shortest way out of the collision
bool resolve_collision(in vec4 position, out vec4 new_position, out vec3 collision_normal){
	vec3 box_space_pos = to_box_space(position.xyz);
	bvec3 b1 = greaterThan(box_space_pos.xyz,aabb_min_p.xyz);
	bvec3 b2 = lessThan(box_space_pos.xyz,aabb_max_p.xyz);
	//check if point is inside the box
	if ( all(b1) && all(b2) ){
		vec3 extent = aabb_max_p.xyz-aabb_min_p.xyz;
		vec3 dist_to_boundary = (extent*0.5)-abs(box_space_pos);
		int min_i = 0;
		for (int i=1;i<3;++i){
			min_i = (dist_to_boundary[min_i] < dist_to_boundary[i]) ? min_i : i;
		}
		collision_normal = vec3(0.0,0.0,0.0);
		collision_normal[min_i] = sign(box_space_pos[min_i]);
		//transform back
		collision_normal = rotate_vector_with_quaternion(collision_normal, selection_box_rotation);
		//new point position
		new_position.xyz = position.xyz+collision_normal*dist_to_boundary[min_i];
		new_position.w = 1.0;
		return true;
	}
	return false;
}

bool update_range(in float lb, in float ub, in float o, in float d, in uint i, out uint i_min, out uint i_max, out float t_min, out float t_max, in float epsilon) {
	// check for case where ray component is parallel to plane slab
	if (abs(d) < epsilon)
		return o >= lb && o <= ub;
	// compute intersections
	float f = 1.0 / d;
	float t0 = f * (lb - o);
	float t1 = f * (ub - o);
	if (t0 > t1) {
		float tmp = t0;
		t1 = t0;
		t0 = t1;
	}

	// update interval
	if (t0 > t_min) {
		i_min = i;
		t_min = t0;
	}
	if (t1 < t_max) {
		i_max = i;
		t_max = t1;
	}
	return true;
}

bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result){
	vec3 box_space_pos = inverse_rotate_vector_with_quaternion(origin.xyz-selection_box_translation.xyz, selection_box_rotation);
	const float epsilon = 0.0000001;
	//ub, lb -> aabb_min_p, aabb_max_p
	float t_min = -infinity;
	float t_max = infinity;

	uint i_min, i_max;
	if (!( update_range(aabb_min_p.x, aabb_max_p.x, origin.x, direction.x, 0, i_min, i_max, t_min, t_max, epsilon) &&
		update_range(aabb_min_p.y, aabb_max_p.y, origin.y, direction.y, 1, i_min, i_max, t_min, t_max, epsilon) &&
		update_range(aabb_min_p.z, aabb_max_p.z, origin.z, direction.z, 2, i_min, i_max, t_min, t_max, epsilon) )) {
		return false;
	}
	
	if (t_max < 0.0 || t_min > t_max)
		return false;

	if (t_min < 0.0) {
		t_result = t_max;
		i_min = i_max;
	}
	else
		t_result = t_min;

	//p_result = origin + t_result * direction;		
	return true;
}