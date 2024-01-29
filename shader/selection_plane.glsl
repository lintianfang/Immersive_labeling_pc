#version 450

uniform vec4 selection_plane_normal;
uniform vec4 selection_plane_origin;

/*
//***** begin interface selection_xxxxxx.glsl
bool is_in_selection(in vec4 position);
//***** end interface selection_xxxxxx.glsl *********************************
*/

bool is_in_selection(in vec4 position){
	return dot(position.xyz-selection_plane_origin.xyz,selection_plane_normal.xyz) <= 0;
}

bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result){
	//not implemented
	return false;
}

bool resolve_collision(in vec4 position, out vec4 new_position, out vec3 collision_normal){
	//not implemented
	return false;
}