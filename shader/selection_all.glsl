#version 450

/*
//***** begin interface selection_xxxxxx.glsl
bool is_in_selection(in vec4 view_position);
bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result);
bool resolve_collision(in vec4 position, out vec4 new_position, out vec3 collision_normal);
//***** end interface selection_xxxxxx.glsl *********************************
*/

bool is_in_selection(in vec4 view_position){
	return true;
}

bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result){
	t_result = 0;
	return true;
}

bool resolve_collision(in vec4 position, out vec4 new_position, out vec3 collision_normal){
	return false;
}